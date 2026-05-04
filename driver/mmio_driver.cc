// Copyright 2019 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "driver/mmio_driver.h"

#include <ctime>
#include <functional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "api/buffer.h"
#include "api/watchdog.h"
#include "driver/config/common_csr_helper.h"
#include "driver/config/register_constants.h"
#include "driver/device_buffer.h"
#include "driver/device_buffer_mapper.h"
#include "driver/dma_info_extractor.h"
#include "driver/hardware_structures.h"
#include "driver/interrupt/interrupt_controller_interface.h"
#include "driver/interrupt/interrupt_handler.h"
#include "driver/interrupt/top_level_interrupt_manager.h"
#include "driver/memory/address_utilities.h"
#include "driver/memory/mmu_mapper.h"
#include "driver/mmio/host_queue.h"
#include "driver/package_registry.h"
#include "driver/single_tpu_request.h"
#include "driver/top_level_handler.h"
#include "driver/tpu_request.h"
#include "driver_shared/time_stamper/driver_time_stamper.h"
#include "driver_shared/time_stamper/time_stamper.h"
#include "executable/executable_generated.h"
#include "port/cleanup.h"
#include "port/errors.h"
#include "port/integral_types.h"
#include "port/logging.h"
#include "port/ptr_util.h"
#include "port/status.h"
#include "port/status_macros.h"
#include "port/statusor.h"
#include "port/std_mutex_lock.h"
#include "port/stringprintf.h"
#include "port/tracing.h"

namespace platforms {
namespace darwinn {
namespace driver {

namespace {

// Indicates no HIB Fatal Error.
constexpr uint64 kHibErrorStatusNone = 0;

}  // namespace

MmioDriver::MmioDriver(
    const api::DriverOptions& driver_options,
    std::unique_ptr<config::ChipConfig> chip_config,
    std::unique_ptr<Registers> registers,
    std::unique_ptr<DramAllocator> dram_allocator,
    std::unique_ptr<MmuMapper> mmu_mapper,
    std::unique_ptr<AddressSpace> address_space,
    std::unique_ptr<Allocator> allocator,
    std::unique_ptr<HostQueue<HostQueueDescriptor, HostQueueStatusBlock>>
        instruction_queue,
    std::unique_ptr<InterruptHandler> interrupt_handler,
    std::unique_ptr<TopLevelInterruptManager> top_level_interrupt_manager,
    std::unique_ptr<InterruptControllerInterface>
        fatal_error_interrupt_controller,
    std::unique_ptr<ScalarCoreController> scalar_core_controller,
    std::unique_ptr<RunController> run_controller,
    std::unique_ptr<TopLevelHandler> top_level_handler,
    std::unique_ptr<PackageRegistry> executable_registry,
    std::unique_ptr<driver_shared::TimeStamper> time_stamper)
    : Driver(
          [](config::ChipConfig* chip_config) {
            CHECK(chip_config != nullptr);
            return chip_config->GetChip();
          }(chip_config.get()),
          std::move(executable_registry), driver_options,
          std::move(time_stamper)),
      hib_user_csr_offsets_(chip_config->GetHibUserCsrOffsets()),
      hib_kernel_csr_offsets_(chip_config->GetHibKernelCsrOffsets()),
      chip_structure_(chip_config->GetChipStructures()),
      registers_(std::move(registers)),
      dram_allocator_(std::move(dram_allocator)),
      mmu_mapper_(std::move(mmu_mapper)),
      address_space_(std::move(address_space)),
      allocator_(std::move(allocator)),
      instruction_queue_(std::move(instruction_queue)),
      interrupt_handler_(std::move(interrupt_handler)),
      top_level_interrupt_manager_(std::move(top_level_interrupt_manager)),
      fatal_error_interrupt_controller_(
          std::move(fatal_error_interrupt_controller)),
      scalar_core_controller_(std::move(scalar_core_controller)),
      run_controller_(std::move(run_controller)),
      top_level_handler_(std::move(top_level_handler)),
      dma_info_extractor_(DmaInfoExtractor::ExtractorType::kInstructionDma),
      // TODO : Check reusing driver time_stamper for scheduler.
      dma_scheduler_(api::Watchdog::MakeWatchdog(
                         driver_options.watchdog_timeout_ns(),
                         [this](int64) { HandleWatchdogTimeout(); }),
                     gtl::MakeUnique<driver_shared::DriverTimeStamper>()),
      chip_config_(std::move(chip_config)) {}

MmioDriver::~MmioDriver() {
  CHECK_OK(UnregisterAll());
  if (Close(api::Driver::ClosingMode::kGraceful).ok()) {
    LOG(WARNING) << "Driver destroyed when open. Forced Close().";
  }
}

Status MmioDriver::ValidateState(State expected_state) const {
  if (state_ != expected_state) {
    return FailedPreconditionError(
        StringPrintf("Bad MMIO driver state. expected=%d, actual=%d.",
                     expected_state, state_));
  }
  return Status();  // OK
}

Status MmioDriver::SetState(State next_state) {
  switch (state_) {
    case kOpen:
      if (next_state == kClosing) {
        state_ = next_state;
        return Status();  // OK
      }
      break;

    case kClosing:
      if (next_state == kClosed) {
        state_ = next_state;
        return Status();  // OK
      }
      break;

    case kClosed:
      if (next_state == kOpen) {
        state_ = next_state;
        return Status();  // OK
      }
      break;
  }

  // Illegal state transition.
  return FailedPreconditionError(StringPrintf(
      "Invalid state transition. current=%d, next=%d.", state_, next_state));
}

Status MmioDriver::RegisterAndEnableAllInterrupts() {
  // Instruction queue completion.
  RETURN_IF_ERROR(interrupt_handler_->Register(
      DW_INTERRUPT_INSTR_QUEUE,
      std::bind(&HostQueue<HostQueueDescriptor,
                           HostQueueStatusBlock>::ProcessStatusBlock,
                instruction_queue_.get())));

  // Execution completions.
  RETURN_IF_ERROR(
      interrupt_handler_->Register(DW_INTERRUPT_SC_HOST_0, [this]() {
        // We need to clear the interrupts _before_ both:
        // -  reading interrupt counts, otherwise the device may concurrently
        //    increment interrupt count without signaling an interrupt. Driver
        //    can miss the completion event in this case.
        // -  calling HandleExecutionCompletion() because that may put the
        //    device in clock gated mode, which causes CSR access to be
        //    rejected.
        CHECK_OK(scalar_core_controller_->ClearInterruptStatus(0));

        auto count_result = scalar_core_controller_->CheckInterruptCounts(0);
        CHECK_OK(count_result.status());
        uint64 count = count_result.ValueOrDie();
        for (int i = 0; i < count; ++i) {
          HandleExecutionCompletion();
        }
      }));

  // Clear status for other scalar core interrupts.
  RETURN_IF_ERROR(
      interrupt_handler_->Register(DW_INTERRUPT_SC_HOST_1, [this]() {
        CHECK_OK(scalar_core_controller_->ClearInterruptStatus(1));
      }));
  RETURN_IF_ERROR(
      interrupt_handler_->Register(DW_INTERRUPT_SC_HOST_2, [this]() {
        CHECK_OK(scalar_core_controller_->ClearInterruptStatus(2));
      }));
  RETURN_IF_ERROR(
      interrupt_handler_->Register(DW_INTERRUPT_SC_HOST_3, [this]() {
        CHECK_OK(scalar_core_controller_->ClearInterruptStatus(3));
      }));

  // Top level interrupts.
  for (int i = 0; i < top_level_interrupt_manager_->NumInterrupts(); ++i) {
    RETURN_IF_ERROR(interrupt_handler_->Register(
        static_cast<Interrupt>(DW_INTERRUPT_TOP_LEVEL_BASE + i), [this, i]() {
          LOG(WARNING) << StringPrintf("Top level interrupt: %d", i);
          CHECK_OK(top_level_interrupt_manager_->HandleInterrupt(i));
        }));
  }

  // HIB Errors.
  RETURN_IF_ERROR(
      interrupt_handler_->Register(DW_INTERRUPT_FATAL_ERR, [this]() {
        // Fatal Error is sticky when raised. Once fatal error is raised,
        // disable first and then clear interrupts. Note that it is still
        // possible for this function to be called multiple times when fatal
        // error is raised because of the host side delay involved in disabling
        // and clearing the interrupts. This is handle inside CheckFatalError().
        CHECK_OK(fatal_error_interrupt_controller_->DisableInterrupts());
        CHECK_OK(fatal_error_interrupt_controller_->ClearInterruptStatus(0));
        CheckFatalError(CheckHibError());
      }));

  // Enable interrupts, if needed.
  LOG(INFO) << "[INIT][RegInts] scalar_core_controller->EnableInterrupts() "
               "(sc_host_int_control)";
  RETURN_IF_ERROR(scalar_core_controller_->EnableInterrupts());
  LOG(INFO) << "[INIT][RegInts] instruction_queue->EnableInterrupts() "
               "(queue_int_control)";
  RETURN_IF_ERROR(instruction_queue_->EnableInterrupts());
  LOG(INFO) << "[INIT][RegInts] fatal_error_interrupt_controller->EnableInterrupts() "
               "(fatal_err_int_control)";
  RETURN_IF_ERROR(fatal_error_interrupt_controller_->EnableInterrupts());

  // TODO: refactor for Darwinn 1.0 vs 2.0 driver.
  LOG(INFO) << "[INIT][RegInts] top_level_interrupt_manager->EnableInterrupts() "
               "(thermal/mbist/pcie_err/thermal_shutdown)";
  RETURN_IF_ERROR(top_level_interrupt_manager_->EnableInterrupts());

  LOG(INFO) << "[INIT][RegInts] END";
  return Status();  // OK
}

Status MmioDriver::CheckHibError() {
  ASSIGN_OR_RETURN(uint64 hib_error_status,
                   registers_->Read(hib_user_csr_offsets_.hib_error_status));
  if (hib_error_status == kHibErrorStatusNone) {
    return Status();  // OK
  }

  uint64 hib_first_error_status =
      registers_->Read(hib_user_csr_offsets_.hib_first_error_status)
          .ValueOrDie();

  auto error_string = StringPrintf(
      "HIB Error. hib_error_status = %016llx, hib_first_error_status = %016llx",
      static_cast<unsigned long long>(hib_error_status),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(                    // NOLINT(runtime/int)
          hib_first_error_status));
  LOG(ERROR) << error_string;
  return InternalError(error_string);
}

Status MmioDriver::DoOpen(bool debug_mode) {
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] BEGIN debug_mode=" << debug_mode;
  StdMutexLock state_lock(&state_mutex_);
  RETURN_IF_ERROR(ValidateState(/*expected_state=*/kClosed));

  // Register Access.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=registers->Open()";
  RETURN_IF_ERROR(registers_->Open());
  auto registers_closer =
      MakeCleanup([this] { CHECK_OK(registers_->Close()); });

  // Reset Handler - Manages power state of the chip.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=top_level_handler->Open()";
  RETURN_IF_ERROR(top_level_handler_->Open());
  auto top_level_handler_closer =
      MakeCleanup([this] { CHECK_OK(top_level_handler_->Close()); });

  // LPM power up core
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=LpmCoreToActive()";
  RETURN_IF_ERROR(top_level_handler_->LpmCoreToActive());

  // Disable clock gate and reset GCB for clean state.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=DisableSoftwareClockGate()";
  RETURN_IF_ERROR(top_level_handler_->DisableSoftwareClockGate());
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=DisableHardwareClockGate()";
  RETURN_IF_ERROR(top_level_handler_->DisableHardwareClockGate());
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=EnableReset()";
  RETURN_IF_ERROR(top_level_handler_->EnableReset());

  // Quit from reset mode.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=QuitReset()";
  RETURN_IF_ERROR(top_level_handler_->QuitReset());
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=EnableHardwareClockGate()";
  RETURN_IF_ERROR(top_level_handler_->EnableHardwareClockGate());

  // HIB should be good to start with.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=CheckHibError()";
  RETURN_IF_ERROR(CheckHibError());

  // Limit AXI DMA burst.
  if (hib_user_csr_offsets_.dma_burst_limiter !=
      kCsrRegisterSpaceInvalidOffset) {
    LOG(INFO) << StringPrintf(
        "[INIT][MmioDriver::DoOpen] dma_burst_limiter (hib_user) @0x%05llx "
        "write=0x%016llx",
        (unsigned long long)hib_user_csr_offsets_.dma_burst_limiter,
        (unsigned long long)chip_structure_.axi_dma_burst_limiter);
    RETURN_IF_ERROR(registers_->Write(hib_user_csr_offsets_.dma_burst_limiter,
                                      chip_structure_.axi_dma_burst_limiter));
  } else {
    LOG(INFO) << StringPrintf(
        "[INIT][MmioDriver::DoOpen] dma_burst_limiter (hib_kernel) @0x%05llx "
        "write=0x%016llx",
        (unsigned long long)hib_kernel_csr_offsets_.dma_burst_limiter,
        (unsigned long long)chip_structure_.axi_dma_burst_limiter);
    RETURN_IF_ERROR(registers_->Write(hib_kernel_csr_offsets_.dma_burst_limiter,
                                      chip_structure_.axi_dma_burst_limiter));
  }

  // MMU Access.
  const int num_simple_entries =
      GetNumSimplePageTableEntries(chip_structure_.num_page_table_entries);
  LOG(INFO) << StringPrintf(
      "[INIT][MmioDriver::DoOpen] phase=mmu_mapper->Open() "
      "num_simple_entries=%d num_page_table_entries=%llu",
      num_simple_entries,
      (unsigned long long)chip_structure_.num_page_table_entries);

  RETURN_IF_ERROR(mmu_mapper_->Open(num_simple_entries));
  auto mmu_mapper_closer =
      MakeCleanup([this] { CHECK_OK(mmu_mapper_->Close()); });

  // Interrupt Handler.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=interrupt_handler->Open()";
  RETURN_IF_ERROR(interrupt_handler_->Open());
  auto interrupt_handler_closer =
      MakeCleanup([this] { CHECK_OK(interrupt_handler_->Close()); });

  // Instruction Queue Access.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=instruction_queue->Open()";
  RETURN_IF_ERROR(instruction_queue_->Open(address_space_.get()));
  auto instruction_queue_closer =
      MakeCleanup([this] { CHECK_OK(instruction_queue_->Close()); });

  // Scalar core control.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=scalar_core_controller->Open()";
  RETURN_IF_ERROR(scalar_core_controller_->Open());
  auto scalar_core_controller_closer =
      MakeCleanup([this] { CHECK_OK(scalar_core_controller_->Close()); });

  if (!debug_mode) {
    // Move all subsystems to Run state.
    LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=DoRunControl(kMoveToRun)";
    RETURN_IF_ERROR(run_controller_->DoRunControl(RunControl::kMoveToRun));
  } else {
    LOG(INFO) << "[INIT][MmioDriver::DoOpen] debug_mode=true — skip DoRunControl";
  }

  // TODO: refactor for Darwinn 1.0 vs 2.0 driver.

  if (hib_user_csr_offsets_.status_block_update !=
      kCsrRegisterSpaceInvalidOffset) {
    // Disable periodic status block updates.
    LOG(INFO) << StringPrintf(
        "[INIT][MmioDriver::DoOpen] status_block_update @0x%05llx write=0 "
        "(disable periodic SB updates)",
        (unsigned long long)hib_user_csr_offsets_.status_block_update);
    RETURN_IF_ERROR(
        registers_->Write(hib_user_csr_offsets_.status_block_update, 0));
  }

  // Register and enable all interrupts.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=RegisterAndEnableAllInterrupts()";
  RETURN_IF_ERROR(RegisterAndEnableAllInterrupts());

  // DMA scheduler.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=dma_scheduler.Open()";
  RETURN_IF_ERROR(dma_scheduler_.Open());
  auto dma_scheduler_closer = MakeCleanup([this] {
    CHECK_OK(dma_scheduler_.Close(api::Driver::ClosingMode::kGraceful));
  });

  // On-Chip DRAM allocator.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=dram_allocator->Open()";
  RETURN_IF_ERROR(dram_allocator_->Open());

  // Errata registers.
  // TODO: refactor for Darwinn 1.0 vs 2.0 driver.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=FixErrata()";
  RETURN_IF_ERROR(FixErrata());

  // All good. Move state to open.
  RETURN_IF_ERROR(SetState(kOpen));

  // Clock gate until the first request arrives.
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] phase=EnableSoftwareClockGate() "
               "(idle until first request)";
  RETURN_IF_ERROR(top_level_handler_->EnableSoftwareClockGate());
  LOG(INFO) << "[INIT][MmioDriver::DoOpen] END (state=kOpen)";

  // Release cleanup functions.
  dma_scheduler_closer.release();
  scalar_core_controller_closer.release();
  interrupt_handler_closer.release();
  instruction_queue_closer.release();
  mmu_mapper_closer.release();
  top_level_handler_closer.release();
  registers_closer.release();

  return Status();  // OK
}

Status MmioDriver::DoClose(bool in_error, api::Driver::ClosingMode mode) {
  StdMutexLock state_lock(&state_mutex_);
  RETURN_IF_ERROR(ValidateState(/*expected_state=*/kOpen));

  // Note our intention to close.
  RETURN_IF_ERROR(SetState(kClosing));

  // Disable Clock Gating so as the closing procedure can access the chip
  RETURN_IF_ERROR(top_level_handler_->DisableSoftwareClockGate());

  // All good. Shut down stuff. This is best effort. So if things starts
  // failing, keep going and try cleaning up as much as we can.
  Status status;

  // Pause all DMAs and wait for that to happen in the hardware otherwise we
  // will be at risk of getting into undefined behavior in the following
  // steps.
  RETURN_IF_ERROR(PauseAllDmas());

  // Stop all pipelines.
  status.Update(run_controller_->DoRunControl(RunControl::kMoveToHalt));

  // Disable all interrupts.
  status.Update(top_level_interrupt_manager_->DisableInterrupts());
  status.Update(fatal_error_interrupt_controller_->DisableInterrupts());

  status.Update(instruction_queue_->DisableInterrupts());
  status.Update(scalar_core_controller_->DisableInterrupts());

  // We have to close interrupt handler before host queue especially for ASAP
  // closing. Otherwise we may get interrupts that result in an Enqueue in host
  // queue while it is closed.
  status.Update(interrupt_handler_->Close(
      in_error || mode == api::Driver::ClosingMode::kAsap));

  status.Update(scalar_core_controller_->Close());
  status.Update(instruction_queue_->Close(
      in_error || mode == api::Driver::ClosingMode::kAsap));

  // Begin shutdown.
  status.Update(dma_scheduler_.Close(mode));
  status.Update(UnmapAllParameters());
  status.Update(mmu_mapper_->Close());
  status.Update(top_level_handler_->EnableReset());

  // LPM rail gate core
  status.Update(top_level_handler_->LpmCoreToRailGate());

  status.Update(top_level_handler_->Close());
  status.Update(registers_->Close());
  status.Update(dram_allocator_->Close());
  RETURN_IF_ERROR(status);

  // Finalize.
  RETURN_IF_ERROR(SetState(kClosed));

  return Status();  // OK
}

Status MmioDriver::DoCancelAndWaitRequests(bool in_error) {
  StdMutexLock state_lock(&state_mutex_);
  RETURN_IF_ERROR(dma_scheduler_.CancelPendingRequests());
  if (!in_error) {
    RETURN_IF_ERROR(dma_scheduler_.WaitActiveRequests());
  }
  return Status();  // OK
}

Buffer MmioDriver::DoMakeBuffer(size_t size_bytes) const {
  return allocator_->MakeBuffer(size_bytes);
}

StatusOr<MappedDeviceBuffer> MmioDriver::DoMapBuffer(const Buffer& buffer,
                                                     DmaDirection direction) {
  if (buffer.IsValid()) {
    // ★ user-data buffer 는 항상 EXTENDED 영역에 매핑되도록 hardcode.
    //    instruction / input / output / parameter 모두 여기 통과.
    LOG(INFO) << StringPrintf(
        "[DoMapBuffer] user buffer (size=%zu B) → forcing kExtended hint",
        buffer.size_bytes());
    ASSIGN_OR_RETURN(auto device_buffer,
                     address_space_->MapMemory(buffer, direction,
                                               MappingTypeHint::kExtended));
    // TODO : this is dangerous: the std::bind captures a raw pointer to
    // the address space. This will break if executable registry outlives
    // address space in the driver. A better way is to at least use share_ptr
    // for address spaces, and here let the std::bind capture a weak_ptr.
    return MappedDeviceBuffer(
        device_buffer, std::bind(&AddressSpace::UnmapMemory,
                                 address_space_.get(), std::placeholders::_1));
  }
  return MappedDeviceBuffer();
}

StatusOr<std::shared_ptr<TpuRequest>> MmioDriver::DoCreateRequest(
    const std::shared_ptr<Request> parent_request,
    const ExecutableReference* executable, TpuRequest::RequestType type) {
  TRACE_SCOPE("MmioDriver::DoCreateRequest");
  StdMutexLock lock(&state_mutex_);
  RETURN_IF_ERROR(ValidateState(kOpen));
  return {std::make_shared<SingleTpuRequest>(
      next_id_++, parent_request, executable, allocator_.get(),
      dram_allocator_.get(),
      gtl::MakeUnique<DeviceBufferMapper>(address_space_.get()),
      &dma_info_extractor_, chip_structure_.minimum_alignment_bytes, type)};
}

Status MmioDriver::DoSubmit(std::shared_ptr<TpuRequest> request) {
  TRACE_SCOPE("MmioDriver::DoSubmit");
  StdMutexLock state_lock(&state_mutex_);
  RETURN_IF_ERROR(ValidateState(kOpen));

  // Disables Clock Gating so as the chip is accessible while the request
  // is built.
  RETURN_IF_ERROR(top_level_handler_->DisableSoftwareClockGate());

  DumpRunStatus("DoSubmit-pre-prepare");

  // Validate and prepare the request.
  RETURN_IF_ERROR(request->Validate());
  RETURN_IF_ERROR(request->Prepare());

  RETURN_IF_ERROR(dma_scheduler_.Submit(std::move(request)));

  TRACE_WITHIN_SCOPE("MmioDriver::DoSubmit::Issue");
  RETURN_IF_ERROR(TryIssueDmas());

  DumpRunStatus("DoSubmit-post-issue");

  return Status();  // OK
}

Status MmioDriver::TryIssueDmas() {
  TRACE_SCOPE("MmioDriver::TryIssueDmas");
  // Both the dma_scheduler and instruction_queue is threadsafe on its own.
  // However, we also want to to make sure that DMAs popped from the dma
  // scheduler are pushed to the instruction queue in the order it is received.
  // So do the following with the dma_issue_mutex held.
  StdMutexLock state_lock(&dma_issue_mutex_);

  CHECK_OK(top_level_handler_->DisableSoftwareClockGate());

  while (instruction_queue_->GetAvailableSpace() > 0) {
    ASSIGN_OR_RETURN(auto* dma, dma_scheduler_.GetNextDma());
    if (dma == nullptr) {
      break;
    }
    CHECK(dma->type() == DmaDescriptorType::kInstruction);

    HostQueueDescriptor descriptor{};
    descriptor.address = dma->buffer().device_address();
    descriptor.size_in_bytes = dma->buffer().size_bytes();

    // Enqueue should always succeed.
    CheckFatalError(
        instruction_queue_->Enqueue(descriptor, [this, dma](uint32 error_code) {
          CHECK_OK(dma_scheduler_.NotifyDmaCompletion(dma));
          HandleHostQueueCompletion(error_code);
        }));

    TRACE_WITHIN_SCOPE("MmioDriver::TryIssueDmas::Enqueue");
  }

  return OkStatus();
}

void MmioDriver::HandleExecutionCompletion() {
  TRACE_SCOPE("MmioDriver::HandleExecutionCompletion");
  CHECK_OK(dma_scheduler_.NotifyRequestCompletion());
  HandleTpuRequestCompletion();
  if (dma_scheduler_.IsEmpty()) {
    CHECK_OK(top_level_handler_->EnableSoftwareClockGate());
  }
}

void MmioDriver::HandleHostQueueCompletion(uint32 error_code) {
  TRACE_SCOPE("MmioDriver::HostQueueCompletion");
  if (error_code != 0) {
    // TODO: Parse the error code and attach a human readable string.
    CheckFatalError(
        InternalError(StringPrintf("Host Queue error %d.", error_code)));
    return;
  }
  CHECK_OK(TryIssueDmas());
}

void MmioDriver::CheckFatalError(const Status& status) {
  if (status.ok()) {
    return;
  }
  NotifyFatalError(status);
}

Status MmioDriver::DoSetRealtimeMode(bool on) {
  dma_scheduler_.SetRealtimeMode(on);
  return OkStatus();
}

Status MmioDriver::PauseAllDmas() {
  constexpr uint64 kPauseDmas = 1;
  RETURN_IF_ERROR(
      registers_->Write(hib_user_csr_offsets_.dma_pause, kPauseDmas));
  constexpr uint64 kAllDmasPaused = 1;
  return registers_->Poll(hib_user_csr_offsets_.dma_paused, kAllDmasPaused);
}

Status MmioDriver::FixErrata() {
  return OkStatus();
}

void MmioDriver::DumpRunStatus(const char* tag) {
  // Authoritative offsets from
  //   driver/config/beagle/beagle_csr_offsets.h
  //   - avDataPopRunStatus     0x44168
  //   - parameterPopRunStatus  0x441a8
  //   - infeedRunStatus        0x441e0
  //   - outfeedRunStatus       0x44220
  //   - scalarCoreRunStatus    0x44258
  // Engine state encoding seen in libedgetpu enums: 0=kIdle 1=kRunning
  // 4=kHalted (other values exist on errata paths).
  auto safe_read = [this](uint64 off) -> uint64 {
    auto r = registers_->Read(off);
    if (!r.ok()) return 0xDEADBEEFDEADBEEFULL;
    return r.ValueOrDie();
  };

  const uint64 av     = safe_read(0x44168);
  const uint64 param  = safe_read(0x441a8);
  const uint64 infeed = safe_read(0x441e0);
  const uint64 outf   = safe_read(0x44220);
  const uint64 scalar = safe_read(0x44258);

  LOG(INFO) << StringPrintf(
      "[RUN-STATUS@%s] SCALAR@0x44258=%llu AVDATA@0x44168=%llu "
      "PARAM@0x441a8=%llu INFEED@0x441e0=%llu OUTFEED@0x44220=%llu",
      tag,
      static_cast<unsigned long long>(scalar),
      static_cast<unsigned long long>(av),
      static_cast<unsigned long long>(param),
      static_cast<unsigned long long>(infeed),
      static_cast<unsigned long long>(outf));

  // PT-related CSRs — used by npu_driver to verify boundary + translation state
  // matches what the chip thinks it is right at submit time.  These are mmap'd
  // (BAR2+0x46000 region in our kernel_registers.cc setup) so safe_read works.
  //   page_table_size               @0x46000  (libedgetpu writes 8192 at init)
  //   extended_table (boundary)     @0x46008  (libedgetpu writes 6144 at init)
  //   kernel_hib_translation_enable @0x46010  (POR=1 expected)
  //   infeed_page_fault_address     @0x48738  (latched VA on page fault, 0=clean)
  //   user_hib_error_status         @0x486f0  (any HIB-side fault flag)
  //   instruction_queue_int_status  @0x485c8  (W1C; read shows pending IQ-int)
  const uint64 ptsize = safe_read(0x46000);
  const uint64 ptbnd  = safe_read(0x46008);
  const uint64 trxen  = safe_read(0x46010);
  const uint64 ifflt  = safe_read(0x48738);
  const uint64 hiberr = safe_read(0x486f0);
  const uint64 iqstat = safe_read(0x485c8);
  LOG(INFO) << StringPrintf(
      "[PT-CSR@%s] page_table_size=%llu extended_table=%llu translation_en=%llu "
      "infeed_fault_va=0x%llx hib_err=0x%llx iq_int_status=0x%llx",
      tag,
      static_cast<unsigned long long>(ptsize),
      static_cast<unsigned long long>(ptbnd),
      static_cast<unsigned long long>(trxen),
      static_cast<unsigned long long>(ifflt),
      static_cast<unsigned long long>(hiberr),
      static_cast<unsigned long long>(iqstat));
}

}  // namespace driver
}  // namespace darwinn
}  // namespace platforms
