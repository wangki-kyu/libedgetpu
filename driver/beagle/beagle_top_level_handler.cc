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

#include "driver/beagle/beagle_top_level_handler.h"

#include "driver/config/beagle_csr_helper.h"
#include "driver/config/common_csr_helper.h"
#include "driver/registers/registers.h"
#include "port/errors.h"
#include "port/integral_types.h"
#include "port/logging.h"
#include "port/status_macros.h"
#include "port/stringprintf.h"

namespace platforms {
namespace darwinn {
namespace driver {

namespace {

using config::registers::ScuCtrl3;

}  // namespace

BeagleTopLevelHandler::BeagleTopLevelHandler(
    const config::ChipConfig& config, Registers* registers, bool use_usb,
    api::PerformanceExpectation performance)
    : cb_bridge_offsets_(config.GetCbBridgeCsrOffsets()),
      hib_user_offsets_(config.GetHibUserCsrOffsets()),
      misc_offsets_(config.GetMiscCsrOffsets()),
      reset_offsets_(config.GetScuCsrOffsets()),
      scalar_core_offsets_(config.GetScalarCoreCsrOffsets()),
      tile_config_offsets_(config.GetTileConfigCsrOffsets()),
      tile_offsets_(config.GetTileCsrOffsets()),
      registers_(registers),
      performance_(performance),
      use_usb_(use_usb) {
  CHECK(registers != nullptr);
}

Status BeagleTopLevelHandler::Open() {
  LOG(INFO) << "[INIT][TopLevel::Open] BEGIN";
  // By reading top level registers, figure out whether chip is in clock gated
  // mode.
  software_clock_gated_ = false;
  hardware_clock_gated_ = false;

  // 1. Always disable inactive mode.
  // Read the register to preserve other fields.
  ASSIGN_OR_RETURN(uint32 scu_ctrl_0_reg,
                   registers_->Read32(reset_offsets_.scu_ctrl_0));
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::Open] scu_ctrl_0 @0x%05llx read=0x%08x",
      (unsigned long long)reset_offsets_.scu_ctrl_0, scu_ctrl_0_reg);
  config::registers::ScuCtrl0 helper(scu_ctrl_0_reg);
  helper.set_rg_pcie_inact_phy_mode(0);
  helper.set_rg_usb_inact_phy_mode(0);
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::Open] scu_ctrl_0 @0x%05llx write=0x%08x "
      "(rg_pcie_inact_phy_mode=0, rg_usb_inact_phy_mode=0)",
      (unsigned long long)reset_offsets_.scu_ctrl_0, helper.raw());
  RETURN_IF_ERROR(registers_->Write32(reset_offsets_.scu_ctrl_0, helper.raw()));

  // 2. Check "rg_gated_gcb".
  // 0x0: deprecated
  // 0x1: hardware clock gated
  // 0x2: no clock gating
  ASSIGN_OR_RETURN(uint32 scu_ctrl_2_reg,
                   registers_->Read32(reset_offsets_.scu_ctrl_2));
  config::registers::ScuCtrl2 scu_ctrl_2(scu_ctrl_2_reg);
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::Open] scu_ctrl_2 @0x%05llx read=0x%08x rg_gated_gcb=%u "
      "(0=deprecated 1=hw_gated 2=no_gating)",
      (unsigned long long)reset_offsets_.scu_ctrl_2, scu_ctrl_2_reg,
      scu_ctrl_2.rg_gated_gcb());
  if (scu_ctrl_2.rg_gated_gcb() == 0x1) {
    hardware_clock_gated_ = true;
  }

  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::Open] END hardware_clock_gated=%d software_clock_gated=%d",
      hardware_clock_gated_, software_clock_gated_);
  return Status();  // OK
}

Status BeagleTopLevelHandler::QuitReset() {
  LOG(INFO) << "[INIT][TopLevel::QuitReset] BEGIN";
  // Disable Sleep Mode (Partial Software Control)
  // 1. Make "rg_force_sleep" to be b10. Read the register to preserve other
  // fields.
  // 2. Set GCB, AXI, and 8051 clock rate according to desired performance
  // level.
  ASSIGN_OR_RETURN(uint32 scu_ctrl_3_reg,
                   registers_->Read32(reset_offsets_.scu_ctrl_3));
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::QuitReset] scu_ctrl_3 @0x%05llx read=0x%08x",
      (unsigned long long)reset_offsets_.scu_ctrl_3, scu_ctrl_3_reg);
  config::registers::ScuCtrl3 helper(scu_ctrl_3_reg);
  helper.set_rg_force_sleep(0b10);

  switch (performance_) {
    case api::PerformanceExpectation_Low:
      helper.set_gcb_clock_rate(ScuCtrl3::GcbClock::k63MHZ);
      helper.set_axi_clock_rate(ScuCtrl3::AxiClock::k125MHZ);
      helper.set_usb_8051_clock_rate(ScuCtrl3::Usb8051Clock::k250MHZ);
      break;

    case api::PerformanceExpectation_Medium:
      helper.set_gcb_clock_rate(ScuCtrl3::GcbClock::k125MHZ);
      helper.set_axi_clock_rate(ScuCtrl3::AxiClock::k125MHZ);
      if (use_usb_) {
        helper.set_usb_8051_clock_rate(ScuCtrl3::Usb8051Clock::k500MHZ);
      } else {
        helper.set_usb_8051_clock_rate(ScuCtrl3::Usb8051Clock::k250MHZ);
      }
      break;

    case api::PerformanceExpectation_High:
      helper.set_gcb_clock_rate(ScuCtrl3::GcbClock::k250MHZ);
      helper.set_axi_clock_rate(ScuCtrl3::AxiClock::k125MHZ);
      if (use_usb_) {
        helper.set_usb_8051_clock_rate(ScuCtrl3::Usb8051Clock::k500MHZ);
      } else {
        helper.set_usb_8051_clock_rate(ScuCtrl3::Usb8051Clock::k250MHZ);
      }
      break;

    case api::PerformanceExpectation_Max:
      helper.set_gcb_clock_rate(ScuCtrl3::GcbClock::k500MHZ);
      if (use_usb_) {
        helper.set_usb_8051_clock_rate(ScuCtrl3::Usb8051Clock::k500MHZ);
        helper.set_axi_clock_rate(ScuCtrl3::AxiClock::k250MHZ);
      } else {
        helper.set_usb_8051_clock_rate(ScuCtrl3::Usb8051Clock::k250MHZ);
        helper.set_axi_clock_rate(ScuCtrl3::AxiClock::k125MHZ);
      }
      break;

    default:
      return InvalidArgumentError(
          StringPrintf("Bad performance setting %d.", performance_));
  }

  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::QuitReset] scu_ctrl_3 @0x%05llx write=0x%08x "
      "(rg_force_sleep=0b10, performance=%d)",
      (unsigned long long)reset_offsets_.scu_ctrl_3, helper.raw(),
      (int)performance_);
  RETURN_IF_ERROR(registers_->Write32(reset_offsets_.scu_ctrl_3, helper.raw()));

  // 2. Poll until "cur_pwr_state" is 0x0. Other fields might change as well,
  // hence "cur_pwr_state" field has to be explicitly checked.
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::QuitReset] poll scu_ctrl_3.cur_pwr_state == 0x0");
  ASSIGN_OR_RETURN(scu_ctrl_3_reg,
                   registers_->Read32(reset_offsets_.scu_ctrl_3));
  helper.set_raw(scu_ctrl_3_reg);
  int pwr_poll_count = 0;
  while (helper.cur_pwr_state() != 0x0) {
    ASSIGN_OR_RETURN(scu_ctrl_3_reg,
                     registers_->Read32(reset_offsets_.scu_ctrl_3));
    helper.set_raw(scu_ctrl_3_reg);
    ++pwr_poll_count;
  }
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::QuitReset] cur_pwr_state=0x0 after %d polls",
      pwr_poll_count);

  // 3. Confirm that moved out of reset by reading any CSR with known initial
  // value. scalar core run control should be zero.
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::QuitReset] poll scalarCoreRunControl @0x%05llx == 0x0 "
      "(reset-release confirmation)",
      (unsigned long long)scalar_core_offsets_.scalarCoreRunControl);
  RETURN_IF_ERROR(
      registers_->Poll(scalar_core_offsets_.scalarCoreRunControl, 0));

  // 4. Enable idle register.
  config::registers::IdleRegister idle_reg;
  idle_reg.set_enable();
  idle_reg.set_counter(1);
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::QuitReset] idleRegister @0x%05llx write=0x%016llx "
      "(enable=1, counter=1)",
      (unsigned long long)misc_offsets_.idleRegister,
      (unsigned long long)idle_reg.raw());
  RETURN_IF_ERROR(
      registers_->Write(misc_offsets_.idleRegister, idle_reg.raw()));

  // 5. Update sleep/wake delay for tiles. toSleepDelay = 2, toWakeDelay = 30.
  // Broadcast to tiles.
  // TODO: helper uses 7-bits as defined by CSR. Extract bitwidth
  // automatically for different chips.
  config::registers::TileConfig<7> tile_config_reg;
  tile_config_reg.set_broadcast();
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::QuitReset] tileconfig0 @0x%05llx write=0x%016llx "
      "(broadcast)",
      (unsigned long long)tile_config_offsets_.tileconfig0,
      (unsigned long long)tile_config_reg.raw());
  RETURN_IF_ERROR(registers_->Write(tile_config_offsets_.tileconfig0,
                                    tile_config_reg.raw()));
  // Wait until tileconfig0 is set correctly. Subsequent writes are going to
  // tiles, but hardware does not guarantee correct ordering with previous
  // write.
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::QuitReset] poll tileconfig0 == 0x%016llx",
      (unsigned long long)tile_config_reg.raw());
  RETURN_IF_ERROR(registers_->Poll(tile_config_offsets_.tileconfig0,
                                   tile_config_reg.raw()));

  config::registers::DeepSleep deep_sleep_reg;
  deep_sleep_reg.set_to_sleep_delay(2);
  deep_sleep_reg.set_to_wake_delay(30);
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::QuitReset] deepSleep @0x%05llx write=0x%016llx "
      "(to_sleep_delay=2, to_wake_delay=30)",
      (unsigned long long)tile_offsets_.deepSleep,
      (unsigned long long)deep_sleep_reg.raw());
  RETURN_IF_ERROR(
      registers_->Write(tile_offsets_.deepSleep, deep_sleep_reg.raw()));

  LOG(INFO) << "[INIT][TopLevel::QuitReset] END";
  return Status();  // OK
}

Status BeagleTopLevelHandler::EnableReset() {
  LOG(INFO) << "[INIT][TopLevel::EnableReset] BEGIN";
  // If already in reset, skip reset. Otherwise, HIB CSR accesses will not be
  // valid.
  ASSIGN_OR_RETURN(uint32 scu_ctrl_3_reg,
                   registers_->Read32(reset_offsets_.scu_ctrl_3));
  config::registers::ScuCtrl3 helper(scu_ctrl_3_reg);
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::EnableReset] scu_ctrl_3 @0x%05llx read=0x%08x "
      "rg_force_sleep=%u cur_pwr_state=%u",
      (unsigned long long)reset_offsets_.scu_ctrl_3, scu_ctrl_3_reg,
      helper.rg_force_sleep(), helper.cur_pwr_state());
  if (helper.rg_force_sleep() == 0x3) {
    LOG(INFO) << "[INIT][TopLevel::EnableReset] already in reset (rg_force_sleep==0x3) — skip";
    return Status();  // OK
  }

  // Enable Sleep Mode (Partial Software Control).
  if (!use_usb_) {
    // Do Software Force GCB Idle.
    // Make sure all outstanding DMAs are drained. Note that USB does not have
    // to do step 1/2 as host controls DMAs.
    // 1. Enable DMA pause.
    LOG(INFO) << StringPrintf(
        "[INIT][TopLevel::EnableReset] dma_pause @0x%05llx write=1",
        (unsigned long long)hib_user_offsets_.dma_pause);
    RETURN_IF_ERROR(registers_->Write(hib_user_offsets_.dma_pause, 1));

    // 2. Wait until DMA is paused.
    LOG(INFO) << StringPrintf(
        "[INIT][TopLevel::EnableReset] poll dma_paused @0x%05llx == 1",
        (unsigned long long)hib_user_offsets_.dma_paused);
    RETURN_IF_ERROR(registers_->Poll(hib_user_offsets_.dma_paused, 1));
  }

  // Actual enable sleep mode.
  // 3. Set "rg_force_sleep" to 0x3. Read the register to preserve other fields.
  helper.set_rg_force_sleep(0x3);
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::EnableReset] scu_ctrl_3 @0x%05llx write=0x%08x "
      "(rg_force_sleep=0x3)",
      (unsigned long long)reset_offsets_.scu_ctrl_3, helper.raw());
  RETURN_IF_ERROR(registers_->Write32(reset_offsets_.scu_ctrl_3, helper.raw()));

  // 4. Poll until "cur_pwr_state" becomes 0x2. Other fields might change as
  // well, hence "cur_pwr_state" field has to be explicitly checked.
  LOG(INFO) << "[INIT][TopLevel::EnableReset] poll scu_ctrl_3.cur_pwr_state == 0x2";
  ASSIGN_OR_RETURN(scu_ctrl_3_reg,
                   registers_->Read32(reset_offsets_.scu_ctrl_3));
  helper.set_raw(scu_ctrl_3_reg);
  int pwr_poll_count = 0;
  while (helper.cur_pwr_state() != 0x2) {
    ASSIGN_OR_RETURN(scu_ctrl_3_reg,
                     registers_->Read32(reset_offsets_.scu_ctrl_3));
    helper.set_raw(scu_ctrl_3_reg);
    ++pwr_poll_count;
  }
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::EnableReset] cur_pwr_state=0x2 after %d polls",
      pwr_poll_count);

  // 5. Clear BULK credit by pulsing LSBs of "gcbb_credit0".
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::EnableReset] gcbb_credit0 @0x%05llx pulse 0xF -> 0x0 "
      "(clear stale BULK credits)",
      (unsigned long long)cb_bridge_offsets_.gcbb_credit0);
  RETURN_IF_ERROR(registers_->Write32(cb_bridge_offsets_.gcbb_credit0, 0xF));
  Status s = registers_->Write32(cb_bridge_offsets_.gcbb_credit0, 0x0);
  LOG(INFO) << "[INIT][TopLevel::EnableReset] END";
  return s;
}

Status BeagleTopLevelHandler::EnableHardwareClockGate() {
  LOG(INFO) << "[INIT][TopLevel::EnableHardwareClockGate] BEGIN";
  if (hardware_clock_gated_) {
    LOG(INFO) << "[INIT][TopLevel::EnableHardwareClockGate] already gated — skip";
    return Status();  // OK
  }

  // Enable Hardware Clock Gate (GCB)
  // 1. Write "rg_gated_gcb" to 0x1. Read the register to preserve other fields.
  ASSIGN_OR_RETURN(uint32 scu_ctrl_2_reg,
                   registers_->Read32(reset_offsets_.scu_ctrl_2));
  config::registers::ScuCtrl2 scu_ctrl_2(scu_ctrl_2_reg);
  scu_ctrl_2.set_rg_gated_gcb(0x1);
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::EnableHardwareClockGate] scu_ctrl_2 @0x%05llx "
      "read=0x%08x write=0x%08x (rg_gated_gcb=0x1=hw_clock_gated)",
      (unsigned long long)reset_offsets_.scu_ctrl_2, scu_ctrl_2_reg,
      scu_ctrl_2.raw());
  RETURN_IF_ERROR(
      registers_->Write32(reset_offsets_.scu_ctrl_2, scu_ctrl_2.raw()));

  hardware_clock_gated_ = true;
  LOG(INFO) << "[INIT][TopLevel::EnableHardwareClockGate] END";
  return Status();  // OK
}

Status BeagleTopLevelHandler::DisableHardwareClockGate() {
  LOG(INFO) << "[INIT][TopLevel::DisableHardwareClockGate] BEGIN";
  if (!hardware_clock_gated_) {
    LOG(INFO) << "[INIT][TopLevel::DisableHardwareClockGate] not gated — skip";
    return Status();  // OK
  }

  // Disable Software Clock Gate (GCB)
  // 1. Force clock on by writing "rg_gated_gcb" to 0x2. Read the register to
  // preserve other fields.
  ASSIGN_OR_RETURN(uint32 scu_ctrl_2_reg,
                   registers_->Read32(reset_offsets_.scu_ctrl_2));
  config::registers::ScuCtrl2 scu_ctrl_2(scu_ctrl_2_reg);
  scu_ctrl_2.set_rg_gated_gcb(0x2);
  LOG(INFO) << StringPrintf(
      "[INIT][TopLevel::DisableHardwareClockGate] scu_ctrl_2 @0x%05llx "
      "read=0x%08x write=0x%08x (rg_gated_gcb=0x2=force_on)",
      (unsigned long long)reset_offsets_.scu_ctrl_2, scu_ctrl_2_reg,
      scu_ctrl_2.raw());
  RETURN_IF_ERROR(
      registers_->Write32(reset_offsets_.scu_ctrl_2, scu_ctrl_2.raw()));

  hardware_clock_gated_ = false;
  LOG(INFO) << "[INIT][TopLevel::DisableHardwareClockGate] END";
  return Status();  // OK
}

}  // namespace driver
}  // namespace darwinn
}  // namespace platforms
