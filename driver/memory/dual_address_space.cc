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

#include "driver/memory/dual_address_space.h"

#include "driver/hardware_structures.h"
#include "driver/memory/buddy_address_space.h"
#include "port/logging.h"
#include "port/ptr_util.h"
#include "port/stringprintf.h"

namespace platforms {
namespace darwinn {
namespace driver {

DualAddressSpace::DualAddressSpace(
    const config::ChipStructures& chip_structures, MmuMapper* mmu_mapper) {
  const int num_simple_entries =
      GetNumSimplePageTableEntries(chip_structures.num_page_table_entries);

  simple_ = gtl::MakeUnique<BuddyAddressSpace>(
      0, kHostPageSize * num_simple_entries, mmu_mapper);

  extended_ = gtl::MakeUnique<BuddyAddressSpace>(
      kExtendedAddressSpaceStart,
      kExtendedPageTableEntryAddressableBytes *
          GetNumExtendedPageTableEntries(
              chip_structures.num_page_table_entries),
      mmu_mapper);
}

StatusOr<DeviceBuffer> DualAddressSpace::MapMemory(
    const Buffer& buffer, DmaDirection direction,
    MappingTypeHint mapping_type) {
  // npu_driver vs libedgetpu 비교용 — caller 가 어느 영역을 요청했고 실제로
  // 어디에 매핑되는지 dump.  user-data 는 mmio_driver.cc::DoMapBuffer 에서
  // kExtended 로 hardcode 되어 항상 extended (0x8000_xxxx_xxxx_xxxx) 로 감.
  // chip-internal 용 (host_queue/status_block) 만 kSimple 로 직접 부름.
  const char* hint_label =
      mapping_type == MappingTypeHint::kSimple   ? "kSimple"   :
      mapping_type == MappingTypeHint::kExtended ? "kExtended" :
      mapping_type == MappingTypeHint::kAny      ? "kAny"      : "?";
  const char* dir_label =
      direction == DmaDirection::kBidirectional ? "BIDIRECTIONAL" :
      direction == DmaDirection::kToDevice      ? "TO_DEVICE(input)" :
      direction == DmaDirection::kFromDevice    ? "FROM_DEVICE(output)" : "?";

  switch (mapping_type) {
    case MappingTypeHint::kSimple:
      LOG(INFO) << StringPrintf(
          "[ADDR-SPACE] MapMemory hint=%s dir=%s size=%zu B → SIMPLE table "
          "(low VA, simple PT, PTE[0..6143])",
          hint_label, dir_label, buffer.size_bytes());
      return simple_->MapMemory(buffer, direction, mapping_type);

    case MappingTypeHint::kExtended:
    case MappingTypeHint::kAny:
      LOG(INFO) << StringPrintf(
          "[ADDR-SPACE] MapMemory hint=%s dir=%s size=%zu B → EXTENDED table "
          "(0x8000_xxxx_xxxx_xxxx, extended PT, PTE[6144..8191])",
          hint_label, dir_label, buffer.size_bytes());
      return extended_->MapMemory(buffer, direction, mapping_type);
  }
}

Status DualAddressSpace::UnmapMemory(DeviceBuffer buffer) {
  return DetermineSource(buffer)->UnmapMemory(buffer);
}

AddressSpace* DualAddressSpace::DetermineSource(
    const DeviceBuffer& device_buffer) const {
  if (device_buffer.device_address() & kExtendedVirtualAddressBit) {
    return extended_.get();
  } else {
    return simple_.get();
  }
}

}  // namespace driver
}  // namespace darwinn
}  // namespace platforms
