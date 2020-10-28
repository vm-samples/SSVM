// SPDX-License-Identifier: Apache-2.0
//===-- ssvm/runtime/instance/elem.h - Element Instance definition --------===//
//
// Part of the SSVM Project.
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the element instance definition in store manager.
///
//===----------------------------------------------------------------------===//
#pragma once

#include "common/span.h"
#include "common/types.h"
#include "common/value.h"

#include <vector>

namespace SSVM {
namespace Runtime {
namespace Instance {

class ElementInstance {
public:
  ElementInstance() = delete;
  ElementInstance(const RefType EType, Span<const RefVariant> Init)
      : Type(EType), Refs(Init.begin(), Init.end()) {}

  /// Get reference type of element instance.
  RefType getRefType() const { return Type; }

  /// Get reference lists in element instance.
  Span<const RefVariant> getRefs() const noexcept { return Refs; }

  /// Clear references in element instance.
  void clear() { Refs.clear(); }

private:
  /// \name Data of element instance.
  /// @{
  const RefType Type;
  std::vector<RefVariant> Refs;
  /// @}
};

} // namespace Instance
} // namespace Runtime
} // namespace SSVM
