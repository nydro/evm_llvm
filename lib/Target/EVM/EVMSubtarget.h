//===-- EVMSubtarget.h - Define Subtarget for the EVM -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the EVM specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_EVM_EVMSUBTARGET_H
#define LLVM_LIB_TARGET_EVM_EVMSUBTARGET_H

#include "EVMFrameLowering.h"
#include "EVMISelLowering.h"
#include "EVMInstrInfo.h"
#include "llvm/CodeGen/SelectionDAGTargetInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"

#define GET_SUBTARGETINFO_HEADER
#include "EVMGenSubtargetInfo.inc"

namespace llvm {
class StringRef;

class EVMSubtarget : public EVMGenSubtargetInfo {
  virtual void anchor();

  // EVM Change
  bool HasEIP211 = false;
  bool HasEIP615 = false;

  bool hasEIP211() const { return HasEIP211; }
  bool hasEIP615() const { return HasEIP615; }
  // end of EVM Change

  EVMFrameLowering FrameLowering;
  EVMInstrInfo InstrInfo;
  EVMTargetLowering TLInfo;
  SelectionDAGTargetInfo TSInfo;

  /// Initializes using the passed in CPU and feature strings so that we can
  /// use initializer lists for subtarget initialization.
  EVMSubtarget &initializeSubtargetDependencies(StringRef CPU, StringRef FS);

public:
  // Initializes the data members to match that of the specified triple.
  EVMSubtarget(const Triple &TT, const std::string &CPU,
               const std::string &FS, const TargetMachine &TM);

  // Parses features string setting specified subtarget options. The
  // definition of this function is auto-generated by tblgen.
  void ParseSubtargetFeatures(StringRef CPU, StringRef FS);

  const EVMFrameLowering *getFrameLowering() const override {
    return &FrameLowering;
  }

  const EVMInstrInfo *getInstrInfo() const override { return &InstrInfo; }

  const EVMRegisterInfo *getRegisterInfo() const override {
    return &InstrInfo.getRegisterInfo();
  }

  const EVMTargetLowering *getTargetLowering() const override {
    return &TLInfo;
  }

  const SelectionDAGTargetInfo *getSelectionDAGInfo() const override {
    return &TSInfo;
  }

  // Memory layout
  const unsigned getFreeMemoryPointer() const { return 64; }
  const unsigned getZeroPointer() const { return getFreeMemoryPointer() + 32; }
  const unsigned getGeneralPurposeMemoryStart() const { return getZeroPointer() + 32; }
  const unsigned getIdentityContractAddress() const { return 4; }

  // Constants
};
} // End llvm namespace

#endif
