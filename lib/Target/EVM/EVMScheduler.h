//===- EVMMachineScheduler.cpp - Custom EVM MI scheduler --*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_EVM_EVMMACHINESCHEDULER_H
#define LLVM_LIB_TARGET_EVM_EVMMACHINESCHEDULER_H

#include "llvm/CodeGen/MachineScheduler.h"

namespace llvm {
/// A MachineSchedStrategy implementation for PowerPC post RA scheduling.
class EVMSchedStrategy : public GenericScheduler {
public:
  EVMSchedStrategy(const MachineSchedContext *C) :
    GenericScheduler(C) {}

protected:
  void initialize(ScheduleDAGMI *Dag) override;
  SUnit *pickNode(bool &IsTopNode) override;
  void enterMBB(MachineBasicBlock *MBB) override;
  void leaveMBB() override;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_EVM_EVMMACHINESCHEDULER_H

