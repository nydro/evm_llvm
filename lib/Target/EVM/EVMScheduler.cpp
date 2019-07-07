//===- EVMMachineScheduler.cpp - MI Scheduler for PowerPC -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "EVMScheduler.h"
#include "MCTargetDesc/EVMMCTargetDesc.h"

using namespace llvm;

void EVMSchedStrategy::enterMBB(MachineBasicBlock *MBB) {
  // Custom EVM specific behavior here.
  GenericScheduler::enterMBB(MBB);
}

void EVMSchedStrategy::leaveMBB() {
  // Custom EVM specific behavior here.
  GenericScheduler::leaveMBB();
}

void EVMSchedStrategy::initialize(ScheduleDAGMI *Dag) {
  // Custom EVM specific initialization here.
  GenericScheduler::initialize(Dag);
}

SUnit *EVMSchedStrategy::pickNode(bool &IsTopNode) {
  // Custom EVM specific scheduling here.
  return GenericScheduler::pickNode(IsTopNode);
}


