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

void EVMPostRASchedStrategy::enterMBB(MachineBasicBlock *MBB) {
  // Custom EVM PostRA specific behavior here.
  PostGenericScheduler::enterMBB(MBB);
}

void EVMPostRASchedStrategy::leaveMBB() {
  // Custom EVM PostRA specific behavior here.
  PostGenericScheduler::leaveMBB();
}

void EVMPostRASchedStrategy::initialize(ScheduleDAGMI *Dag) {
  // Custom EVM PostRA specific initialization here.
  PostGenericScheduler::initialize(Dag);
}

SUnit *EVMPostRASchedStrategy::pickNode(bool &IsTopNode) {
  // Custom EVM PostRA specific scheduling here.
  return PostGenericScheduler::pickNode(IsTopNode);
}


