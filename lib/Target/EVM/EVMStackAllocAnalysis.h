//===- EVMStackAllocAnalysis --------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_EVM_EVMSTACKALLOCANALYSIS_H
#define LLVM_LIB_TARGET_EVM_EVMSTACKALLOCANALYSIS_H

#include "EVM.h"
#include "EVMTargetMachine.h"
#include "llvm/ADT/DenseMap.h"

#include "llvm/ADT/Triple.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"
#include "llvm/Pass.h"
#include "llvm/CodeGen/LiveIntervals.h"

#include <algorithm>
#include <memory>

namespace llvm {

typedef enum {
  P_STACK,  // parameter
  X_STACK,  // transfer
  E_STACK,  // evaluate
  L_STACK,  // local
  NONSTACK, // Allocate on memory
} StackRegion;

typedef struct StackAssignment_ {
  StackRegion region;
  union {
    // For memory allocation
    unsigned MemorySlot;
    // For L stack allocation
    unsigned StackSlot;
    unsigned ParameterSlot;
  };
} StackAssignment;

class EVMStackAlloc : public ImmutablePass {
public:
  static char ID;

  EVMStackAlloc() : ImmutablePass(ID) {
    //initializeStackAllocPassPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override;

  // stack allocation specific fields

  LiveIntervals *LIS;

  DenseMap<unsigned, StackAssignment> regAssignments;

  void allocateRegistersToStack(MachineFunction &F);

  StackAssignment getStackAssignment(unsigned reg) const;

private:
  // analyze a single basicblock
  void analyzeBasicBlock(MachineBasicBlock *MBB);

  bool defIsLocal(MachineInstr *MI);

};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_EVM_EVMSTACKALLOCANALYSIS_H
