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
class EVMStackAllocWrapperPass : ImmutablePass {
  static char ID;

  EVMStackAllocWrapperPass() : ImmutablePass(ID) {
    //initializeStackAllocPassPass(*PassRegistry::getPassRegistry());
  }

  explicit EVMStackAllocWrapperPass()
      : ImmutablePass(ID) {
    //initializeEVMStackAllocWrapperPassPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override;

  // stack allocation specific fields
  typedef enum {
    P_STACK, // parameter
    X_STACK, // transfer
    E_STACK, // evaluate
    L_STACK, // local
    NONSTACK,// Allocate on memory
  } StackRegion;

  LiveIntervals *LIS;

  DenseMap<unsigned, StackRegion> regAssignments;

  void allocateRegistersToStack(MachineFunction &F);

private:
  void analyzeBasicBlock(MachineBasicBlock *MBB);

};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_EVM_EVMSTACKALLOCANALYSIS_H
