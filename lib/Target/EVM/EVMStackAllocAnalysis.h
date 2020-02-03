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
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Triple.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"
#include "llvm/Pass.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

#include <algorithm>
#include <memory>

namespace llvm {

typedef enum {
  P_STACK,       // parameter
  X_STACK,       // transfer
  E_STACK,       // evaluate
  L_STACK,       // local
  NONSTACK,      // allocate on memory
  NO_ALLOCATION, // do not allocate
} StackRegion;

// We also assign a memory slot
typedef struct StackAssignment_ {
  StackRegion region;
  unsigned numUses;
  // For memory allocation
  unsigned MemorySlot;
} StackAssignment;

// The stack arrangement is as follows:
// 
// +----------------+
// |                |
// |  Evaluations   |
// |                |
// +----------------+
// |                |
// |  Locals        |
// |                |
// +----------------+
// |                |
// |  Transfers     |
// |                |
// +----------------+
// |                |
// |  Parameters    |
// |                |
// +----------------+
// 
// So we can calculate an element's depth.

typedef struct {
  std::set<unsigned> P;     // Parameter Stack
  std::set<unsigned> X;     // Transfer Stack
  std::set<unsigned> E;     // Evaluation Stack
  std::set<unsigned> L;     // Local Stack
  std::set<unsigned> M;     // Memory
} StackStatus;

class EVMStackAlloc : public ImmutablePass {
public:
  static char ID;

  EVMStackAlloc() : ImmutablePass(ID) {
    initializeEVMStackAllocPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override;



  void allocateRegistersToStack(MachineFunction &F);

  StackAssignment getStackAssignment(unsigned reg) const;

  bool runOnModule(Module &M) override;

  unsigned getNumOfAllocatedMemorySlots() const {
    return memoryAssignment.size();
  };

private:
  LiveIntervals *LIS;
  MachineFunction *F;
  MachineRegisterInfo *MRI;

  // record assignments of each virtual register 
  DenseMap<unsigned, StackAssignment> regAssignments;
  StackStatus currentStackStatus;
  std::vector<unsigned> memoryAssignment;

  void initializePass();

  // the pass to analyze a single basicblock
  void analyzeBasicBlock(MachineBasicBlock *MBB);

  void handleDef(const MachineInstr &MI);
  void handleUses(const MachineInstr &MI);

  // handle a single use in the specific machine instruction. 
  void handleSingleUse(const MachineInstr &MI, const MachineOperand &MOP);

  // if the def and use is within a single BB
  bool defIsLocal(const MachineInstr &MI) const;

  // return true if the use in the specific MI is the last use of a reg
  bool regIsLastUse(const MachineInstr &MI, unsigned reg) const;

  // return numbers of uses of a register.
  unsigned getRegNumUses(unsigned reg) const;

  // helper function:
  void assignRegister(unsigned reg, StackRegion region);
  void removeRegisterAssignment(unsigned reg, StackRegion region);

  // for allocating 
  void deallocateMemorySlot(unsigned reg);
  unsigned allocateMemorySlot(unsigned reg);

  bool hasUsesAfterInBB(unsigned reg, const MachineInstr &MI) const;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_EVM_EVMSTACKALLOCANALYSIS_H
