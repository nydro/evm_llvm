//===- EVMStackAllocAnalysis ------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
/// This is the AMGPU address space based alias analysis pass.
//===----------------------------------------------------------------------===//

#include "EVMStackAllocAnalysis.h"
#include "llvm/Pass.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>
#include <queue>

using namespace llvm;

#define DEBUG_TYPE "evm-stackalloc"

// Register this pass...
char EVMStackAllocWrapperPass::ID = 0;

INITIALIZE_PASS(EVMStackAllocWrapperPass, "evm-stackalloc",
                "Stack Allocation Analysis", false, true)

ImmutablePass *llvm::createEVMStackAllocWrapperPass() {
  return new EVMStackAllocWrapperPass();
}

void EVMStackAllocWrapperPass::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesAll();
}

void EVMStackAllocWrapperPass::allocateRegistersToStack(MachineFunction &F) {
    // clean up previous assignments.
    regAssignments.clear();

    // topologically iterate over machine basic blocks
    std::queue<MachineBasicBlock*> WorkingQueue;
    MachineBasicBlock &Entry = F.front();
    WorkingQueue.push(&Entry);

    while (!WorkingQueue.empty()) {
      MachineBasicBlock *MBB = WorkingQueue.pop();
      analyzeBasicBlock(MBB);
    }
}

static unsigned getDefRegister(MachineInstr* MI) {
    unsigned numDefs = MI->getDesc().getNumDefs();
    assert(numDefs <= 1);
    MachineOperand &def = *MI->defs().begin();
    assert(def.isReg() && def.isDef());
    return def.getReg();
}

static bool isLocal(LiveIntervals *LIS, MachineInstr *MI) {
  unsigned defReg = getDefRegister(MI);
  const LiveInterval &LI = LIS->getInterval(defReg);
  
  // if it has multiple VNs, ignore it.
  if (LI.segments.size() > 1) {
    return false;
  }

  // if it goes across multiple MBBs, ignore it.
  MachineBasicBlock* MBB = MI->getParent();
  SlotIndex MBBBegin = LIS->getMBBStartIdx(MBB);
  SlotIndex MBBEnd = LIS->getMBBEndIdx(MBB);

  return LI.isLocal(MBBBegin, MBBEnd);
}

void EVMStackAllocWrapperPass::analyzeBasicBlock(MachineBasicBlock *MBB) {

  // Iterate over the instructions in the basic block.
  
  for (MachineInstr &MI : *MBB) {
    // handle stack arguments
    if (MI.getOpcode() == EVM::pSTACKARG_r) {
        // assign them to parameter stack.
        // TODO
        unsigned defReg = getDefRegister(&MI);
        regAssignments.insert(
            std::pair<unsigned, StackRegion>(defReg, P_STACK));
        continue;
    }

    // get register definition
    if (MI.getDesc().getNumDefs() == 0) {
      continue;
    }
    unsigned defReg = getDefRegister(&MI);

    if (isLocal(this->LIS, &MI)) {
      regAssignments.insert(std::pair<unsigned, StackRegion>(defReg, L_STACK));
      continue;
    }

    // the register liveness expands multiple basicblocks.
    // TODO

    // finally, we have to allocate it on to memory.
    regAssignments.insert(std::pair<unsigned, StackRegion>(defReg, NONSTACK));
  }

}