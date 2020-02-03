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

#include "EVM.h"
#include "EVMStackAllocAnalysis.h"
#include "llvm/Pass.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>
#include <queue>

using namespace llvm;

#define DEBUG_TYPE "evm-stackalloc"

// Register this pass...
char EVMStackAlloc::ID = 0;

INITIALIZE_PASS(EVMStackAlloc, "evm-stackalloc",
                "Stack Allocation Analysis", false, true)

void EVMStackAlloc::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesAll();
}

void EVMStackAlloc::initializePass() {
  llvm_unreachable("unimplemented");
}

void EVMStackAlloc::allocateRegistersToStack(MachineFunction &F) {
    // clean up previous assignments.
    initializePass();

    // topologically iterate over machine basic blocks
    std::queue<MachineBasicBlock*> WorkingQueue;
    MachineBasicBlock &Entry = F.front();
    WorkingQueue.push(&Entry);

    while (!WorkingQueue.empty()) {
      MachineBasicBlock *MBB = WorkingQueue.front();
      WorkingQueue.pop();
      analyzeBasicBlock(MBB);

      // topological order
      for (MachineBasicBlock *NextMBB : MBB->successors()) {
        WorkingQueue.push(NextMBB);
      }
    }
}

static unsigned getDefRegister(const MachineInstr &MI) {
    unsigned numDefs = MI.getDesc().getNumDefs();
    assert(numDefs <= 1);
    const MachineOperand &def = *MI.defs().begin();
    assert(def.isReg() && def.isDef());
    return def.getReg();
}

bool EVMStackAlloc::defIsLocal(const MachineInstr &MI) const {
  unsigned defReg = getDefRegister(MI);
  const LiveInterval &LI = LIS->getInterval(defReg);
  
  // if it has multiple VNs, ignore it.
  if (LI.segments.size() > 1) {
    return false;
  }

  // if it goes across multiple MBBs, ignore it.
  const MachineBasicBlock* MBB = MI.getParent();
  SlotIndex MBBBegin = LIS->getMBBStartIdx(MBB);
  SlotIndex MBBEnd = LIS->getMBBEndIdx(MBB);

  return LI.isLocal(MBBBegin, MBBEnd);
}

void EVMStackAlloc::analyzeBasicBlock(MachineBasicBlock *MBB) {

  // Iterate over the instructions in the basic block.
  
  for (MachineInstr &MI : *MBB) {
    handleDef(MI);



    // handle stack arguments
    if (MI.getOpcode() == EVM::pSTACKARG_r) {
        // assign them to parameter stack.
        // TODO
        unsigned defReg = getDefRegister(MI);
        regAssignments.insert(
            std::pair<unsigned, StackAssignment>(defReg, {P_STACK, {0}}));
        continue;
    }

    // get register definition
    if (MI.getDesc().getNumDefs() == 0) {
      continue;
    }
    unsigned defReg = getDefRegister(MI);

    if (defIsLocal(MI)) {
      regAssignments.insert(
          std::pair<unsigned, StackAssignment>(defReg, {E_STACK, {0}}));
      continue;
    }

    // the register liveness expands multiple basicblocks.
    // * If the def BB dominates all sucessor BB:
    //     * If each leaf BB has a use: X_STACK
    //     * If some leaf BB do not have a use:
    //     * If a child's sucessor BB also have a use: (here we need to traverse the sucessors)
    //         * DUP before use. <-- possibly need to mark the instruction
    //         * POP if conditionally jump to an unused sucessor.
    // * If the def BB is a sucessor of a use BB:
    //     * we move it to NONSTACK.
    // TODO

    // finally, we have to allocate it on to memory.
    regAssignments.insert(
        std::pair<unsigned, StackAssignment>(defReg, {NONSTACK, {0}}));
  }

}

StackAssignment EVMStackAlloc::getStackAssignment(unsigned reg) const {
  assert(regAssignments.find(reg) != regAssignments.end() &&
         "Cannot find stack assignment for register.");
  return regAssignments.lookup(reg);
}

// return true of the register use in the machine instruction is the last use.
bool EVMStackAlloc::regIsLastUse(const MachineInstr &MI, unsigned reg) const {
  llvm_unreachable("unimplemented");
  return false;
}

// return the number of uses of the register.
unsigned EVMStackAlloc::getRegNumUses(unsigned reg) const {
  llvm_unreachable("unimplemented");
  return 0;
}

void EVMStackAlloc::handleDef(const MachineInstr &MI) {
  llvm_unreachable("unimplemented");
  unsigned defReg = getDefRegister(MI);

  // look into its liveness range, assign a stack.

  // PARAMETER case
  if (MI.getOpcode() == EVM::pSTACKARG_r) {
    // assign them to parameter stack.
    regAssignments.insert(
        std::pair<unsigned, StackAssignment>(defReg, {P_STACK, {0}}));
    
    currentStackStatus.P.insert(defReg);
    return;
  }

  // if the register has no use, then we do not allocate it
  if (MRI->use_nodbg_empty(defReg)) {
    regAssignments.insert(
        std::pair<unsigned, StackAssignment>(defReg, {NO_ALLOCATION, {0}}));
  }

  // LOCAL case
  if (defIsLocal(MI)) {
    // record assignment
    regAssignments.insert(
        std::pair<unsigned, StackAssignment>(defReg, {E_STACK, {0}}));

    // update stack status
    currentStackStatus.L.insert(defReg); 
    return;
  } 

  // TRANSFER case 
  // TODO


  // last resort: locate on memory
  regAssignments.insert(
      std::pair<unsigned, StackAssignment>(defReg, {NONSTACK, {0}}));
  currentStackStatus.M.insert(defReg);
  allocateMemorySlot(defReg);
}

void EVMStackAlloc::handleUses(const MachineInstr &MI) {
  for (const MachineOperand &MOP : MI.uses()) {
    handleSingleUse(MI, MOP);
  }
}


void EVMStackAlloc::handleSingleUse(const MachineInstr &MI, const MachineOperand &MOP) {
  llvm_unreachable("unimplemented");

  if (!MOP.isReg()) {
    return;
  }
  unsigned useReg = MOP.getReg();

  // get stack assignment
  assert(regAssignments.find(useReg) != regAssignments.end());
  StackAssignment SA = regAssignments.lookup(useReg);
  // we also do not care if we has determined we do not allocate it.
  if (SA.region == NO_ALLOCATION) {
    return;
  }

  // update stack status if it is the last use.
  if (regIsLastUse(MI, useReg)) {
    switch (SA.region) {
      case NONSTACK: {
        // release memory slot
        currentStackStatus.M.erase(useReg);
        deallocateMemorySlot(useReg);
        break;
      }
      case P_STACK: {
        currentStackStatus.P.erase(useReg);
        // TODO
        break;
      }
      case X_STACK: {
        currentStackStatus.X.erase(useReg);
        // TODO
        break;
      }
      case L_STACK: {
        // TODO
        break;
      }
      default: {
        llvm_unreachable("Impossible case");
        break;
      }
    }
  } else { // It is not the last use of a register.

    // * If it is not the last use in the same BB, dup it.
    // we only care about the last use in the BB, becuase only it matters
    if (hasUsesAfterInBB(useReg, MI)) {
      // TODO

      llvm_unreachable("unimplemented");
      return;
    }

    // * If each sucessor path has at least a use, dup it.
    switch (SA.region) {
      case NONSTACK: {
        // If it is a memory variable, we can simply ignore it.
        // do nothing
        break;
      }
      case P_STACK: {
        break;
      }
      case X_STACK: {
        break;
      }
      case L_STACK: {
        break;
      }
      default: {
        llvm_unreachable("Impossible case");
        break;
      }
    }
    llvm_unreachable("unimplemented");
  }
}

// return the allocated slot index of a memory
unsigned EVMStackAlloc::allocateMemorySlot(unsigned reg) {
  assert(reg != 0 && "Incoming registers cannot be zero.");
  // first, find if there is an empty slot:
  for (unsigned i = 0; i < memoryAssignment.size(); ++i) {
    // here we let 0 represent an empty slot
    if (memoryAssignment[i] == 0) {
      memoryAssignment[i] = reg;
      return i;
    }
  }

  // now we need to expand the memory:
  memoryAssignment.push_back(reg);
  return (memoryAssignment.size() - 1);
}

void EVMStackAlloc::deallocateMemorySlot(unsigned reg) {
  for (unsigned i = 0; i < memoryAssignment.size(); ++i) {
    if (reg == memoryAssignment[i]) {
      memoryAssignment[i] = 0;
      return;
    }
  }
  llvm_unreachable("Cannot find allocated memory slot");
}

bool EVMStackAlloc::hasUsesAfterInBB(unsigned reg, const MachineInstr &MI) const {
  const MachineBasicBlock* MBB = MI.getParent();

  // if this is the only use, then for sure it is the last use in MBB.
  assert(!MRI->use_nodbg_empty(reg) && "Empty use registers should not have a use.");
  if (MRI->hasOneUse(reg)) {
    return false;
  }

  // iterate over uses and see if any use exists in the same BB.
  for (MachineRegisterInfo::use_instr_nodbg_iterator
           Use = MRI->use_instr_nodbg_begin(reg),
           E = MRI->use_instr_nodbg_end();
       Use != E; ++Use) {
    // TODO     
    llvm_unreachable("unimplemented");
  }

  // we cannot find a use after it in BB
  return false;
}