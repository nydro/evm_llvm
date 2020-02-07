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

unsigned EdgeSets::getEdgeSetIndex(Edge edge) const {
  // Linear search. Can improve it.
  for (std::pair<unsigned, Edge> edgePair : edgeIndex) {
    if (edgePair.second == edge) {
      auto result = edgeIndex2EdgeSet.find(edgePair.first);
      assert(result != edgeIndex2EdgeSet.end() && "Cannot find edge set!");
      return result->second;
    }
  }
  llvm_unreachable("Cannot find edge index.");
}

unsigned EdgeSets::getEdgeIndex(Edge edge) const {
  // Linear search. Can improve it.
  for (std::pair<unsigned, Edge> edgePair : edgeIndex) {
    if (edgePair.second == edge) {
      return edgePair.first;
    }
  }
  llvm_unreachable("Cannot find edge index.");
}


void EdgeSets::collectEdges(MachineFunction *MF) {
  std::set<MachineBasicBlock*> visited;
  for (MachineBasicBlock &MBB : *MF) {
    // Skip if we have visited this MBB:
    if (visited.find(&MBB) != visited.end()) {
      continue;
    }

    visited.insert(&MBB);
    for (auto *NextMBB : MBB.successors()) {
      unsigned key = edgeIndex.size();
      assert(edgeIndex.find(key) == edgeIndex.end() &&
             "Trying to emplace edge indexes.");
      edgeIndex[edgeIndex.size()] = Edge(&MBB, NextMBB);
    }
  }
}

void EdgeSets::computeEdgeSets(MachineFunction *MF) {
  // First, assign each edge with a number:
  collectEdges(MF);

  // Then, assign a new edge set for each of the edges
  for (std::pair<unsigned, Edge> index : edgeIndex) {
    edgeIndex2EdgeSet.insert({index.first, index.first});
  }

  // Criteria: Two MBBs share a same edge set if:
  // 1. they have a common child.
  // 2. they have a common parent.  
  for (MachineBasicBlock &MBB : *MF) {
    // we have more than one predecessors
    if (std::distance(MBB.pred_begin(), MBB.pred_end()) > 1) {
      // all predecessors belong to an edge set
      MachineBasicBlock *first = *MBB.pred_begin();

      MachineBasicBlock::pred_iterator iter = ++MBB.pred_begin();
      while (iter != MBB.pred_end()) {
        mergeEdgeSets({first, &MBB}, {*iter, &MBB});
      }
    }
    if (std::distance(MBB.succ_begin(), MBB.succ_end()) > 1) {
      // all sucessors belong to an edge set
      MachineBasicBlock *first = *MBB.succ_begin();

      MachineBasicBlock::pred_iterator iter = ++MBB.succ_begin();
      while (iter != MBB.succ_end()) {
        mergeEdgeSets({&MBB, first}, {&MBB, *iter});
      }
    }
  }
}

void EdgeSets::mergeEdgeSets(Edge edge1, Edge edge2) {
  unsigned edgeSet1 = getEdgeSetIndex(edge1);
  unsigned edgeSet2 = getEdgeSetIndex(edge2);

  // return if they are already in the same edge set.
  if (edgeSet1 == edgeSet2) {
    return;
  }

  // change edgeSet2 to edgeSet1
  for (std::pair<unsigned, unsigned> index2Set : edgeIndex2EdgeSet) {
    if (index2Set.second == edgeSet2) {
      edgeIndex2EdgeSet.emplace(index2Set.first, edgeSet1);
    }
  }
  return;
}



void EVMStackAlloc::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesAll();
}

void EVMStackAlloc::initialize() {
  edgeSets.reset();
  regAssignments.clear();
  currentStackStatus.reset();
  edgeset2assignment.clear();
}

void EVMStackAlloc::allocateRegistersToStack(MachineFunction &F) {
    // clean up previous assignments.
    initializePass();

    // compute edge sets
    edgeSets.computeEdgeSets(&F);

    // analyze each BB
    for (MachineBasicBlock &MBB : F) {
      analyzeBasicBlock(&MBB);
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

  // examine live range to see if it only covers a single MBB:
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
    handleUses(MI);
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
  unsigned defReg = getDefRegister(MI);

  // Rule out case: multiple defs:
  // if there are multiple defines, then it goes to memory
  if (!MRI->hasOneDef(defReg)) {
    currentStackStatus.M.insert(defReg);
    allocateMemorySlot(defReg);
    return;
  }

  // if the register has no use, then we do not allocate it
  if (MRI->use_nodbg_empty(defReg)) {
    regAssignments.insert(
        std::pair<unsigned, StackAssignment>(defReg, {NO_ALLOCATION, 0}));
    return;
  }

  // LOCAL case
  if (defIsLocal(MI)) {
    // record assignment
    regAssignments.insert(
        std::pair<unsigned, StackAssignment>(defReg, {L_STACK, 0}));
    // update stack status
    currentStackStatus.L.insert(defReg); 
    return;
  } 

  // If all uses are in a same edge set, send it to Transfer Stack
  // This could greatly benefit from a stack machine specific optimization.
  if (liveIntervalWithinSameEdgeSet(defReg)) {
    // it is a def register, so we only care about out-going edges.

    // construct outgoing edges 

    MachineBasicBlock* ThisMBB = const_cast<MachineInstr&>(MI).getParent(); 
    for (MachineBasicBlock *NextMBB : ThisMBB->successors()) {
      EdgeSets::Edge edge = {ThisMBB, NextMBB};
      unsigned edgeIndex = edgeSets.getEdgeIndex(edge);
      allocateXRegion(edgeIndex, defReg);
    }
    return;
  }

  // Everything else goes to memory
  currentStackStatus.M.insert(defReg);
  allocateMemorySlot(defReg);
  return; 
}

// We only look at uses.
bool EVMStackAlloc::liveIntervalWithinSameEdgeSet(unsigned defReg) {
  std::set<unsigned> edgeSetIndices;
  for (MachineOperand &use : MRI->use_operands(defReg)){
    MachineBasicBlock* MBB = use.getParent()->getParent();

    // Look for predecessor edges
    for (MachineBasicBlock* Pred : MBB->predecessors()) {
      EdgeSets::Edge edge = {Pred, MBB};
      unsigned setIndex = edgeSets.getEdgeSetIndex(edge);
      edgeSetIndices.insert(setIndex);
    }

    // Look for successor edges
    for (MachineBasicBlock* Succ : MBB->successors()) {
      EdgeSets::Edge edge = {MBB, Succ};
      unsigned setIndex = edgeSets.getEdgeSetIndex(edge);
      edgeSetIndices.insert(setIndex);
    }
  }

  assert(!edgeSetIndices.empty() && "Edge set cannot be empty.");

  if (edgeSetIndices.size() == 1) {
    return true;
  } else {
    return false;
  }
}

void EVMStackAlloc::handleUses(const MachineInstr &MI) {
  for (const MachineOperand &MOP : MI.uses()) {
    handleSingleUse(MI, MOP);
  }
}

void EVMStackAlloc::handleSingleUse(const MachineInstr &MI, const MachineOperand &MOP) {
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

unsigned EVMStackAlloc::allocateXRegion(unsigned setIndex, unsigned reg) {
  // Each entry of an edge set should contain the same
  // X region layout
  std::vector<unsigned> &x_region = edgeset2assignment[setIndex];

  assert(std::find(x_region.begin(), x_region.end(), reg) == x_region.end() &&
         "Inserting duplicate element in X region.");

  x_region.push_back(reg);
  return x_region.size();
}

/*
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
*/

unsigned EVMStackAlloc::getCurrentStackDepth() const {
  return currentStackStatus.L.size() + currentStackStatus.X.size();
}

void EVMStackAlloc::pruneStackDepth() {
  unsigned stackDepth = getCurrentStackDepth();
  if (stackDepth < MAXIMUM_STACK_DEPTH) {
    return;
  }
  LLVM_DEBUG(dbgs() << "Stack Depth exceeds maximum, start pruning.");

  // First look at transfer stack:
  unsigned spillingCandidate = 0;
  std::set<unsigned> *vecRegs;
  if (currentStackStatus.X.size() != 0) {
    vecRegs = &currentStackStatus.X;
  } else {
    vecRegs = &currentStackStatus.L;
  }
  spillingCandidate = findSpillingCandidate(*vecRegs);
  allocateMemorySlot(spillingCandidate);
}

unsigned EVMStackAlloc::findSpillingCandidate(std::set<unsigned> &vecRegs) const {
  llvm_unreachable("unimplemented");
}