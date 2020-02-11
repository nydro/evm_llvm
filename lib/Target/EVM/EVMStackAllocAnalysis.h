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
  X_STACK,       // transfer
  L_STACK,       // local
  NONSTACK,      // allocate on memory
  NO_ALLOCATION, // do not allocate
} StackRegion;

// We also assign a memory slot
typedef struct StackAssignment_ {
  StackRegion region;
  unsigned numUses;
  // For memory allocation
  unsigned memorySlot;
  unsigned stackSlot;
} StackAssignment;

// The stack arrangement is as follows:
// 
// +----------------+
// |                |
// |  Locals        |
// |                |
// +----------------+
// |                |
// |  Transfers     |
// |                |
// +----------------+
// 
// So we can calculate an element's depth.

// This class records basic block edge set information.
class EdgeSets {
public:
  typedef std::pair<MachineBasicBlock *, MachineBasicBlock *> Edge;
  void computeEdgeSets(MachineFunction *MF);
  unsigned getEdgeSetIndex(Edge edge) const;
  unsigned getEdgeIndex(Edge edge) const;

  void reset() {
    edgeIndex2EdgeSet.clear();
    edgeIndex.clear();
  }

  void dump() const;

private:
  // Index :: EdgeSet
  std::map<unsigned, unsigned> edgeIndex2EdgeSet;   

  // Index : Edge
  std::map<unsigned, Edge> edgeIndex;

  void collectEdges(MachineFunction *MF);

  // given two edges, merge their edgesets.
  void mergeEdgeSets(Edge edge1, Edge edge2);
};

class EVMStackAlloc : public ImmutablePass {
public:
  static char ID;

  // TODO: 15 is a bit arbitrary:
  static const unsigned MAXIMUM_STACK_DEPTH = 15;

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

  void getXStackRegion(unsigned edgeSetIndex,
                       std::vector<unsigned> xRegion) const;
  
  EdgeSets& getEdgeSets() const;

private:
  typedef struct {
    std::set<unsigned> X; // Transfer Stack
    std::set<unsigned> L; // Local Stack
    std::set<unsigned> M; // Memory
    void reset() {
      X.clear();
      L.clear();
      M.clear();
    }
  } StackStatus;

  LiveIntervals *LIS;
  MachineFunction *F;
  MachineRegisterInfo *MRI;

  // edge set information
  EdgeSets edgeSets;

  // record assignments of each virtual register 
  DenseMap<unsigned, StackAssignment> regAssignments;
  StackStatus currentStackStatus;
  std::vector<unsigned> memoryAssignment;

  // map: edgeset -> Stack Assignment
  std::map<unsigned, std::vector<unsigned>> edgeset2assignment;

  void initialize();

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
  unsigned allocateMemorySlot(unsigned reg);
  void deallocateMemorySlot(unsigned reg);

  unsigned allocateXRegion(unsigned setIndex, unsigned reg);

  bool hasUsesAfterInBB(unsigned reg, const MachineInstr &MI) const;

  // test if we should spill some registers to memory
  unsigned getCurrentStackDepth() const; 

  void pruneStackDepth();
  unsigned findSpillingCandidate(std::set<unsigned> &vecRegs) const;

  bool liveIntervalWithinSameEdgeSet(unsigned def);
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_EVM_EVMSTACKALLOCANALYSIS_H
