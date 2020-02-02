//===-- EVMStackification.cpp - Optimize stack operands ---------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file Ported over from WebAssembly backend.
///
//===----------------------------------------------------------------------===//

#include "EVM.h"
#include "EVMMachineFunctionInfo.h"
#include "EVMSubtarget.h"
#include "EVMRegisterInfo.h"
#include "EVMTargetMachine.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Support/Debug.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineDominanceFrontier.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/LiveIntervals.h"

using namespace llvm;

#define DEBUG_TYPE "evm-stackification"

namespace {

class StackStatus {
public:
  StackStatus() {}

  void swap(unsigned depth);
  void dup(unsigned depth);
  void push(unsigned reg);
  void pop();
  unsigned getStackDepth();
  unsigned get(unsigned depth);
  void dump() const;

private:
  // stack arrangements.
  // TODO: think of a way to do it.
  std::vector<unsigned> stackElements;

  // TODO: use DUPs instead of DUP1 + SWAPx for uses.
  DenseMap<unsigned, unsigned> remainingUses;
};

unsigned StackStatus::getStackDepth() {
  return stackElements.size();
}

unsigned StackStatus::get(unsigned depth) {
  return stackElements.rbegin()[depth];
}

void StackStatus::swap(unsigned depth) {
    assert(depth != 0);
    assert(stackElements.size() >= 2);
    LLVM_DEBUG({
      unsigned first = stackElements.rbegin()[0];
      unsigned fst_idx = Register::virtReg2Index(first);
      unsigned second = stackElements.rbegin()[depth];
      unsigned snd_idx = Register::virtReg2Index(second);
      dbgs() << "  SWAP" << depth << ": Swapping %" << fst_idx << " and %"
             << snd_idx << "\n";
    });
    std::iter_swap(stackElements.rbegin(), stackElements.rbegin() + depth);
}

void StackStatus::dup(unsigned depth) {
  unsigned elem = *(stackElements.rbegin() + depth);

  LLVM_DEBUG({
    unsigned idx = Register::virtReg2Index(elem);
    dbgs() << "  Duplicating %" << idx << " at depth " << depth << "\n";
  });

  stackElements.push_back(elem);
}

void StackStatus::pop() {
  LLVM_DEBUG({
    unsigned reg = stackElements.back();
    unsigned idx = Register::virtReg2Index(reg);
    dbgs() << "  Popping %" << idx << " from stack.\n";
  });
  stackElements.pop_back();
}

void StackStatus::push(unsigned reg) {
  /*
  LLVM_DEBUG({
    unsigned idx = Register::virtReg2Index(reg);
    dbgs() << "  Pushing %" << idx << " to top of stack.\n";
  });
  */
  stackElements.push_back(reg);
}


void StackStatus::dump() const {
  LLVM_DEBUG({
    dbgs() << "  Stack : ";
    unsigned counter = 0;
    for (auto i = stackElements.rbegin(), e = stackElements.rend(); i != e; ++i) {
      unsigned idx = Register::virtReg2Index(*i);
      dbgs() << "(" << counter << ", %" << idx << "), ";
      counter ++;
    }
    dbgs() << "\n";
  });
}

class EVMStackification final : public MachineFunctionPass {
public:
  static char ID; // Pass identification, replacement for typeid
  EVMStackification() : MachineFunctionPass(ID) {}

  StringRef getPassName() const override { return "EVM Stackification"; }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
    AU.addRequired<LiveIntervals>();
    AU.addRequired<MachineDominatorTree>();
    AU.addRequired<MachineDominanceFrontier>();
    AU.addPreserved<MachineDominatorTree>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

private:
  bool isSingleDefSingleUse(unsigned RegNo) const;
  MachineInstr *getVRegDef(unsigned Reg, const MachineInstr *Insert) const;

  void handleUses(StackStatus &ss, MachineInstr &MI);
  void handleDef(StackStatus &ss, MachineInstr &MI);
  bool canStackifyReg(unsigned reg, MachineInstr &MI) const;
  unsigned findNumOfUses(unsigned reg) const;

  void insertPop(MachineInstr &MI, bool insertAfter = true);
  void insertDup(unsigned index, MachineInstr &MI, bool insertAfter);
  void insertSwap(unsigned index, MachineInstr &MI, bool insertAfter);

  void insertLoadFromMemoryBefore(unsigned reg, MachineInstr& MI);
  void insertStoreToMemory(unsigned reg, MachineInstr &MI, bool insertAfter);
  void bringOperandToTop(StackStatus &ss, unsigned depth, MachineInstr &MI);

  void handleIrregularInstruction(StackStatus &ss, MachineInstr &MI);

  void handleEntryMBB(StackStatus &ss, MachineBasicBlock &MBB);
  void handleMBB(StackStatus &ss, MachineBasicBlock &MBB);

  DenseMap<unsigned, unsigned> reg2index;

  const EVMInstrInfo *TII;
  MachineRegisterInfo *MRI;
  LiveIntervals *LIS;
  EVMMachineFunctionInfo *MFI;
};
} // end anonymous namespace

char EVMStackification::ID = 0;
INITIALIZE_PASS(EVMStackification, DEBUG_TYPE,
      "Converting register-based instructions into stack instructions",
      false, false)

FunctionPass *llvm::createEVMStackification() {
  return new EVMStackification();
}

bool EVMStackification::isSingleDefSingleUse(unsigned RegNo) const {
  return (MRI->hasOneUse(RegNo) && MRI->hasOneDef(RegNo));
}

// Identify the definition for this register at this point. This is a
// generalization of MachineRegisterInfo::getUniqueVRegDef that uses
// LiveIntervals to handle complex cases.
MachineInstr *EVMStackification::getVRegDef(unsigned Reg,
                                            const MachineInstr *Insert) const {
  // Most registers are in SSA form here so we try a quick MRI query first.
  if (MachineInstr *Def = MRI->getUniqueVRegDef(Reg))
    return Def;

  auto &LIS = *this->LIS;

  // MRI doesn't know what the Def is. Try asking LIS.
  if (const VNInfo *ValNo = LIS.getInterval(Reg).getVNInfoBefore(
          LIS.getInstructionIndex(*Insert)))
    return LIS.getInstructionFromIndex(ValNo->def);

  return nullptr;
}

/***
// The following are the criteria for deciding whether to stackify the register
// or not:
// 1. This reg has only one def
// 2. the uses of the reg is in the same basicblock
*/
bool EVMStackification::canStackifyReg(unsigned reg, MachineInstr& MI) const {
  assert(!Register::isPhysicalRegister(reg));

  const LiveInterval &LI = LIS->getInterval(reg);
  
  // if it has multiple VNs, ignore it.
  if (LI.segments.size() > 1) {
    return false;
  }

  // if it goes across multiple MBBs, ignore it.
  MachineBasicBlock* MBB = MI.getParent();
  SlotIndex MBBBegin = LIS->getMBBStartIdx(MBB);
  SlotIndex MBBEnd = LIS->getMBBEndIdx(MBB);

  if(!LI.isLocal(MBBBegin, MBBEnd)) {
    return false;
  }

  return true;
}

unsigned EVMStackification::findNumOfUses(unsigned reg) const {
  auto useOperands = MRI->reg_nodbg_operands(reg);
  unsigned numUses = std::distance(useOperands.begin(), useOperands.end());
  return numUses;
}

void EVMStackification::insertPop(MachineInstr& MI, bool insertAfter) {
  MachineBasicBlock *MBB = MI.getParent();
  MachineFunction &MF = *MBB->getParent();
  MachineInstrBuilder pop = BuildMI(MF, MI.getDebugLoc(), TII->get(EVM::POP_r));
  if (insertAfter) {
    MBB->insertAfter(MachineBasicBlock::iterator(MI), pop);
  } else {
    MBB->insert(MachineBasicBlock::iterator(MI), pop);
  }
}

void EVMStackification::insertDup(unsigned index, MachineInstr &MI, bool insertAfter = true) {
  MachineBasicBlock *MBB = MI.getParent();
  MachineFunction &MF = *MBB->getParent();
  MachineInstrBuilder dup = BuildMI(MF, MI.getDebugLoc(), TII->get(EVM::DUP_r)).addImm(index);
  if (insertAfter) {
    MBB->insertAfter(MachineBasicBlock::iterator(MI), dup);
  } else {
    MBB->insert(MachineBasicBlock::iterator(MI), dup);
  }
}

// The skip here means how many same items needs to be skipped.
static bool findRegDepthOnStack(StackStatus &ss, unsigned reg, unsigned *depth,
                                unsigned skip = 0) {
  unsigned curHeight = ss.getStackDepth();

  for (unsigned d = 0; d < curHeight; ++d) {
    unsigned stackReg = ss.get(d);
    if (stackReg == reg) {
      if (skip != 0) {
        // skipping register
        --skip;
      } else {
        *depth = d;
        LLVM_DEBUG({
          unsigned idx = Register::virtReg2Index(reg);
          dbgs() << "  Found %" << idx << " at depth: " << *depth << "\n";
        });
        return true;
      }
    }
  }

  return false;
}

void EVMStackification::insertSwap(unsigned index, MachineInstr &MI,
                                         bool insertAfter = false) {
  MachineBasicBlock *MBB = MI.getParent();
  MachineInstrBuilder swap =
      BuildMI(*MBB->getParent(), MI.getDebugLoc(), TII->get(EVM::SWAP_r))
          .addImm(index);
  if (insertAfter) {
    MBB->insertAfter(MachineBasicBlock::iterator(MI), swap);
  } else {
    MBB->insert(MachineBasicBlock::iterator(MI), swap);
  }
}

void EVMStackification::insertLoadFromMemoryBefore(unsigned reg, MachineInstr &MI) {
  MachineBasicBlock *MBB = MI.getParent();

  // deal with physical register.
  unsigned index = MFI->get_memory_index(reg);
  unsigned ridx = Register::virtReg2Index(reg);
  LLVM_DEBUG(dbgs() << "  %" << ridx << " <= GETLOCAL(" << index << ") inserted.\n");
  BuildMI(*MBB, MI, MI.getDebugLoc(), TII->get(EVM::pGETLOCAL_r), reg)
      .addImm(index);
}

void EVMStackification::insertStoreToMemory(unsigned reg, MachineInstr &MI, bool InsertAfter = true) {
  MachineBasicBlock *MBB = MI.getParent();
  MachineFunction &MF = *MBB->getParent();

  unsigned index = MFI->get_memory_index(reg);
  unsigned ridx = Register::virtReg2Index(reg);
  LLVM_DEBUG(dbgs() << "  PUTLOCAL(" << index << ") => %" << ridx << 
                 "  is inserted.\n");
  MachineInstrBuilder putlocal =
      BuildMI(MF, MI.getDebugLoc(), TII->get(EVM::pPUTLOCAL_r)).addReg(reg).addImm(index);
  if (InsertAfter) {
    MBB->insertAfter(MachineBasicBlock::iterator(MI), putlocal);
  } else {
    MBB->insert(MachineBasicBlock::iterator(MI), putlocal);
  }
}

// bring a stack element to top, without altering other stack element positions.
void EVMStackification::bringOperandToTop(StackStatus &ss, unsigned depth,
                                          MachineInstr &MI) {
  assert(depth <= 16);

  for (unsigned i = 1; i <= depth; ++i) {
    insertSwap(i, MI);
    ss.swap(i);
  }
}

void EVMStackification::handleIrregularInstruction(StackStatus &ss,
                                                   MachineInstr &MI) {
  // iterate over the operands (back to front), and bring each of them to top.
  unsigned use_counter = MI.getNumOperands() - 1;
  unsigned end_defs = MI.getNumExplicitDefs() - 1;

  unsigned pop_counter = 0;
  while (use_counter != end_defs) {
    const MachineOperand &MO = MI.getOperand(use_counter);
    --use_counter;
    ++pop_counter;

    if (!MO.isReg() || MO.isImplicit()) {
      LLVM_DEBUG(dbgs() << "  Operand is not reg or is implicit, skip: ";
                 MO.dump());
      return;
    }

    unsigned reg = MO.getReg();
    if (!MFI->isVRegStackified(reg)) {
      LLVM_DEBUG(dbgs() << "  Operand is not stackified: "; MO.dump());
      insertLoadFromMemoryBefore(reg, MI);
      ss.push(reg);
    } else {
      LLVM_DEBUG(dbgs() << "  Operand is stackified: "; MO.dump());
      // stackified case:
      unsigned depthFromTop = 0;
      bool result = findRegDepthOnStack(ss, reg, &depthFromTop);
      assert(result);

      bringOperandToTop(ss, depthFromTop, MI);
    }
  }

  for (unsigned i = 0; i < MI.getNumOperands() - MI.getNumExplicitDefs(); ++i) {
    ss.pop();
  }

}

void EVMStackification::handleUses(StackStatus &ss, MachineInstr& MI) {
  // TODO: do not support more than 2 uses in an MI. We need scheduler to help
  // us make sure that the registers are on stack top.
 
  const auto &uses = MI.uses();
  unsigned numUsesInMI = 0;

  if (MI.isPseudo()){
    // PUTLOCAL and GETLOCAL will have their constant value at the back
    // find actual num uses:
    for (const MachineOperand &MO : uses) {
      if (MO.isReg()) {
        numUsesInMI++;
      }
    }
  } else {
    numUsesInMI = std::distance(uses.begin(), uses.end());
  }



  // Case 1: only 1 use
  if (numUsesInMI == 1) {
    MachineOperand& MO = *MI.uses().begin(); 
    if (!MO.isReg()) {
      return;
    }
    unsigned reg = MO.getReg();

    // handle vreg unstackfied case
    if (!MFI->isVRegStackified(reg)) {
      LLVM_DEBUG(dbgs() << "  Operand is not stackified.\n";);
      insertLoadFromMemoryBefore(reg, MI);
      ss.push(reg);
    } else {

      // handle vreg stackified case
      unsigned depthFromTop = 0;
      bool result = findRegDepthOnStack(ss, reg, &depthFromTop);
      assert(result);
      LLVM_DEBUG(dbgs() << "  Operand is on the stack at depth: "
                        << depthFromTop << "\n";);

      // check if it is on top of the stack.
      if (depthFromTop != 0) {
        // TODO: insert swap
        insertSwap(depthFromTop, MI);
        ss.swap(depthFromTop);
      }
    }

    ss.pop();
    return;
  }

  if (numUsesInMI == 2) {
    MachineOperand& MO1 = *MI.uses().begin(); 
    MachineOperand& MO2 = *(MI.uses().begin() + 1); 

    assert(MO1.isReg() && MO2.isReg());

    unsigned firstReg  = MO1.getReg();
    unsigned secondReg = MO2.getReg();

    bool firstStackified = MFI->isVRegStackified(firstReg);
    bool secondStackified = MFI->isVRegStackified(secondReg);

    // case 1: both regs are not stackified:
    if (!firstStackified && !secondStackified) {
      insertLoadFromMemoryBefore(secondReg, MI);
      ss.push(secondReg);

      insertLoadFromMemoryBefore(firstReg, MI);
      ss.push(firstReg);

      ss.pop();
      ss.pop();
      return;
    }

    // case 2: the first reg is not stackified:
    if (!firstStackified && secondStackified) {
      unsigned depthFromTop = 0;
      bool result = findRegDepthOnStack(ss, secondReg, &depthFromTop);
      assert(result);

      // swap second reg to top, thend load first reg
      if (depthFromTop != 0) {
        insertSwap(depthFromTop, MI);
        ss.swap(depthFromTop);
      }

      insertLoadFromMemoryBefore(firstReg, MI);
      ss.push(firstReg);

      ss.pop();
      ss.pop();
      return;
    }

    // case 3: the second reg is not stackfieid:
    if (firstStackified && !secondStackified) {
      unsigned depthFromTop = 0;
      bool result = findRegDepthOnStack(ss, firstReg, &depthFromTop);
      assert(result);

      // 1: bring a to top
      // 2: load b
      // 3: swap1
      if (depthFromTop != 0) {
        insertSwap(depthFromTop, MI);
        ss.swap(depthFromTop);
      }

      insertLoadFromMemoryBefore(secondReg, MI);
      ss.push(secondReg);

      insertSwap(1, MI);
      ss.swap(1);

      ss.pop();
      ss.pop();
      return;
    }

    // case 4: both reg is stackified:
    unsigned firstDepthFromTop = 0;
    unsigned secondDepthFromTop = 0;
    bool result = findRegDepthOnStack(ss, firstReg, &firstDepthFromTop);
    assert(result);

    // there is a special case: both the operands are the same.
    if (firstReg == secondReg) {
      LLVM_DEBUG(
          { dbgs() << "  Special case: both operands are the same.\n"; });
      //we should skip the first one and find the second one.
      result = findRegDepthOnStack(ss, secondReg, &secondDepthFromTop,
                                   /*skip = */ 1);
    } else {
      result = findRegDepthOnStack(ss, secondReg, &secondDepthFromTop);
    }
    assert(result);

    // ideal case, we don't need to do anything
    if (firstDepthFromTop == 0 && secondDepthFromTop == 1) {
      LLVM_DEBUG({ dbgs() << "  case1.\n"; });
      // do nothing
    } else
    
    // first in position, second not in.
    if (firstDepthFromTop == 0 && secondDepthFromTop != 1) {
      LLVM_DEBUG({ dbgs() << "  case2.\n"; });
      // TODO: do if it is commutatble, optimization

      // 0: start:  a, xx, xx1, b
      // 1: swap(b, a): b, xx, xx1, a
      // 2: swap(b, xx):  xx, b, xx1, a
      // 3: swap(a, xx): a, b, xx1, xx

      // move the second operand to top, so a swap
      insertSwap(secondDepthFromTop, MI);
      ss.swap(secondDepthFromTop);

      // and another swap1 to swap the fst and snd operands
      insertSwap(1, MI);
      ss.swap(1);

      result = findRegDepthOnStack(ss, firstReg, &firstDepthFromTop);
      assert(result);

      if (firstDepthFromTop != 0) {
        insertSwap(firstDepthFromTop, MI);
        ss.swap(firstDepthFromTop);
      }
    } else 

    // second in position, first is not.
    if (firstDepthFromTop != 0 && secondDepthFromTop == 1) {
      LLVM_DEBUG({ dbgs() << "  case3.\n"; });

      // before:
      // x, b, ..., a 
      // after swap(x):
      // a, b, ..., x

      insertSwap(firstDepthFromTop, MI);
      ss.swap(firstDepthFromTop);
    } else

    // first and second are reversed
    if (firstDepthFromTop == 1 && secondDepthFromTop == 0) {
      LLVM_DEBUG({ dbgs() << "  case4.\n"; });
      insertSwap(1, MI);
      ss.swap(1);
    } else

    // special case: 
    /*
    if (firstDepthFromTop == 0 && secondDepthFromTop > 1) {
      LLVM_DEBUG({ dbgs() << "  case5.\n"; });
      // move the first operand to the correct position.
      insertSwap(1, MI);
      ss.swap(1);
      
      // then move the second operand on to the top
      insertSwap(secondDepthFromTop, MI);
      ss.swap(secondDepthFromTop);
    } else
    */
    
    // all other situations.
    if (firstDepthFromTop != 0 && secondDepthFromTop != 1) {
      LLVM_DEBUG({ dbgs() << "  case6.\n"; });
      // either registers are not in place.
      // first, swap first operand to top, then swap second operand to top

      // 1: move b to top: b, c, ..., a
      // 2: swap b and c:  c, b, ..., a
      // 3: if c is a: then doen.
      // 4: otherwise: swap c and a

      if (secondDepthFromTop != 0) {
        insertSwap(secondDepthFromTop, MI);
        ss.swap(secondDepthFromTop);
      }

      insertSwap(1, MI);
      ss.swap(1);

      result = findRegDepthOnStack(ss, firstReg, &firstDepthFromTop);
      assert(result);
      // first operand cannot be second
      assert(firstDepthFromTop != 1);

      if (firstDepthFromTop != 0) {
        insertSwap(firstDepthFromTop, MI);
        ss.swap(firstDepthFromTop);
      }

    } else{
      llvm_unreachable("missing cases for handling.");
    }

    ss.pop();
    ss.pop();
    return;
  }

  LLVM_DEBUG({
    dbgs() << "  numUsesInMI == " << numUsesInMI << ", handle specifically\n";
  });

  handleIrregularInstruction(ss, MI);
  return;
}

void EVMStackification::handleDef(StackStatus &ss, MachineInstr& MI) {
  // Look up LiveInterval info.
  // Insert DUP if it is necessary.
  unsigned numDefs = MI.getDesc().getNumDefs();
  assert(numDefs <= 1 && "more than one defs");

  // skip if there is no definition.
  if (numDefs == 0) {
    return;
  }

  MachineOperand& def = *MI.defs().begin();
  assert(def.isReg() && def.isDef());
  unsigned defReg = def.getReg();

  // insert pop if it does not have def.
  bool IsDead = MRI->use_empty(defReg);
  if (IsDead) {
    LLVM_DEBUG({ dbgs() << "  Def's use is empty, insert POP after.\n"; });
    insertPop(MI);
    return;
  }

  if (!canStackifyReg(defReg, MI)) {
    assert(!Register::isPhysicalRegister(defReg));

    LLVM_DEBUG({
      unsigned ridx = Register::virtReg2Index(defReg);
      dbgs() << "  Cannot stackify %" << ridx << "\n";
    });

    MFI->allocate_memory_index(defReg);
    insertStoreToMemory(defReg, MI);
    return;
  }

  // stackify the register
  assert(!MFI->isVRegStackified(defReg));
  MFI->stackifyVReg(defReg);

  LLVM_DEBUG({
    unsigned ridx = Register::virtReg2Index(defReg);
    dbgs() << "  Reg %" << ridx << " is stackified.\n";
  });

  ss.push(defReg);

  unsigned numUses = std::distance(MRI->use_begin(defReg), MRI->use_end());
  for (unsigned i = 1; i < numUses; ++i) {
    insertDup(1, MI);
    ss.dup(0);
  }

}

typedef struct {
  unsigned reg;
  bool canStackify;
} Sarg;

void EVMStackification::handleEntryMBB(StackStatus &ss, MachineBasicBlock &MBB) {
    assert(ss.getStackDepth() == 0);

    std::vector<Sarg> canStackifyStackarg;

    LLVM_DEBUG({ dbgs() << "// start of handling stack args.\n"; });
    // iterate over stackargs:
    MachineBasicBlock::iterator SI;
    for (MachineBasicBlock::iterator I = MBB.begin(), E = MBB.end();
         I != E; ++I) {
      MachineInstr &MI = *I;

      if (MI.getOpcode() != EVM::pSTACKARG_r) {
        SI = I;
        break;
      }
      // record stack arg status.
      unsigned reg = MI.getOperand(0).getReg();
      bool canStackfy = canStackifyReg(reg, MI);
      Sarg x{reg, canStackfy};
      canStackifyStackarg.push_back(x);

      LLVM_DEBUG({
        unsigned ridx = Register::virtReg2Index(reg);
        dbgs() << "Stackarg Reg %" << ridx << " is stackifiable? "
               << canStackfy << "\n";
      });

      // we should also update stackstatus:
      ss.push(reg);
      ss.dump();
    }

    // This is the instruction of the first non-stackarg instruction.
    MachineInstr &MI = *SI;
    LLVM_DEBUG({
      dbgs() << "First non-stack arg instruction:";
      MI.dump();
    });

    // from top to bottom.
    std::reverse(canStackifyStackarg.begin(), canStackifyStackarg.end());

    // insert stack manipulation code here.
    for (unsigned index = 0; index < canStackifyStackarg.size();  ++index) {
      Sarg pos = canStackifyStackarg[index];

      unsigned depth = 0;
      bool found = findRegDepthOnStack(ss, pos.reg, &depth);
      assert(found);

      LLVM_DEBUG({
        unsigned ridx = Register::virtReg2Index(pos.reg);
        dbgs() << "Handling stackarg  %" << ridx << "\n"; 
      });

      if (pos.canStackify) {
        // duplicate on to top of stack.
        unsigned numUses =
            std::distance(MRI->use_begin(pos.reg), MRI->use_end());
        LLVM_DEBUG({ dbgs() << "  Num of uses: " << numUses << "\n"; });

        for (unsigned i = 1; i < numUses; ++i) {
          if (i == 1) {
            insertDup(depth + 1, MI, false);
            ss.dup(depth); 
          } else {
            // dup the top
            insertDup(1, MI, false);
            ss.dup(0);
          }
        }

        // stackify the register
        assert(!MFI->isVRegStackified(pos.reg));
        MFI->stackifyVReg(pos.reg);
        ss.dump();
      } else {
        if (depth != 0) {
          // We can't stackify it:
          // SWAP and then store.
          insertSwap(depth, MI);
          ss.swap(depth);
        }

        MFI->allocate_memory_index(pos.reg);
        // we actually need to insert BEFORE
        insertStoreToMemory(pos.reg, MI, false);
        ss.pop();
        ss.dump();
      }
    }

    LLVM_DEBUG({
      dbgs() << "// end of handling stack args, next instr:";
      (*SI).dump();
    });

    for (MachineBasicBlock::iterator I = SI, E = MBB.end();
         I != E;) {
      MachineInstr &MI = *I++;

      LLVM_DEBUG({
        dbgs() << "Stackifying instr: ";
        MI.dump();
      });

      // If the Use is stackified:
      // insert SWAP if necessary
      handleUses(ss, MI);

      // If the Def is able to be stackified:
      // 1. mark VregStackified
     // 2. insert DUP if necessary
      handleDef(ss, MI);

      ss.dump();
    }

}

void EVMStackification::handleMBB(StackStatus &ss, MachineBasicBlock &MBB) {
    assert(ss.getStackDepth() == 0);
    // The scheduler has already set the sequence for us. We just need to
    // iterate over by order.
    for (MachineBasicBlock::iterator I = MBB.begin(), E = MBB.end();
         I != E;) {
      MachineInstr &MI = *I++;

      LLVM_DEBUG({
        dbgs() << "Stackifying instr: ";
        MI.dump();
      });

      // If the Use is stackified:
      // insert SWAP if necessary
      handleUses(ss, MI);

      // If the Def is able to be stackified:
      // 1. mark VregStackified
      // 2. insert DUP if necessary
      handleDef(ss, MI);

      ss.dump();
    }
}

bool EVMStackification::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG({
    dbgs() << "********** Stackification **********\n"
           << "********** Function: " << MF.getName() << '\n';
  });

  // Prepare


  // iterate BBs from the entry point.

  // For each BBs, examine the live-ins.
  // For each MI in BB:
  //  For each DEF:
  //   1. if it is assigned on memory, insert store
  //   2. if it is assigned to P_stack:
  //   3. if it is assigned to L_stack: do nothing, leave it on stack.
  //   4. if it is assigned to X_stack: 
  //  For each USE:
  //   1. if it is NONSTACK: insert load
  //   2. if it is not the last USE:
  //     Insert DUP.
  //       If a sucessor is outside the liveness range: POP it.
  //       If a sucessor is inside the liveness range: need to find a place to pop it. (TODO: Investigate)
  //   3. if it is the last USE: insert SWAP


  return true;
}
