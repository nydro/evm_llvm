//===-- EVMRearrangeFunctions.cpp - Rearraing the order of functions ---------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_EVM_REARRANGE_FUNCTIONS_H
#define LLVM_EVM_REARRANGE_FUNCTIONS_H

#include "EVM.h"
#include "EVMSubtarget.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/IR/PassManager.h"
using namespace llvm;

namespace {

class Function;
class Module;

class EVMRearrangeFunctions final : public ModulePass {
public:
  static char ID; // Pass identification, replacement for typeid
  EVMRearrangeFunctions();

  bool runOnModule(Module &M);
};
} // end anonymous namespace

ModulePass *createEVMRearrangeFunctions();

#endif //LLVM_EVM_REARRANGE_FUNCTIONS_H
