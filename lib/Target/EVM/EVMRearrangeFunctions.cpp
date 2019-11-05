//===-- EVMRearrangeFunctions.cpp - Argument instruction moving ---------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "EVMRearrangeFunctions.h"

using namespace llvm;

#define DEBUG_TYPE "evm-rearrange-functions"


char EVMRearrangeFunctions::ID = 0;

INITIALIZE_PASS(EVMRearrangeFunctions, DEBUG_TYPE,
                "Rearrange functions so the contract constructor is the first",
                false, false)

EVMRearrangeFunctions::EVMRearrangeFunctions() : ModulePass(ID) {}

ModulePass *llvm::createEVMRearrangeFunctions() {
  return new EVMRearrangeFunctions();
}

bool EVMRearrangeFunctions::runOnModule(Module &M) {

  return false;
}
