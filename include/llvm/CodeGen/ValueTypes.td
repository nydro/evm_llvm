//===- ValueTypes.td - ValueType definitions ---------------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Value types - These values correspond to the register types defined in the
// MachineValueTypes.h file.  If you update anything here, you must update it
// there as well!
//
//===----------------------------------------------------------------------===//

class ValueType<int size, int value> {
  string Namespace = "MVT";
  int Size = size;
  int Value = value;
}

def OtherVT: ValueType<0  ,  1>;   // "Other" value
def i1     : ValueType<1  ,  2>;   // One bit boolean value
def i8     : ValueType<8  ,  3>;   // 8-bit integer value
def i16    : ValueType<16 ,  4>;   // 16-bit integer value
def i32    : ValueType<32 ,  5>;   // 32-bit integer value
def i64    : ValueType<64 ,  6>;   // 64-bit integer value
def i128   : ValueType<128,  7>;   // 128-bit integer value
def i256   : ValueType<256,  8>;   // 256-bit integer value
def f16    : ValueType<16 ,  9>;   // 16-bit floating point value
def f32    : ValueType<32 , 10>;   // 32-bit floating point value
def f64    : ValueType<64 , 11>;   // 64-bit floating point value
def f80    : ValueType<80 , 12>;   // 80-bit floating point value
def f128   : ValueType<128, 13>;   // 128-bit floating point value
def ppcf128: ValueType<128, 14>;   // PPC 128-bit floating point value

def v1i1   : ValueType<1 ,  15>;   //   1 x i1 vector value
def v2i1   : ValueType<2 ,  16>;   //   2 x i1 vector value
def v4i1   : ValueType<4 ,  17>;   //   4 x i1 vector value
def v8i1   : ValueType<8 ,  18>;   //   8 x i1 vector value
def v16i1  : ValueType<16,  19>;   //  16 x i1 vector value
def v32i1  : ValueType<32 , 20>;   //  32 x i1 vector value
def v64i1  : ValueType<64 , 21>;   //  64 x i1 vector value
def v128i1 : ValueType<128, 22>;   // 128 x i1 vector value
def v512i1 : ValueType<512, 23>;   // 512 x i1 vector value
def v1024i1: ValueType<1024,24>;   //1024 x i1 vector value

def v1i8   : ValueType<8,   25>;   //  1 x i8  vector value
def v2i8   : ValueType<16 , 26>;   //  2 x i8  vector value
def v4i8   : ValueType<32 , 27>;   //  4 x i8  vector value
def v8i8   : ValueType<64 , 28>;   //  8 x i8  vector value
def v16i8  : ValueType<128, 29>;   // 16 x i8  vector value
def v32i8  : ValueType<256, 30>;   // 32 x i8  vector value
def v64i8  : ValueType<512, 31>;   // 64 x i8  vector value
def v128i8 : ValueType<1024,32>;   //128 x i8  vector value
def v256i8 : ValueType<2048,33>;   //256 x i8  vector value

def v1i16  : ValueType<16 , 34>;   //  1 x i16 vector value
def v2i16  : ValueType<32 , 35>;   //  2 x i16 vector value
def v4i16  : ValueType<64 , 36>;   //  4 x i16 vector value
def v8i16  : ValueType<128, 37>;   //  8 x i16 vector value
def v16i16 : ValueType<256, 38>;   // 16 x i16 vector value
def v32i16 : ValueType<512, 39>;   // 32 x i16 vector value
def v64i16 : ValueType<1024,40>;   // 64 x i16 vector value
def v128i16: ValueType<2048,41>;   //128 x i16 vector value

def v1i32  : ValueType<32 , 42>;   //  1 x i32 vector value
def v2i32  : ValueType<64 , 43>;   //  2 x i32 vector value
def v3i32  : ValueType<96 , 44>;   //  3 x i32 vector value
def v4i32  : ValueType<128, 45>;   //  4 x i32 vector value
def v5i32  : ValueType<160, 46>;   //  5 x i32 vector value
def v8i32  : ValueType<256, 47>;   //  8 x i32 vector value
def v16i32 : ValueType<512, 48>;   // 16 x i32 vector value
def v32i32 : ValueType<1024,49>;   // 32 x i32 vector value
def v64i32 : ValueType<2048,50>;   // 64 x i32 vector value

def v1i64  : ValueType<64 , 51>;   //  1 x i64 vector value
def v2i64  : ValueType<128, 52>;   //  2 x i64 vector value
def v4i64  : ValueType<256, 53>;   //  4 x i64 vector value
def v8i64  : ValueType<512, 54>;   //  8 x i64 vector value
def v16i64 : ValueType<1024,55>;   // 16 x i64 vector value
def v32i64 : ValueType<2048,56>;   // 32 x i64 vector value

def v1i128 : ValueType<128, 57>;   //  1 x i128 vector value

def nxv1i1  : ValueType<1,   58>;  // n x  1 x i1  vector value
def nxv2i1  : ValueType<2,   59>;  // n x  2 x i1  vector value
def nxv4i1  : ValueType<4,   60>;  // n x  4 x i1  vector value
def nxv8i1  : ValueType<8,   61>;  // n x  8 x i1  vector value
def nxv16i1 : ValueType<16,  62>;  // n x 16 x i1  vector value
def nxv32i1 : ValueType<32,  63>;  // n x 32 x i1  vector value

def nxv1i8  : ValueType<8,   64>;  // n x  1 x i8  vector value
def nxv2i8  : ValueType<16,  65>;  // n x  2 x i8  vector value
def nxv4i8  : ValueType<32,  66>;  // n x  4 x i8  vector value
def nxv8i8  : ValueType<64,  67>;  // n x  8 x i8  vector value
def nxv16i8 : ValueType<128, 68>;  // n x 16 x i8  vector value
def nxv32i8 : ValueType<256, 69>;  // n x 32 x i8  vector value

def nxv1i16 : ValueType<16,  70>;  // n x  1 x i16 vector value
def nxv2i16 : ValueType<32,  71>;  // n x  2 x i16 vector value
def nxv4i16 : ValueType<64,  72>;  // n x  4 x i16 vector value
def nxv8i16 : ValueType<128, 73>;  // n x  8 x i16 vector value
def nxv16i16: ValueType<256, 74>;  // n x 16 x i16 vector value
def nxv32i16: ValueType<512, 75>;  // n x 32 x i16 vector value

def nxv1i32 : ValueType<32,  76>;  // n x  1 x i32 vector value
def nxv2i32 : ValueType<64,  77>;  // n x  2 x i32 vector value
def nxv4i32 : ValueType<128, 78>;  // n x  4 x i32 vector value
def nxv8i32 : ValueType<256, 79>;  // n x  8 x i32 vector value
def nxv16i32: ValueType<512, 80>;  // n x 16 x i32 vector value
def nxv32i32: ValueType<1024,81>;  // n x 32 x i32 vector value

def nxv1i64 : ValueType<64,  82>;  // n x  1 x i64 vector value
def nxv2i64 : ValueType<128, 83>;  // n x  2 x i64 vector value
def nxv4i64 : ValueType<256, 84>;  // n x  4 x i64 vector value
def nxv8i64 : ValueType<512, 85>;  // n x  8 x i64 vector value
def nxv16i64: ValueType<1024,86>;  // n x 16 x i64 vector value
def nxv32i64: ValueType<2048,87>;  // n x 32 x i64 vector value

def v2f16  : ValueType<32 , 88>;   //  2 x f16 vector value
def v4f16  : ValueType<64 , 89>;   //  4 x f16 vector value
def v8f16  : ValueType<128, 90>;   //  8 x f16 vector value
def v1f32  : ValueType<32 , 91>;   //  1 x f32 vector value
def v2f32  : ValueType<64 , 92>;   //  2 x f32 vector value
def v3f32  : ValueType<96 , 93>;   //  3 x f32 vector value
def v4f32  : ValueType<128, 94>;   //  4 x f32 vector value
def v5f32  : ValueType<160, 95>;   //  5 x f32 vector value
def v8f32  : ValueType<256, 96>;   //  8 x f32 vector value
def v16f32 : ValueType<512, 97>;   // 16 x f32 vector value
def v1f64  : ValueType<64,  98>;   //  1 x f64 vector value
def v2f64  : ValueType<128, 99>;   //  2 x f64 vector value
def v4f64  : ValueType<256,100>;   //  4 x f64 vector value
def v8f64  : ValueType<512,101>;   //  8 x f64 vector value

def nxv2f16  : ValueType<32 , 102>; // n x  2 x f16 vector value
def nxv4f16  : ValueType<64 , 103>; // n x  4 x f16 vector value
def nxv8f16  : ValueType<128, 104>; // n x  8 x f16 vector value
def nxv1f32  : ValueType<32 , 105>; // n x  1 x f32 vector value
def nxv2f32  : ValueType<64 , 106>; // n x  2 x f32 vector value
def nxv4f32  : ValueType<128, 107>; // n x  4 x f32 vector value
def nxv8f32  : ValueType<256, 108>; // n x  8 x f32 vector value
def nxv16f32 : ValueType<512, 109>; // n x 16 x f32 vector value
def nxv1f64  : ValueType<64,  110>; // n x  1 x f64 vector value
def nxv2f64  : ValueType<128, 111>; // n x  2 x f64 vector value
def nxv4f64  : ValueType<256, 112>; // n x  4 x f64 vector value
def nxv8f64  : ValueType<512, 113>; // n x  8 x f64 vector value

def x86mmx : ValueType<64 , 114>;   // X86 MMX value
def FlagVT : ValueType<0  , 115>;   // Pre-RA sched glue
def isVoid : ValueType<0  , 116>;   // Produces no value
def untyped: ValueType<8  , 117>;   // Produces an untyped value
def ExceptRef: ValueType<0, 118>;   // WebAssembly's except_ref type
def token  : ValueType<0  , 248>;   // TokenTy
def MetadataVT: ValueType<0, 249>;  // Metadata

// Pseudo valuetype mapped to the current pointer size to any address space.
// Should only be used in TableGen.
def iPTRAny   : ValueType<0, 250>;

// Pseudo valuetype to represent "vector of any size"
def vAny   : ValueType<0  , 251>;

// Pseudo valuetype to represent "float of any format"
def fAny   : ValueType<0  , 252>;

// Pseudo valuetype to represent "integer of any bit width"
def iAny   : ValueType<0  , 253>;

// Pseudo valuetype mapped to the current pointer size.
def iPTR   : ValueType<0  , 254>;

// Pseudo valuetype to represent "any type of any size".
def Any    : ValueType<0  , 255>;

/// This class is for targets that want to use pointer types in patterns
/// with the GlobalISelEmitter.  Targets must define their own pointer
/// derived from this class.  The scalar argument should be an
/// integer type with the same bit size as the ponter.
/// e.g. def p0 : PtrValueType <i64, 0>;

class PtrValueType <ValueType scalar, int addrspace> :
    ValueType<scalar.Size, scalar.Value> {
  int AddrSpace = addrspace;
}
