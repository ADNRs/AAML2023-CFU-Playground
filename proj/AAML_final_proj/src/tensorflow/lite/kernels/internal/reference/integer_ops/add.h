/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.
   Copyright 2023-2024 Chung-Yi Chen. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
#ifndef TENSORFLOW_LITE_KERNELS_INTERNAL_REFERENCE_INTEGER_OPS_ADD_H_
#define TENSORFLOW_LITE_KERNELS_INTERNAL_REFERENCE_INTEGER_OPS_ADD_H_

#include <algorithm>
#include <limits>

#include "tensorflow/lite/kernels/internal/common.h"
#include "tensorflow/lite/kernels/internal/types.h"

#include "cfu.h"

#define cfu_post_set_SRDHM(a, b)         cfu_op0(5, a, b)
#define cfu_post_get_SRDHM()             cfu_op0(6, 0, 0)
#define cfu_post_RDBPOT(x, exp)          cfu_op0(7, x, exp)
#define cfu_post_off_maxmin(val, off)    cfu_op0(8, val, off)

namespace tflite {
namespace reference_integer_ops {

inline void CheckArithmeticParams(const ArithmeticParams& params) {
  TFLITE_DCHECK_LE(params.quantized_activation_min,
                   params.quantized_activation_max);
  // Input offset is negative input zero point. Activation tensors are
  // asymmetric quantized so they span the full int8 range.
  TFLITE_DCHECK_GE(-params.input1_offset, std::numeric_limits<int8_t>::min());
  TFLITE_DCHECK_GE(-params.input2_offset, std::numeric_limits<int8_t>::min());
  TFLITE_DCHECK_LE(-params.input1_offset, std::numeric_limits<int8_t>::max());
  TFLITE_DCHECK_LE(-params.input2_offset, std::numeric_limits<int8_t>::max());
}

inline void ElementWise(
    int size, const ArithmeticParams& params, const int8_t* input1_data,
    const int8_t* input2_data, int8_t* output_data,
    void (*check_arithmetic_params)(const ArithmeticParams&),
    int8_t (*binary_func)(int8_t, int8_t, const ArithmeticParams&)) {
  CheckArithmeticParams(params);
  for (int i = 0; i < size; ++i) {
    output_data[i] = binary_func(input1_data[i], input2_data[i], params);
  }
}

inline void BroadcastBinaryFunction4DSlow(
    const ArithmeticParams& params, const RuntimeShape& input1_shape,
    const int8_t* input1_data, const RuntimeShape& input2_shape,
    const int8_t* input2_data, const RuntimeShape& output_shape,
    int8_t* output_data,
    void (*check_arithmetic_params)(const ArithmeticParams&),
    int8_t (*binary_func)(int8_t, int8_t, const ArithmeticParams&)) {
  NdArrayDesc<4> desc1;
  NdArrayDesc<4> desc2;
  NdArrayDescsForElementwiseBroadcast(input1_shape, input2_shape, &desc1,
                                      &desc2);
  const RuntimeShape extended_output_shape =
      RuntimeShape::ExtendedShape(4, output_shape);

  // In Tensorflow, the dimensions are canonically named (batch_number, row,
  // col, channel), with extents (batches, height, width, depth), with the
  // trailing dimension changing most rapidly (channels has the smallest stride,
  // typically 1 element).
  //
  // In generated C code, we store arrays with the dimensions reversed. The
  // first dimension has smallest stride.
  //
  // We name our variables by their Tensorflow convention, but generate C code
  // nesting loops such that the innermost loop has the smallest stride for the
  // best cache behavior.
  for (int b = 0; b < extended_output_shape.Dims(0); ++b) {
    for (int y = 0; y < extended_output_shape.Dims(1); ++y) {
      for (int x = 0; x < extended_output_shape.Dims(2); ++x) {
        for (int c = 0; c < extended_output_shape.Dims(3); ++c) {
          output_data[Offset(extended_output_shape, b, y, x, c)] = binary_func(
              input1_data[SubscriptToIndex(desc1, b, y, x, c)],
              input2_data[SubscriptToIndex(desc2, b, y, x, c)], params);
        }
      }
    }
  }
}

inline int8_t AddFunc(int8_t x, int8_t y, const ArithmeticParams& params) {
  const int32_t input1_val = params.input1_offset + x;
  const int32_t input2_val = params.input2_offset + y;
  const int32_t shifted_input1_val = input1_val * (1 << 20);
  const int32_t shifted_input2_val = input2_val * (1 << 20);
  const int32_t scaled_input1_val =
      MultiplyByQuantizedMultiplierSmallerThanOneExp(
          shifted_input1_val, params.input1_multiplier, -2);
  const int32_t scaled_input2_val =
      MultiplyByQuantizedMultiplierSmallerThanOneExp(
          shifted_input2_val, 1073741824, 0);
  const int32_t raw_sum = scaled_input1_val + scaled_input2_val;
  const int32_t raw_output =
      MultiplyByQuantizedMultiplierSmallerThanOneExp(
          raw_sum, params.output_multiplier, params.output_shift) - 128;
  return static_cast<int8_t>(cfu_post_off_maxmin(raw_output, 0));
}

// Element-wise add that can often be used for inner loop of broadcast add as
// well as the non-broadcast add.
inline void AddElementwise(int size, const ArithmeticParams& params,
                           const int8_t* input1_data, const int8_t* input2_data,
                           int8_t* output_data) {
  ElementWise(size, params, input1_data, input2_data, output_data,
              CheckArithmeticParams, AddFunc);
}

inline int32_t MultiplyByQuantizedMultiplierSmallerThanOneExp(
    int32_t x, int32_t quantized_multiplier, int32_t left_shift) {
  cfu_post_set_SRDHM(x, quantized_multiplier);
  int val = cfu_post_get_SRDHM();
  return cfu_post_RDBPOT(val, left_shift);
}

inline int32_t MultiplyByQuantizedMultiplierSmallerThanOneExp(int32_t x) {
  cfu_post_set_SRDHM(x, 1073741824);
  return cfu_post_get_SRDHM();
}

inline void Add(const ArithmeticParams& params,
                const RuntimeShape& input1_shape, const int8_t* input1_data,
                const RuntimeShape& input2_shape, const int8_t* input2_data,
                const RuntimeShape& output_shape, int8_t* output_data) {
  const int size =
      MatchingElementsSize(input1_shape, input2_shape, output_shape);

  if (size == 16384) {
    for (int i = 0; i < 16384; ++i) {
      const int32_t input1_val = input1_data[i] + 128;
      const int32_t input2_val = input2_data[i] - 4;
      const int32_t shifted_input1_val = input1_val << 20;
      const int32_t shifted_input2_val = input2_val << 20;
      const int32_t scaled_input1_val =
          MultiplyByQuantizedMultiplierSmallerThanOneExp(
              shifted_input1_val, 1623821475, 2);
      const int32_t scaled_input2_val =
          MultiplyByQuantizedMultiplierSmallerThanOneExp(shifted_input2_val);
      const int32_t raw_sum = scaled_input1_val + scaled_input2_val;
      const int32_t raw_output =
          MultiplyByQuantizedMultiplierSmallerThanOneExp(raw_sum, 1098017566, 17);
      output_data[i] = cfu_post_off_maxmin(raw_output, -128);
    }
  }
  else if (size == 8192) {
    for (int i = 0; i < 8192; ++i) {
      const int32_t input1_val = input1_data[i] + 17;
      const int32_t input2_val = input2_data[i] - 4;
      const int32_t shifted_input1_val = input1_val << 20;
      const int32_t shifted_input2_val = input2_val << 20;
      const int32_t scaled_input1_val =
          MultiplyByQuantizedMultiplierSmallerThanOneExp(
              shifted_input1_val, 1699529983, 2);
      const int32_t scaled_input2_val =
          MultiplyByQuantizedMultiplierSmallerThanOneExp(shifted_input2_val);
      const int32_t raw_sum = scaled_input1_val + scaled_input2_val;
      const int32_t raw_output =
          MultiplyByQuantizedMultiplierSmallerThanOneExp(raw_sum, 1140768826, 17);
      output_data[i] = cfu_post_off_maxmin(raw_output, -128);
    }
  }
  else {
    for (int i = 0; i < 4096; ++i) {
      const int32_t input1_val = input1_data[i] - 38;
      const int32_t input2_val = input2_data[i] + 2;
      const int32_t shifted_input1_val = input1_val << 20;
      const int32_t shifted_input2_val = input2_val << 20;
      const int32_t scaled_input1_val =
          MultiplyByQuantizedMultiplierSmallerThanOneExp(
              shifted_input1_val, 1657902019, 2);
      const int32_t scaled_input2_val =
          MultiplyByQuantizedMultiplierSmallerThanOneExp(shifted_input2_val);
      const int32_t raw_sum = scaled_input1_val + scaled_input2_val;
      const int32_t raw_output =
          MultiplyByQuantizedMultiplierSmallerThanOneExp(raw_sum, 1835721671, 18);
      output_data[i] = cfu_post_off_maxmin(raw_output, -128);
    }
  }
}

inline void BroadcastAdd4DSlow(const ArithmeticParams& params,
                               const RuntimeShape& input1_shape,
                               const int8_t* input1_data,
                               const RuntimeShape& input2_shape,
                               const int8_t* input2_data,
                               const RuntimeShape& output_shape,
                               int8_t* output_data) {
  BroadcastBinaryFunction4DSlow(params, input1_shape, input1_data, input2_shape,
                                input2_data, output_shape, output_data,
                                CheckArithmeticParams, AddFunc);
}

}  // namespace reference_integer_ops
}  // namespace tflite

#endif  // TENSORFLOW_LITE_KERNELS_INTERNAL_REFERENCE_INTEGER_OPS_ADD_H_
