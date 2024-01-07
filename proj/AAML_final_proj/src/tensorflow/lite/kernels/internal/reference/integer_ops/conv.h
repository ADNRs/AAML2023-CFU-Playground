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
#ifndef TENSORFLOW_LITE_KERNELS_INTERNAL_REFERENCE_INTEGER_OPS_CONV_H_
#define TENSORFLOW_LITE_KERNELS_INTERNAL_REFERENCE_INTEGER_OPS_CONV_H_

#include <algorithm>

#include "tensorflow/lite/kernels/internal/common.h"
#include "tensorflow/lite/kernels/internal/portable_tensor_utils.h"

#include "cfu.h"

namespace tflite {
namespace reference_integer_ops {

#define cfu_set_matrixA(uval, lval)      cfu_op0(0, uval, lval)
#define cfu_set_matrixB(uval, lval)      cfu_op0(1, uval, lval)
#define cfu_start_GEMM(M, N, K, off)     cfu_op0(2, M<<20 | N<<10 | K, off)
#define cfu_start_GEMM_acc(M, N, K, off) cfu_op0(2, 1<<30 | M<<20 | N<<10 | K, off)
#define cfu_check_GEMM()                 cfu_op0(3, 0, 0)
#define cfu_get_matrixC()                cfu_op0(4, 0, 0)
#define cfu_post_set_SRDHM(a, b)         cfu_op0(5, a, b)
#define cfu_post_get_SRDHM()             cfu_op0(6, 0, 0)
#define cfu_post_RDBPOT(x, exp)          cfu_op0(7, x, exp)
#define cfu_post_off_maxmin(val, off)    cfu_op0(8, val, off)

const int H_tile = 256;
const int W_tile = 256;

inline int round_to_smallest_multiple(int val, int base) {
  return val + (val % base ? base - val%base : 0);
}

inline int get_buffer_rows(int M, int N) {
  return M * ((N + 15) / 16);
}

#define implicit_GEMM()                                                     \
do {                                                                        \
                                                                            \
for (int m = 0; m < pad_M; m += W_tile) {                                   \
  const int MM = std::min(m + W_tile, M) - m;                               \
                                                                            \
  for (int n = 0; n < pad_N; n += W_tile) {                                 \
    const int NN = std::min(n + W_tile, N) - n;                             \
                                                                            \
    for (int k = 0; k < pad_K; k += H_tile) {                               \
      const int KK = std::min(k + H_tile, K) - k;                           \
                                                                            \
      for (int mm = 0; mm < MM; mm += 16) {                                 \
        for (int kk = 0; kk < KK; kk++) {                                   \
          int8_t lowerA[] = {                                               \
            im2col(k + kk, m + mm + 7),                                     \
            im2col(k + kk, m + mm + 6),                                     \
            im2col(k + kk, m + mm + 5),                                     \
            im2col(k + kk, m + mm + 4)                                      \
          };                                                                \
                                                                            \
          int8_t upperA[] = {                                               \
            im2col(k + kk, m + mm + 3),                                     \
            im2col(k + kk, m + mm + 2),                                     \
            im2col(k + kk, m + mm + 1),                                     \
            im2col(k + kk, m + mm)                                          \
          };                                                                \
                                                                            \
          cfu_set_matrixA(*(int32_t*)&upperA, *(int32_t*)&lowerA);          \
                                                                            \
          int8_t lowerB[] = {                                               \
            im2col(k + kk, m + mm + 15),                                    \
            im2col(k + kk, m + mm + 14),                                    \
            im2col(k + kk, m + mm + 13),                                    \
            im2col(k + kk, m + mm + 12)                                     \
          };                                                                \
                                                                            \
          int8_t upperB[] = {                                               \
            im2col(k + kk, m + mm + 11),                                    \
            im2col(k + kk, m + mm + 10),                                    \
            im2col(k + kk, m + mm + 9),                                     \
            im2col(k + kk, m + mm + 8)                                      \
          };                                                                \
                                                                            \
          cfu_set_matrixA(*(int32_t*)&upperB, *(int32_t*)&lowerB);          \
        }                                                                   \
      }                                                                     \
                                                                            \
      for (int nn = 0; nn < NN; nn += 16) {                                 \
        for (int kk = 0; kk < KK; kk++) {                                   \
          int8_t lowerA[] = {                                               \
            kernel(k + kk, n + nn + 7),                                     \
            kernel(k + kk, n + nn + 6),                                     \
            kernel(k + kk, n + nn + 5),                                     \
            kernel(k + kk, n + nn + 4)                                      \
          };                                                                \
                                                                            \
          int8_t upperA[] = {                                               \
            kernel(k + kk, n + nn + 3),                                     \
            kernel(k + kk, n + nn + 2),                                     \
            kernel(k + kk, n + nn + 1),                                     \
            kernel(k + kk, n + nn)                                          \
          };                                                                \
                                                                            \
          cfu_set_matrixB(*(int32_t*)&upperA, *(int32_t*)&lowerA);          \
                                                                            \
          int8_t lowerB[] = {                                               \
            kernel(k + kk, n + nn + 15),                                    \
            kernel(k + kk, n + nn + 14),                                    \
            kernel(k + kk, n + nn + 13),                                    \
            kernel(k + kk, n + nn + 12)                                     \
          };                                                                \
                                                                            \
          int8_t upperB[] = {                                               \
            kernel(k + kk, n + nn + 11),                                    \
            kernel(k + kk, n + nn + 10),                                    \
            kernel(k + kk, n + nn + 9),                                     \
            kernel(k + kk, n + nn + 8)                                      \
          };                                                                \
                                                                            \
          cfu_set_matrixB(*(int32_t*)&upperB, *(int32_t*)&lowerB);          \
        }                                                                   \
      }                                                                     \
                                                                            \
      if (k == 0) cfu_start_GEMM(MM, NN, KK, 128);                          \
      else cfu_start_GEMM_acc(MM, NN, KK, 128);                             \
      while (cfu_check_GEMM());                                             \
    }                                                                       \
                                                                            \
    for (int r = 0; r < get_buffer_rows(MM, NN); r++) {                     \
      const int mm = r % MM;                                                \
      const int nn = r / MM * 16;                                           \
                                                                            \
      for (int i = 0; i < 16; i++) {                                        \
        int32_t acc = cfu_get_matrixC() + bias_data[n + nn + i];            \
                                                                            \
        acc = MultiplyByQuantizedMultiplier(                                \
          acc, output_multiplier[n + nn + i], output_shift[n + nn + i]);    \
        acc = cfu_post_off_maxmin(acc, output_offset);                      \
                                                                            \
        const int out_y = (m + mm) / W_output;                              \
        const int out_x = (m + mm) % W_output;                              \
                                                                            \
        output_data[Offset(output_shape, 0, out_y, out_x, n + nn + i)] =    \
          static_cast<int8_t>(acc);                                         \
      }                                                                     \
    }                                                                       \
  }                                                                         \
}                                                                           \
                                                                            \
} while (false)

inline int32_t MultiplyByQuantizedMultiplier(int32_t x, int32_t quantized_multiplier,
                                           int32_t shift) {
  if (shift < 0) {
    cfu_post_set_SRDHM(x, quantized_multiplier);
    int val = cfu_post_get_SRDHM();
    return cfu_post_RDBPOT(val, -shift);
  }
  else [[unlikely]] {
    cfu_post_set_SRDHM(x << shift, quantized_multiplier);
    return cfu_post_get_SRDHM();
  }
}

// Fixed-point per-channel-quantization convolution reference kernel.
inline void ConvPerChannel(
    const ConvParams& params, const int32_t* output_multiplier,
    const int32_t* output_shift, const RuntimeShape& input_shape,
    const int8_t* input_data, const RuntimeShape& filter_shape,
    const int8_t* filter_data, const RuntimeShape& bias_shape,
    const int32_t* bias_data, const RuntimeShape& output_shape,
    int8_t* output_data) {
  const int H_input = input_shape.Dims(1);
  const int W_input = input_shape.Dims(2);
  const int C_input = input_shape.Dims(3);
  const int H_output = output_shape.Dims(1);
  const int W_output = output_shape.Dims(2);
  const int C_output = output_shape.Dims(3);
  const int H_kernel = filter_shape.Dims(1);
  const int W_kernel = filter_shape.Dims(2);
  const int32_t output_offset = params.output_offset;

  const int M = H_output * W_output;
  const int K = H_kernel * W_kernel * C_input;
  const int N = C_output;

  const int pad_M = round_to_smallest_multiple(M, W_tile);
  const int pad_N = round_to_smallest_multiple(N, W_tile);
  const int pad_K = round_to_smallest_multiple(K, H_tile);

  if (params.stride_height == 1) {
    auto im2col = [&](const int out_y, const int out_x) {
      const int in_channel = out_y / 9;
      const int kernel_id = out_y % 9;
      const int y = kernel_id / 3;
      const int x = kernel_id % 3;
      const int in_y = y + out_x/W_output - 1;
      const int in_x = x + out_x%W_output - 1;
      const bool is_point_inside_image =
        (uint32_t)in_y < (uint32_t)(H_input) &&
        (uint32_t)in_x < (uint32_t)(W_input);
      if (!is_point_inside_image) return (int8_t)-128;
      return input_data[Offset(input_shape, 0, in_y, in_x, in_channel)];
    };

    auto kernel = [&](const int out_y, const int out_x) {
      const int in_channel = out_y / 9;
      const int kernel_id = out_y % 9;
      const int y = kernel_id / 3;
      const int x = kernel_id % 3;
      return filter_data[Offset(filter_shape, out_x, y, x, in_channel)];
    };

    implicit_GEMM();
  }
  else if (H_kernel == 3) {
    auto im2col = [&](const int out_y, const int out_x) {
      const int in_channel = out_y / 9;
      const int kernel_id = out_y % 9;
      const int y = kernel_id / 3;
      const int x = kernel_id % 3;
      const int in_y = y + (out_x/W_output << 1);
      const int in_x = x + (out_x%W_output << 1);
      const bool is_point_inside_image = in_y < H_input and in_x < W_input;
      if (!is_point_inside_image) return (int8_t)-128;
      return input_data[Offset(input_shape, 0, in_y, in_x, in_channel)];
    };

    auto kernel = [&](const int out_y, const int out_x) {
      const int in_channel = out_y / 9;
      const int kernel_id = out_y % 9;
      const int y = kernel_id / 3;
      const int x = kernel_id % 3;
      return filter_data[Offset(filter_shape, out_x, y, x, in_channel)];
    };

    implicit_GEMM();
  }
  else {
    auto im2col = [&](const int out_y, const int out_x) {
      const int in_y = out_x/W_output << 1;
      const int in_x = out_x%W_output << 1;
      const bool is_point_inside_image = in_y < H_input and in_x < W_input;
      if (!is_point_inside_image) return (int8_t)-128;
      return input_data[Offset(input_shape, 0, in_y, in_x, out_y)];
    };

    auto kernel = [&](const int out_y, const int out_x) {
      return filter_data[Offset(filter_shape, out_x, 0, 0, out_y)];
    };

    implicit_GEMM();
  }
}

inline void ConvPerChannelWithPackedInt4Weights(
    const ConvParams& params, const int32_t* output_multiplier,
    const int32_t* output_shift, const RuntimeShape& input_shape,
    const int8_t* input_data, const RuntimeShape& filter_shape,
    const int8_t* filter_input, int8_t* unpacked_filter_data,
    const RuntimeShape& bias_shape, const int32_t* bias_data,
    const RuntimeShape& output_shape, int8_t* output_data) {
  TFLITE_DCHECK(unpacked_filter_data != nullptr);
  tflite::tensor_utils::UnpackDenseInt4IntoInt8(
      filter_input, filter_shape.FlatSize(), unpacked_filter_data);
  ConvPerChannel(params, output_multiplier, output_shift, input_shape,
                 input_data, filter_shape, unpacked_filter_data, bias_shape,
                 bias_data, output_shape, output_data);
}

// Fixed-point per-channel-quantization convolution reference kernel.
// 16-bit data and 8-bit filter
template <typename AccumScalar>
inline void ConvPerChannel(
    const ConvParams& params, const int32_t* output_multiplier,
    const int32_t* output_shift, const RuntimeShape& input_shape,
    const int16_t* input_data, const RuntimeShape& filter_shape,
    const int8_t* filter_data, const RuntimeShape& bias_shape,
    const AccumScalar* bias_data, const RuntimeShape& output_shape,
    int16_t* output_data) {
  // Get parameters.
  const int stride_width = params.stride_width;
  const int stride_height = params.stride_height;
  const int dilation_width_factor = params.dilation_width_factor;
  const int dilation_height_factor = params.dilation_height_factor;
  const int pad_width = params.padding_values.width;
  const int pad_height = params.padding_values.height;

  // Set min and max value of the output.
  const int32_t output_activation_min = params.quantized_activation_min;
  const int32_t output_activation_max = params.quantized_activation_max;

  // Consistency check.
  TFLITE_DCHECK_LE(output_activation_min, output_activation_max);
  TFLITE_DCHECK_EQ(input_shape.DimensionsCount(), 4);
  TFLITE_DCHECK_EQ(filter_shape.DimensionsCount(), 4);
  TFLITE_DCHECK_EQ(output_shape.DimensionsCount(), 4);
  const int batches = MatchingDim(input_shape, 0, output_shape, 0);
  const int input_depth = input_shape.Dims(3);
  const int output_depth = MatchingDim(filter_shape, 0, output_shape, 3);
  if (bias_data) {
    TFLITE_DCHECK_EQ(bias_shape.FlatSize(), output_depth);
  }

  // Check dimensions of the tensors.
  const int input_height = input_shape.Dims(1);
  const int input_width = input_shape.Dims(2);
  const int filter_height = filter_shape.Dims(1);
  const int filter_width = filter_shape.Dims(2);
  const int filter_input_depth = filter_shape.Dims(3);
  const int groups = input_depth / filter_input_depth;
  TFLITE_DCHECK_EQ(input_depth % filter_input_depth, 0);
  const int filters_per_group = output_depth / groups;
  const int output_height = output_shape.Dims(1);
  const int output_width = output_shape.Dims(2);
  for (int batch = 0; batch < batches; ++batch) {
    for (int out_y = 0; out_y < output_height; ++out_y) {
      const int in_y_origin = (out_y * stride_height) - pad_height;
      for (int out_x = 0; out_x < output_width; ++out_x) {
        const int in_x_origin = (out_x * stride_width) - pad_width;
        for (int out_channel = 0; out_channel < output_depth; ++out_channel) {
          auto group = out_channel / filters_per_group;
          AccumScalar acc = 0;
          for (int filter_y = 0; filter_y < filter_height; ++filter_y) {
            const int in_y = in_y_origin + dilation_height_factor * filter_y;
            for (int filter_x = 0; filter_x < filter_width; ++filter_x) {
              const int in_x = in_x_origin + dilation_width_factor * filter_x;

              // Zero padding by omitting the areas outside the image.
              const bool is_point_inside_image =
                  (in_x >= 0) && (in_x < input_width) && (in_y >= 0) &&
                  (in_y < input_height);

              if (!is_point_inside_image) {
                continue;
              }

              for (int in_channel = 0; in_channel < filter_input_depth;
                   ++in_channel) {
                int32_t input_val =
                    input_data[Offset(input_shape, batch, in_y, in_x,
                                      in_channel + group * filter_input_depth)];
                int32_t filter_val = filter_data[Offset(
                    filter_shape, out_channel, filter_y, filter_x, in_channel)];
                // Accumulate with 64 bits accumulator.
                // int64_t += int8_t * int16_t so the highest value we can
                // get from each accumulation is [-127, 127] * ([-32768,
                // 32767] -
                // [-32768, 32767]), which is [-8322945, 8322945].
                // log2(8322945) = 22.99.
                acc += filter_val * input_val;
              }
            }
          }
          if (bias_data) {
            acc += bias_data[out_channel];
          }
          int32_t scaled_acc = MultiplyByQuantizedMultiplier(
              acc, output_multiplier[out_channel], output_shift[out_channel]);
          scaled_acc = std::max(scaled_acc, output_activation_min);
          scaled_acc = std::min(scaled_acc, output_activation_max);
          output_data[Offset(output_shape, batch, out_y, out_x, out_channel)] =
              static_cast<int16_t>(scaled_acc);
        }
      }
    }
  }
}

}  // namespace reference_integer_ops
}  // namespace tflite

#endif  // TENSORFLOW_LITE_KERNELS_INTERNAL_REFERENCE_INTEGER_OPS_CONV_H_
