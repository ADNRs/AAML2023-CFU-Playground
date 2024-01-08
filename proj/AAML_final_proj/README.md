# Accelerating Convolutional Neural Network with Systolic Array and Manual Optimizations

## Team Member
+ 陳忠義 311551070
+ 林霆寬 311551079

## Note

Although it is very unlikely to happen, if the golden test fails, please `make prog` again. It is certainly due to poor placement and routing.

## Hardware Feature
+ A matrix multiplication unit
  + Backed by a 16×16 output stationary systolic array
  + Supporting up to m256n256k256 matrix multiplication
+ Hardware implementation of `gemmlowp::RoundingDivideByPOT` and `gemmlowp::SaturatingRoundingDoublingHighMul`
+ Hardware implementation of the clamping operation in the quantization post-processing

## Software Optimization
+ Specialize `ConvPerChannel`, `Add`, and `FullyConnected` according to the model hyperparameter setting
+ Enhance the computation for the out-of-range check while accessing input data
+ Simplify the logic of `MultiplyByQuantizedMultiplier*` functions
+ Apply implicit im2col to save the time for constructing im2col and kernel matrices
+ Unroll the `FullyConnected` inner loop

## Result
+ Original: 4044074.545 μs
+ Pure software optimization: 1428413.2 μs (2.831x speedup)
+ With hardware acceleration: 406357.93 μs (9.952x speedup)

## Supplementary Article
+ Available here: [link](https://yeecy.medium.com/在-cfu-playground-上加速-mlperf-tiny-影像分類模型-0-前言-a65b0c3a6a6b)
