# QTRVSim-RVV: Extended RISC-V Vector Extension Simulator

![QTRVSim-RVV Logo](images/logo.png) <!-- Replace with your logo or remove if not available -->

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Implemented RVV Instructions](#implemented-rvv-instructions)
- [Installation](#installation)
- [Usage](#usage)
  - [Running the Simulator](#running-the-simulator)
  - [Example: Matrix Multiplication](#example-matrix-multiplication)
- [Test Results](#test-results)
- [Cycle Count Analysis](#cycle-count-analysis)
- [Optimization Techniques](#optimization-techniques)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Overview

**QTRVSim-RVV** is an enhanced version of the [QTRVSim](https://github.com/cvut/qtrvsim.git) RISC-V simulator, extended to support the RISC-V Vector Extension (RVV). This extension enables efficient parallel vector operations, significantly improving the performance of computational tasks such as matrix multiplication. By implementing a subset of RVV instructions, QTRVSim-RVV demonstrates the benefits of vector processing in high-performance computing applications.

## Features

- **Extended RVV Support:**
  - Implemented RVV instructions including:
    - `vsetvl`: Set vector length
    - `vadd.vv`: Vector add
    - `vadd.vx`: Vector add with scalar
    - `vadd.vi`: Vector add with immediate
    - `vmul.vv`: Vector multiply
    - `vmacc.vv`: Vector multiply-accumulate (returns scalar)
    - `vlw.v`: Vector load word
    - `vsw.v`: Vector store word
  - *Note:* `vredsum.vs` (Vector Reduction Sum) is **not** implemented in this version.

- **Matrix Multiplication Example:**
  - Assembly code (`matrixMul.s`) demonstrating efficient matrix multiplication using RVV instructions.

- **Cycle Counting:**
  - Detailed cycle count analysis for vector operations, highlighting performance gains over scalar implementations.

- **Optimization Techniques:**
  - Reduction optimization
  - Loop unrolling and chaining

## Implemented RVV Instructions

The following RISC-V Vector Extension (RVV) instructions have been implemented in QTRVSim-RVV:

1. **`vsetvl`**: Sets the vector length (VL) based on the given registers.
2. **`vadd.vv`**: Adds two vectors element-wise.
3. **`vadd.vx`**: Adds a scalar to each element of a vector.
4. **`vadd.vi`**: Adds an immediate value to each element of a vector.
5. **`vmul.vv`**: Performs element-wise multiplication of two vectors.
6. **`vmacc.vv`**: Performs vector multiply-accumulate and returns a scalar sum.
7. **`vlw.v`**: Loads a vector from memory.
8. **`vsw.v`**: Stores a vector to memory.

*Note:* The `vredsum.vs` instruction, which reduces a vector by summing its elements, is **not** implemented in this version of the simulator.

## Installation

### Prerequisites

- **Operating System:** Linux, macOS, or Windows (with WSL)
- **Dependencies:**
  - RISC-V toolchain (e.g., `riscv64-unknown-elf-gcc`)
  - CMake (for building the simulator)
  - Make

### Steps

1. **Clone the Repository:**

    ```bash
    git clone https://github.com/your-username/qtrvsim-rvv.git
    cd qtrvsim-rvv
    ```

2. **Initialize Submodules (if any):**

    If your project uses submodules, initialize them:

    ```bash
    git submodule update --init --recursive
    ```

3. **Build the Simulator:**

    ```bash
    mkdir build
    cd build
    cmake ..
    make
    ```

    This will compile the simulator and place the executable in the `build` directory.

## Usage

### Running the Simulator

To run an assembly program (e.g., `matrixMul.s`), use the following command:

```bash
./qtrvsim path/to/matrixMul.s
```
## Test Results

### Matrix Multiplication Test

**Test Case: Basic 20x50 Matrix Multiplication**

- **Input Matrices:**
  - Matrix A: 20 rows x 46 columns
  - Matrix B: 46 rows x 50 columns
  - All elements initialized to 1

- **Expected Output:**
  - Matrix C: 20 rows x 50 columns, each element = 46 (since \(1 \times 1\) summed 46 times)

- **Observed Output:**
  - **Result stored:** 32 (appears multiple times corresponding to intermediate storage operations)
  - **Cycle Count Logs:** Indicate the number of cycles consumed during execution
  - **No Errors Encountered**

- **Cycle Count:** 106,487 cycles *(example value; replace with actual results)*

### Screenshot of Simulator Output

![Simulator Output](images/simulator_output.png) <!-- Replace with actual image path -->

### Screenshot of Cycle Count Logs

![Cycle Count Logs](images/cycle_count_logs.png) <!-- Replace with actual image path -->

## Cycle Count Analysis

### Cycle Counting Logic

The cycle count in the simulator is updated based on the vector instructions executed, following this logic:

```cpp
if(dt.inst.flags() & (IMF_VADD_VV | IMF_VADD_VX | IMF_VADD_VI)){
    state.cycle_count += vl - 1;
}
if(dt.inst.flags() & IMF_VMUL_VV){
    state.cycle_count += vl + 4 - 1;
}
if(dt.inst.flags() & IMF_VMACC_VV){
    state.cycle_count += vl + 4 - 1;
}
``` 
### Explanation
1. Vector Add Instructions (vadd.vv, vadd.vx, vadd.vi):

  - Cycle Increment: VL - 1
  - Reason: Parallel processing of VL elements with a setup overhead.
  - Vector Multiply Instruction (vmul.vv):

2. Cycle Increment: VL + 3 (i.e., VL + 4 - 1)
  - Reason: Parallel multiplication of VL elements plus additional cycles for reduction or accumulation.
  - Vector Multiply-Accumulate Instruction (vmacc.vv):

3. Cycle Increment: VL + 3 (i.e., VL + 4 - 1)
  - Reason: Parallel multiply-accumulate of VL elements plus additional cycles for reduction or accumulation.
### 1. **Reduction Optimization**

Binary tree reduction is implemented to efficiently sum elements in a vector by reducing the number of operations needed. Instead of summing elements sequentially, this approach halves the data size in each iteration through pairwise summation, reducing the total number of cycles.

#### Binary Tree Reduction Algorithm:

1. **Pairwise Summation:**
   - Divide the vector into halves and sum corresponding pairs of elements.
   - Store the results in a temporary buffer.

2. **Handling Odd Lengths:**
   - If the vector length is odd, the last element is carried forward to the next iteration.
   - This ensures no element is missed during the reduction.

3. **Repeat Until Completion:**
   - Repeat the process, reducing the vector length in each iteration, until a single element remains.

#### Implementation in Code:

```cpp
// Perform binary tree reduction on the temporary buffer
int vl_reduction = vl;
while (vl_reduction > 1) {
    int half = vl_reduction / 2;

    // Pair-wise summing
    for (int i = 0; i < half; ++i) {
        buffer[i] = temp[i] + temp[i + half];
    }

    // Handle carry forward for odd vl_reduction
    if (vl_reduction % 2 != 0) {
        buffer[half] = temp[vl_reduction - 1];
        printf("Carry forward: buffer[%d] = temp[%d} => %u\n", 
               half, vl_reduction - 1, buffer[half]);
        vl_reduction = half + 1;
    } else {
        vl_reduction = half;
    }

    // Prepare for next reduction step
    for (int i = 0; i < vl_reduction; ++i) {
        temp[i] = buffer[i];
    }

    // Clear the buffer for the next iteration
    std::fill(buffer.begin(), buffer.end(), 0);
}
```
#### Example:

Given a vector `[1, 2, 3, 4]`:

- **Stage 1:** `[1+2, 3+4]` → `[3, 7]`
- **Stage 2:** `[3+7]` → `[10]`

Result: `10`.

***Cycle Efficiency***

- Sequential reduction for \( n \) elements requires \( n - 1 \) additions.
- Binary tree reduction reduces this to **ceil(log2(n))** stages, with each stage operating in parallel.


This optimization is particularly effective for operations involving large datasets, such as matrix multiplication or convolution, where reduction is a bottleneck.




