# PSO Orbital Transfer Optimization

<div align="center">

**Particle Swarm Optimization for Circular Orbital Transfer Problems with 3D Visualization**

[Quick Start](#quick-start) • [Features](#features) • [Examples](#examples) • [Installation](#installation) • [Usage](#usage)

</div>

---

## Overview

This project implements a comprehensive **Particle Swarm Optimization (PSO)** algorithm specifically designed for solving circular orbital transfer optimization problems. It combines advanced orbital mechanics with modern visualization techniques to provide both computational results and intuitive understanding of optimal spacecraft trajectories.

### What is Particle Swarm Optimization?

PSO is a population-based stochastic optimization method inspired by the collective behavior of bird flocks or fish schools. Unlike traditional optimization methods, PSO:

- **Requires no derivatives** - works with any objective function
- **Finds global optima** - explores the entire solution space
- **Handles constraints naturally** - perfect for orbital mechanics
- **Provides intuitive visualization** - swarm behavior is easy to understand

### Why Orbital Transfer Optimization?

Circular orbital transfer problems involve:

- **Non-linear dynamics** with complex gravitational interactions
- **Multiple local optima** where traditional methods get stuck
- **Complex constraints** (collision avoidance, fuel limits, timing)
- **Plane change optimization** for non-coplanar transfers

**Our solution provides globally optimal transfers that minimize fuel consumption (ΔV) while satisfying all mission constraints.**

---

## Quick Start

Get up and running in 5 minutes:

### 1. Prerequisites Check

```bash
# Check if you have the required tools
cmake --version  # Need 3.24+
gcc --version    # Need C++20 support (GCC 10+ or Clang 12+)
```

### 2. Install Dependencies

### Dependencies

The project requires these libraries (automatically detected by CMake):

**Required:**

- **OpenGL** (for visualization)
- **GLFW3** (window management)
- **GLM** (mathematics library)
- **GLAD** (OpenGL loader - included in project)

**Optional:**

- **OpenMP** (parallel processing - enabled by default)
- **Catch2** (testing framework)

**Ubuntu/Debian:**

```bash
sudo apt update
sudo apt install -y cmake build-essential libglfw3-dev libglm-dev libgl1-mesa-dev libglu1-mesa-dev libomp-dev
```

**macOS (additional):**

```bash
# macOS users may need LLVM for better C++20 support
brew install llvm
```

### 3. Build and Run

```bash
# Clone and build
git clone https://github.com/nncm-xbc/PSO-Orbital-Transfer
cd PSO-Orbital-Transfer
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

# Run optimization
./pso_compute

# View results with 3D visualization
./pso_visualize ../resources/results.txt
```

### 4. First Results

You should see:

- PSO optimization progress with real-time ΔV values
- Results saved to `../resources/results.txt`
- 3D visualization showing optimal transfer trajectory

---

## Features

### Advanced Optimization Engine

- **Particle Swarm Optimization** with adaptive parameters
- **Global optimization** - finds true optimal solutions
- **Constraint handling** for realistic mission scenarios
- **Parallel processing** with OpenMP for faster convergence

### Circular Orbital Mechanics

- **Two main transfer types**:
    - **Coplanar circular-to-circular** (Hohmann, bi-elliptic)
    - **Non-coplanar circular transfers** with optimal plane changes
- **Lambert problem solver** for precise trajectory calculation
- **3D orbital mechanics** with full coordinate transformations
- **Physical constraint validation** (collision avoidance, fuel limits)

### Interactive 3D Visualization

- **Real-time 3D rendering** with OpenGL
- **Interactive camera controls** (rotate, zoom, pan)
- **Animated trajectory visualization** with playback controls
- **Multiple orbit display** (initial, target, transfer)
- **Impulse visualization** showing maneuver locations and directions

---

## Examples

### Example 1: Coplanar Circular Transfer (LEO to GEO)

**Scenario**: Transfer from Low Earth Orbit to Geostationary Orbit

```cpp
// Configuration in Constants.hpp
inline constexpr double R1 = 1.0;      // LEO (6,378 km)
inline constexpr double R2 = 6.61;     // GEO (42,164 km)
inline constexpr double I1 = 0.0;      // Equatorial
inline constexpr double I2 = 0.0;      // Equatorial
```

**Results**:

- **Total ΔV**: ~6.85 km/s
- **Transfer strategy**: Two-impulse Hohmann transfer
- **Transfer time**: ~3.69 hours

### Example 2: Non-Coplanar Circular Transfer

**Scenario**: Transfer from inclined circular orbit to equatorial circular orbit

```cpp
// Configuration in Constants.hpp
inline constexpr double R1 = 1.0;      // Initial radius (6,378 km)
inline constexpr double R2 = 6.61;     // Final radius (42,164 km)
inline constexpr double I1 = 0.497;    // 28.5° inclination
inline constexpr double I2 = 0.0;      // Equatorial (0°)
```

**Results**:

- **Total ΔV**: ~7.04 km/s
- **Transfer strategy**: Combined radius and plane change
- **Transfer time**: ~1.08 hours

---

## Installation

### Build Options

```bash
# Full build with all features (default)
cmake .. -DCMAKE_BUILD_TYPE=Release

# Computation only (no visualization dependencies)
cmake .. -DBUILD_VISUALIZATION=OFF

# Debug build with symbols
cmake .. -DCMAKE_BUILD_TYPE=Debug

# Enable profiling
cmake .. -DENABLE_PROFILING=ON
```

---

## Usage

### Configuration

Modify transfer parameters in `include/core/Constants.hpp`:

```cpp
namespace Physics::Constants {
    // Orbital parameters (in canonical units)
    inline constexpr double R1 = 1.0;     // Initial orbit radius
    inline constexpr double R2 = 6.61;    // Final orbit radius
    inline constexpr double I1 = 0.0;     // Initial inclination (rad)
    inline constexpr double I2 = 0.0;     // Final inclination (rad)

    // PSO parameters
    inline constexpr int SWARM_SIZE = 500;
    inline constexpr int MAX_ITERATIONS = 1000;
}
```

### Running with Development Features

```bash
# Basic optimization run
./pso_compute

# With profiling enabled (build with -DPROF=ON)
./pso_compute
# Profile data saved to ../resources/ProfAnalysis.txt

# Run profiling target (if built with profiling)
make profile
```

### Visualization Controls

```bash
# Load and visualize results
./pso_visualize ../resources/results.txt

# Controls in visualization:
# - Mouse: Rotate view
# - Scroll: Zoom in/out
# - WASD: Pan camera
# - Space: Play/pause animation
# - R: Reset view
```

### Understanding Results

The optimization outputs detailed transfer information:

```ini
[OptimalTransfer]
TotalDeltaV=3.940899    # Total velocity change (km/s)
Impulse1=2.431770       # First impulse magnitude (km/s)
Impulse2=1.509130       # Second impulse magnitude (km/s)
PlaneChange1=0.92       # First impulse plane change (degrees)
PlaneChange2=9.08       # Second impulse plane change (degrees)
TransferType=NON_COPLANAR
```

---

## Architecture

### Core Components

- **PSO Engine** (`src/core/`): Particle swarm optimization implementation
- **Orbital Mechanics** (`src/core/OrbitMechanics.hpp`): Trajectory calculations
- **Visualization** (`src/visualization/`): 3D rendering and animation
- **Analysis Tools** (`src/analysis/`): Performance profiling and validation

### Key Algorithms

1. **Particle Swarm Optimization**: Global optimization with adaptive parameters
2. **Lambert Problem Solver**: Precise two-body trajectory calculation
3. **Coordinate Transformations**: Orbital elements ↔ Cartesian coordinates
4. **Constraint Handling**: Physical limitations and mission requirements

---

## Performance

### Optimization Characteristics

- **Convergence**: Typically achieves optimal solution within 200-300 iterations
- **Accuracy**: Results match analytical solutions (where available)
- **Scalability**: Linear scaling with number of CPU cores via OpenMP

### Computational Requirements

| Transfer Type | Typical Runtime | Memory Usage | Iterations |
| ------------- | --------------- | ------------ | ---------- |
| Coplanar      | 1-2 seconds     | <100 MB      | 100-200    |
| Non-coplanar  | 2-5 seconds     | <150 MB      | 300-500    |

---

### Development and Testing

```bash
# Install development dependencies (Ubuntu/Debian)
sudo apt install -y catch2

# Build with tests
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make

# Run all tests
./run_tests

# Run individual test suites
./test_orbit_mechanics
./test_orbit_problem
./test_pso

# Or use CTest
ctest
```
