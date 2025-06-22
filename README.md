# Impulsive Orbit Transfer Optimization using Particle Swarm Optimization

## Overview

This repository contains a comprehensive implementation of the Particle Swarm Optimization (PSO) algorithm applied to orbital transfer optimization problems. The project focuses on determining globally optimal impulsive orbital transfers between various types of orbits while minimizing the total velocity change (ΔV). The implementation includes both computational optimization and real-time 3D visualization capabilities.

## Project Description

Particle Swarm Optimization is a population-based stochastic method that mimics the collective behavior of bird flocks or fish schools searching for food. This intuitive yet powerful optimization technique does not require continuity or differentiability of the objective function, needs no initial guess, and is well-suited for finding global optima in complex search spaces.

This implementation applies PSO to impulsive orbital transfer problems, which involve determining the optimal maneuvers to transfer a spacecraft between different orbits while minimizing the total velocity change (ΔV). The project combines advanced orbital mechanics with modern visualization techniques to provide both computational results and intuitive understanding of the transfer trajectories.

## Orbital Transfer Problem Types

This project addresses two main types of orbital transfer problems:

1. **Coplanar Circular Orbits**: Determination of globally optimal two- and three-impulse transfers between two coplanar circular orbits using Hohmann and bi-elliptic transfer strategies
2. **Non-coplanar Circular Orbits**: Determination of optimal two-impulse transfers between circular orbits with different inclinations, incorporating plane change maneuvers

## Key Features

### Core Optimization

- **Advanced PSO Implementation**: Complete implementation with adaptive inertia weights, cognitive and social components
- **Constraint Handling**: Sophisticated constraint system handling trajectory feasibility, intersection requirements, and physical limits
- **Lambert Solver**: High-precision Lambert problem solver for connecting two position vectors in specified time
- **Adaptive Parameters**: Dynamic adjustment of PSO parameters throughout optimization for improved convergence

### Orbital Mechanics

- **Comprehensive Orbit Models**: Support for circular, elliptical, coplanar, and non-coplanar orbits
- **3D Orbital Mechanics**: Full 3D position and velocity calculations with proper coordinate transformations
- **Impulse Calculations**: Accurate modeling of impulsive maneuvers with magnitude and direction optimization
- **Transfer Validation**: Built-in validation of transfer trajectories and energy conservation

### Visualization System

- **Real-time 3D Rendering**: OpenGL-based visualization with interactive camera controls
- **Orbit Display**: Visual representation of initial, target, and transfer orbits
- **Animation System**: Time-based animation of transfer trajectories with play/pause/speed controls
- **Complete Ellipse Rendering**: Option to display full transfer ellipses vs. trajectory segments

### Performance & Analysis

- **Convergence Logging**: Detailed logging of optimization progress and parameter evolution
- **Results Export**: Comprehensive results output including orbital elements and transfer parameters
- **Performance Profiling**: Built-in profiling capabilities for algorithm optimization

## Code Architecture

```
project/
├── src/
│   ├── core/                     # Core physics and mathematics
│   │   ├── OrbitMechanics.cpp    # Orbital calculations and transformations
│   │   ├── LambertSolver.cpp     # Lambert problem solver
│   │   ├── OrbitProblem.cpp      # Transfer problem formulation
│   │   └── CoordinateSystem.hpp  # Coordinate transformations
│   ├── optimization/             # PSO implementation
│   │   ├── PSO.cpp              # Main PSO algorithm
│   │   ├── Swarm.cpp            # Particle swarm management
│   │   ├── Particle.hpp         # Individual particle implementation
│   │   └── Logger.cpp           # Convergence logging
│   ├── visualization/            # OpenGL rendering system
│   │   ├── OrbitModel.cpp       # Orbit visualization
│   │   ├── TransferModel.cpp    # Transfer trajectory rendering
│   │   ├── Camera.cpp           # Interactive camera system
│   │   ├── Animation.cpp        # Animation controls
│   │   └── Shader.cpp           # OpenGL shader management
│   └── application/              # Main applications
│       ├── pso_main.cpp         # Optimization-only executable
│       └── render_main.cpp      # Visualization executable
├── include/                      # Header files
├── shaders/                      # OpenGL shaders
├── resources/                    # Results and data files
└── tests/                        # Unit tests
```

## Installation

### Prerequisites

- **C++ Compiler**: C++20 support required
- **OpenGL**: For 3D visualization
- **GLFW**: Window management and input handling
- **GLM**: OpenGL Mathematics library
- **CMake**: Version 3.24 or higher
- **Optional**: OpenMP for parallel processing, Catch2 for testing

### macOS Installation (Homebrew)

```bash
# Install dependencies
brew install cmake glfw glm llvm

# Clone and build
git clone [repository-url]
cd PSO-Orbital-Transfer
mkdir build && cd build
cmake ..
make
```

### Linux Installation

```bash
# Ubuntu/Debian
sudo apt-get install cmake libglfw3-dev libglm-dev libgl1-mesa-dev

# Build project
mkdir build && cd build
cmake ..
make
```

## Usage

### Computation-Only Mode

For pure optimization without visualization:

```bash
./pso_compute
```

This runs the PSO algorithm and outputs results to `../resources/results.txt`.

### Visualization Mode

For interactive 3D visualization:

```bash
./pso_visualize [results_file]
```

**Controls:**

- **Left mouse + drag**: Rotate camera
- **Scroll wheel**: Zoom in/out
- **Space**: Play/pause animation
- **R**: Reset animation
- **+/-**: Adjust animation speed
- **L**: Toggle animation looping
- **T**: Toggle complete transfer ellipse display
- **Esc**: Exit

### Configuration

Edit `include/core/Constants.hpp` to modify transfer scenarios:

```cpp
namespace constant {
    // Orbital radii (in Distance Units)
    inline constexpr double R1 = 6378.165/6378.165;  // Initial orbit
    inline constexpr double R2 = (1.5 * 6378.165)/6378.165;  // Target orbit

    // Orbital inclinations (radians)
    inline constexpr double I1 = 0.497419;  // ~28.5 degrees
    inline constexpr double I2 = 0.0;       // Equatorial

    // Eccentricities
    inline constexpr double E1 = 0.0;  // Circular
    inline constexpr double E2 = 0.0;  // Circular
}
```

### PSO Parameters

Modify PSO parameters in `src/application/pso_main.cpp`:

```cpp
size_t numParticles = 1000;        // Swarm size
size_t maxIterations = 20000;      // Maximum iterations
double tolerance = 1e-2;           // Convergence tolerance
double inertiaWeight = 0.5;        // Inertia component
double cognitiveWeight = 2.0;      // Personal best component
double socialWeight = 1.7;         // Global best component
```

## Results Interpretation

The optimization outputs comprehensive transfer details:

```
[InitialOrbit]
radius = 1.0
inclination = 0.497419
raan = 0.0
eccentricity = 0.0

[TargetOrbit]
radius = 1.5
inclination = 0.0
raan = 0.0
eccentricity = 0.0

[OptimalTransfer]
initial_true_anomaly = 2.34159
final_true_anomaly = 3.14159
transfer_time = 1.8542
is_three_impulse = false

[DeltaV]
magnitude = 0.2453,0.1876
plane_change = 0.3142,0.0
```

## Validation and Performance

### Test Cases

The implementation has been validated against:

- **Hohmann transfers**: Classic two-impulse coplanar transfers
- **Plane change maneuvers**: Non-coplanar transfer validation
- **Lambert problem solutions**: Independent verification of trajectory calculations

### Performance Characteristics

- **Convergence**: Typically 1000-5000 iterations for simple transfers
- **Accuracy**: Solutions accurate to within 0.1% of analytical results
- **Scalability**: Handles problems with 6+ optimization variables
- **Robustness**: Successful convergence rate >95% for well-posed problems

## Advanced Features

### Constraint Handling

The implementation includes intensive constraint management:

- **Trajectory feasibility**: Ensures transfers intersect initial and target orbits
- **Physical limits**: Prevents impossible maneuvers (escape velocity constraints)
- **Geometric constraints**: Maintains proper orbital geometry
- **Relaxation factors**: Adaptive constraint relaxation during optimization

### Coordinate Systems

Unified coordinate system handling throughout:

- **Physics coordinates**: Standard orbital mechanics reference frame
- **Visualization coordinates**: OpenGL-compatible display coordinates
- **Automatic transformations**: Seamless conversion between systems

## Testing

Run the test suite:

```bash
make run_tests
```

Tests cover:

- PSO algorithm convergence
- Orbital mechanics calculations
- Lambert solver accuracy
- Coordinate transformations

## Profiling

Enable performance profiling:

```bash
cmake -DPROF=ON ..
make profile
```

This generates detailed performance analysis in `resources/ProfAnalysis.txt`.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
