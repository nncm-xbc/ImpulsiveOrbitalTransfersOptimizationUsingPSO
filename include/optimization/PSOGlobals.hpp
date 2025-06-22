/**
 * @file PSOGlobals.hpp
 * @brief Global variables for PSO algorithm state tracking
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Provides global access to PSO iteration state for components that need
 * to implement iteration-dependent behavior, such as adaptive constraints.
 */

#ifndef PSOGLOBALS_HPP
#define PSOGLOBALS_HPP

/**
 * @namespace PSOGlobals
 * @brief Global variables for PSO algorithm coordination
 *
 * Contains global state variables that allow different components of the
 * PSO system to coordinate behavior based on optimization progress.
 *
 * These globals are primarily used for:
 * - Adaptive constraint relaxation in orbital transfer problems
 * - Progress reporting and monitoring
 * - Debugging and analysis
 *
 * @warning Use of global variables should be minimized. These are included
 * for legacy compatibility and specific use cases where passing iteration
 * state through function calls would be impractical.
 */
namespace PSOGlobals {
    /**
     * @brief Current iteration number of the PSO algorithm
     *
     * Updated by the main PSO loop at the beginning of each iteration.
     * Used by adaptive constraint systems to implement iteration-dependent
     * constraint relaxation strategies.
     */
    inline int currentIteration = 0;

    /**
     * @brief Maximum number of iterations for the PSO run
     *
     * Set at the beginning of optimization and used to calculate
     * relative progress (currentIteration / maxIterations) for
     * adaptive parameter and constraint strategies.
     */
    inline int maxIterations = 1;
}

#endif
