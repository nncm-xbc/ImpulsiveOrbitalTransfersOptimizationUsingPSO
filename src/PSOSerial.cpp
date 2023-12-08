#include <iostream>
#include <limits>
#include <random>
#include <fstream>
#include "PSO.hpp"

void PSO_serial::pso(double (*ObjFuncPtr)(double*, int),
            const int dimensions,
            const std::pair<double,double>* &bounds,
            vector<Particle> &swarm,
            int num_particles,
            int max_iter,
            const double inertiaWeight,
            const double c1,
            const double c2) 
    {
    // Main loop
    for (int iter = 0; iter < max_iter; ++iter) {
        int particle_count = 0;

        for (auto& particle : swarm) {
            // Update velocity & position
            for (int i = 0; i < dimensions; ++i) {

                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> dis(0.0, 1.0);
                std::vector<double> r1 = {dis(gen), dis(gen)};
                std::vector<double> r2 = {dis(gen), dis(gen)};

                // v(t+1) = inertiaWeight * v(t) + c1 * r1 * (local_best - x(t)) + c2 * r2 * (global_best - x(t))
                particle.velocity[i] = inertiaWeight * particle.velocity[i] 
                                        + c1 * r1[i] * (particle.best_position[i] - particle.position[i])
                                        + c2 * r2[i] * (global_best_position[i] - particle.position[i]);
                particle.position[i] += particle.velocity[i];
            }
            particle.value = ObjFuncPtr(particle.position, dimensions);

            // Update local best
            if (particle.value < particle.best_value) {
                particle.best_value = particle.value;
                particle.best_position = particle.position;
            }

            // Update global best position and solution
            if (particle.best_value < global_best_sol) {
                global_best_position = particle.position;
                global_best_sol = particle.value;
            }
            particle_count++;
        }
        global_best_sol_history[iter] = global_best_sol;
        global_best_positions_history[iter] = global_best_position;
    }
}