#include <omp.h>

#include <cmath>
#include <iostream>
#include <vector>

#include "io.h"
#include "sim_validator.h"

void init(const Params& params);

void simulate_step(std::vector<Particle>& particles, int square_size, int radius);

int main(const int argc, char* argv[]) {
    Params params{};
    std::vector<Particle> particles;
    read_args(argc, argv, params, particles);

    omp_set_num_threads(params.param_threads);

#if CHECK == 1
    SimulationValidator validator(params.param_particles, params.square_size, params.param_radius);
    validator.initialize(particles);
    // validator.enable_viz_output("test.out");
#endif

    init(params);
    for (auto step = 0; step < params.param_steps; ++step) {
        simulate_step(particles, params.square_size, params.param_radius);
#if CHECK == 1
        validator.validate_step(particles);
#endif
    }
}
