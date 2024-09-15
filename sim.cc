#include <omp.h>

#include <cmath>
#include <vector>

#include "io.h"
#include "sim_validator.h"

void simulate_step(const Params& params, std::vector<Particle>& particles);

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

    for (auto _ = 0; _ < params.param_steps; ++_) {
        simulate_step(params, particles);
#if CHECK == 1
        validator.validate_step(particles);
#endif
    }
}
