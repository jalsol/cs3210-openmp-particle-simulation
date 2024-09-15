#include <vector>

#include "io.h"
#include "collision.h"

void simulate_step(const Params& params, std::vector<Particle>& particles) {
    for (auto& [_, loc, vel] : particles) {
        loc.x += vel.x;
        loc.y += vel.y;
    }

    bool has_updates;
    do {
        has_updates = false;

        for (auto& [i, loc, vel] : particles) {
            if (is_wall_collision(loc, vel, params.square_size, params.param_radius)) {
                resolve_wall_collision(loc, vel, params.square_size, params.param_radius);
                has_updates = true;
            }
        }

        for (auto& [i, loc1, vel1] : particles) {
            for (auto& [j, loc2, vel2] : particles) {
                if (i == j) continue;

                if (is_particle_collision(loc1, vel1, loc2, vel2, params.param_radius)) {
                    resolve_particle_collision(loc1, vel1, loc2, vel2);
                    has_updates = true;
                }
            }
        }
    } while (has_updates);
}
