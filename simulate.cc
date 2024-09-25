#include "io.h"
#include "collision.h"
#include "kdtree.h"

#include <vector>
#include <ranges>
#include <numeric>

std::vector<int> indices;

void init(const Params& params) {
    indices.resize(params.param_particles);
    std::iota(indices.begin(), indices.end(), 0);
}

void simulate_step(std::vector<Particle>& particles, const int square_size, const int radius) {
    using std::views::iota;

    // move particles
    for (auto& [i, loc, vel] : particles) {
        loc.x += vel.x;
        loc.y += vel.y;
    }

    bool has_updates;

    do {
        has_updates = false;
        const KdNode* root = build(particles, indices);

        std::vector<std::vector<int>> neighbors;

        for (auto& [i, loc, vel] : particles) {
            neighbors.push_back(search(loc, root, particles, 2 * radius));
        }

        for (int i = 0; i < particles.size(); ++i) {
            auto& [_, loc1, vel1] = particles[i];
            for (const auto bj : neighbors[i]) {
                auto& [j, loc2, vel2] = particles[bj];

                if (is_particle_collision(loc1, vel1, loc2, vel2, radius)) {
                    resolve_particle_collision(loc1, vel1, loc2, vel2);
                    has_updates = true;
                }
            }
        }

        for (auto& [i, loc, vel] : particles) {
            if (is_wall_collision(loc, vel, square_size, radius)) {
                resolve_wall_collision(loc, vel, square_size, radius);
                has_updates = true;
            }
        }

        delete root;
    } while (has_updates);
}
