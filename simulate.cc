#include "io.h"
#include "collision.h"

#include <vector>
#include <ranges>

int bin_num;
double bin_height;
std::vector<std::vector<int>> bins;

int f(const int x, const int y) { return x * bin_num + y; }

void push_to_bin(const Vec2 loc, const int i) {
    const int x = static_cast<int>(loc.x / bin_height);
    const int y = static_cast<int>(loc.y / bin_height);
    bins[f(x, y)].push_back(i);
}

void init(const Params& params) {
    bin_height = 2 * params.param_radius;
    bin_num = static_cast<int>(params.square_size / bin_height) + 1;
    bins.resize(bin_num * bin_num);
}

void simulate_step(std::vector<Particle>& particles, const int square_size, const int radius) {
    using std::views::iota;

#pragma omp parallel for
    for (auto& [i, loc, vel] : particles) {
        loc.x += vel.x;
        loc.y += vel.y;
    }

    bool has_updates;
    do {
        has_updates = false;

        for (const auto& [i, loc, _] : particles) {
            push_to_bin(loc, i);
        }

#pragma omp parallel for collapse(2) reduction(|:has_updates)
        for (const int x : iota(0, bin_num)) {
            for (const int y : iota(0, bin_num)) {
                for (const int nx : iota(x - 1, x + 2)) {
                    if (nx < 0 || bin_num <= nx) continue;
                    for (const int ny : iota(y - 1, y + 2)) {
                        if (ny < 0 || bin_num <= ny) continue;

                        auto& bin = bins[f(x, y)];
                        auto& other = bins[f(nx, ny)];

                        for (const auto bi : bin) {
                            auto& [i, loc1, vel1] = particles[bi];
                            for (const auto bj : other) {
                                auto& [j, loc2, vel2] = particles[bj];

                                if (i < j && is_particle_collision(loc1, vel1, loc2, vel2, radius)) {
#pragma omp critical
                                    resolve_particle_collision(loc1, vel1, loc2, vel2);
                                    has_updates = true;
                                }
                            }
                        }
                    }
                }
            }
        }

#pragma omp parallel for reduction(|:has_updates)
        for (auto& [i, loc, vel] : particles) {
            if (is_wall_collision(loc, vel, square_size, radius)) {
                resolve_wall_collision(loc, vel, square_size, radius);
                has_updates = true;
            }
        }

#pragma omp parallel for
        for (auto& bin : bins) bin.clear();
    } while (has_updates);
}
