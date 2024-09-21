#include "io.h"
#include "collision.h"
#include "vector.h"

#include <ranges>
#include <algorithm>

namespace {

constexpr int BIN_SIZE = 16;
Vector bins[BIN_SIZE][BIN_SIZE];
Vector temps[BIN_SIZE][BIN_SIZE];
int bin_length;

} // namespace

bool simulate_substep(std::vector<Particle>& particles, int square_size, int radius);

void init(int square_size, const std::vector<Particle>& particles) {
    using std::views::iota;
    bin_length = square_size / BIN_SIZE + 1;

    #pragma omp parallel for
    for (const auto& [i, loc, vel] : particles) {
        const auto bin_x = static_cast<int>(loc.x / bin_length);
        const auto bin_y = static_cast<int>(loc.y / bin_length);

        #pragma omp critical
        bins[bin_x][bin_y].push_back(i);
    }
}

void simulate_step(std::vector<Particle>& particles, int square_size, int radius) {
    using std::views::iota;

    #pragma omp parallel for
    for (auto& [i, loc, vel] : particles) {
        loc.x += vel.x;
        loc.y += vel.y;
    }

    bool has_updates;
    do {
        has_updates = simulate_substep(particles, square_size, radius);

        #pragma omp for
        for (const auto bin_x : iota(0, BIN_SIZE)) {
            for (const auto bin_y : iota(0, BIN_SIZE)) {
                auto& bin = bins[bin_x][bin_y];
                auto& temp = temps[bin_x][bin_y];

                auto&& predicate = [&](const auto i) {
                    const auto& [_, loc, vel] = particles[i];
                    const auto next_bin_x = static_cast<int>(loc.x / bin_length);
                    const auto next_bin_y = static_cast<int>(loc.y / bin_length);
                    return bin_x == next_bin_x && bin_y == next_bin_y;
                };

                const auto it = std::partition(bin.begin(), bin.end(), predicate);
                for (auto p = it; p != bin.end(); ++p) {
                    temp.push_back(*p);
                }
                bin.size = std::distance(bin.begin(), it);
            }
        }

        #pragma omp for
        for (const auto bin_x : iota(0, BIN_SIZE)) {
            for (const auto bin_y : iota(0, BIN_SIZE)) {
                auto& temp = temps[bin_x][bin_y];
                for (const auto i : temp) {
                    auto& [_, loc, vel] = particles[i];
                    const auto next_bin_x = static_cast<int>(loc.x / bin_length);
                    const auto next_bin_y = static_cast<int>(loc.y / bin_length);
                    bins[next_bin_x][next_bin_y].push_back(i);
                }
                temp.size = 0;
            }
        }
    } while (has_updates);
}

bool simulate_substep(std::vector<Particle>& particles, int square_size, int radius) {
    using std::views::iota;
    bool has_updates = false;

    #pragma omp parallel for
    for (const auto bin_x : iota(0, BIN_SIZE)) {
        for (const auto bin_y : iota(0, BIN_SIZE)) {
            auto& bin = bins[bin_x][bin_y];

            for (const auto dx : iota(-1, 2)) {
                if (bin_x + dx < 0 || BIN_SIZE <= bin_x + dx) continue;

                for (const auto dy : iota(-1, 2)) {
                    if (bin_y + dy < 0 || BIN_SIZE <= bin_y + dy) continue;

                    auto& other = bins[bin_x + dx][bin_y + dy];

                    for (const auto bi : bin) {
                        auto& [i, loc1, vel1] = particles[bi];
                        for (const auto bj : other) {
                            auto& [j, loc2, vel2] = particles[bj];

                            if (is_particle_collision(loc1, vel1, loc2, vel2, radius)) {
                                resolve_particle_collision(loc1, vel1, loc2, vel2);

                                #pragma omp atomic write
                                has_updates = true;
                            }
                        }
                    }
                }
            }
        }
    }

    #pragma omp parallel for
    for (auto& [i, loc, vel] : particles) {
        if (is_wall_collision(loc, vel, square_size, radius)) {
            resolve_wall_collision(loc, vel, square_size, radius);

            #pragma omp atomic write
            has_updates = true;
        }
    }

    return has_updates;
}
