#include "io.h"
#include "collision.h"

#include <ranges>

namespace {

constexpr int BIN_SIZE = 4;
std::vector<int> bins[BIN_SIZE][BIN_SIZE];
int bin_length;

} // namespace

bool simulate_substep(std::vector<Particle>& particles, int square_size, int radius);

void init(const int square_size) {
    bin_length = square_size / BIN_SIZE + (square_size % BIN_SIZE != 0);
}

void simulate_step(std::vector<Particle>& particles, int square_size, int radius) {
    using std::views::iota;

    for (auto& [i, loc, vel] : particles) {
        loc.x += vel.x;
        loc.y += vel.y;
    }

    bool has_updates;
    do {
        // TODO: parallelize this somehow?
        for (const auto& [i, loc, vel] : particles) {
            const auto bin_x = static_cast<int>(loc.x / bin_length);
            const auto bin_y = static_cast<int>(loc.y / bin_length);
            bins[bin_x][bin_y].push_back(i);
        }

        has_updates = simulate_substep(particles, square_size, radius);

        // TODO: parallelize this somehow?
        for (const auto bin_x : iota(0, BIN_SIZE)) {
            for (const auto bin_y : iota(0, BIN_SIZE)) {
                bins[bin_x][bin_y].clear();
            }
        }
    } while (has_updates);
}

bool simulate_substep(std::vector<Particle>& particles, int square_size, int radius) {
    using std::views::iota;
    bool has_updates = false;

    for (auto& [i, loc, vel] : particles) {
        if (is_wall_collision(loc, vel, square_size, radius)) {
            resolve_wall_collision(loc, vel, square_size, radius);
            has_updates = true;
        }
    }

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
                                has_updates = true;
                            }
                        }
                    }
                }
            }
        }
    }

    return has_updates;
}
