#include "io.h"
#include "collision.h"

#include <vector>
#include <ranges>

using std::views::iota;

int bin_num;
int num_threads;
double bin_height;
int radius;
int square_size;
int slice_count;
int slice_size;
int last_cell;
std::vector<std::vector<int>> bins;

int f(const int x, const int y) { return x * bin_num + y; }

void push_to_bin(const Vec2 loc, const int i) {
    const int x = static_cast<int>((loc.x + square_size / 2) / bin_height);
    const int y = static_cast<int>((loc.y + square_size / 2) / bin_height);
    bins[f(x, y)].push_back(i);
}

void init(const Params& params) {
    bin_height = 2 * params.param_radius;
    bin_num = static_cast<int>(2 * params.square_size / bin_height) + 1;
    bins.resize(bin_num * bin_num);
    num_threads = params.param_threads;

    radius = params.param_radius;
    square_size = params.square_size;
    slice_count = 2 * num_threads;
    slice_size = bin_num / slice_count;
    last_cell = 2 * num_threads * slice_size;
}

bool resolve_collision_threaded(std::vector<Particle>& particles, int start, int end);

void simulate_step(std::vector<Particle>& particles) {
    #pragma omp parallel for
    for (auto& [i, loc, vel] : particles) {
        loc.x += vel.x;
        loc.y += vel.y;
    }

    bool has_updates;
    do {
        has_updates = false;

        for (const auto& [i, loc, vel] : particles) {
            push_to_bin(loc, i);
        }

        // first pass
        #pragma omp parallel shared(has_updates)
        #pragma omp single
        {
            #pragma omp taskgroup task_reduction(|:has_updates)
            {
                for (const int i : iota(0, num_threads)) {
                    const int start = 2 * i * slice_size;
                    const int end = start + slice_size;

                    #pragma omp task in_reduction(|:has_updates)
                    has_updates |= resolve_collision_threaded(particles, start, end);
                }

                if (last_cell < square_size) {
                    #pragma omp task in_reduction(|:has_updates)
                    has_updates |= resolve_collision_threaded(particles, last_cell, bin_num);
                }
            }
        }
        #pragma omp barrier

        // second pass
        #pragma omp parallel shared(has_updates)
        #pragma omp single
        {
            #pragma omp taskgroup task_reduction(|:has_updates)
            {
                for (const int i : iota(0, num_threads)) {
                    const int start = (2 * i + 1) * slice_size;
                    const int end = start + slice_size;

                    #pragma omp task in_reduction(|:has_updates)
                    has_updates |= resolve_collision_threaded(particles, start, end);
                }
            }
        }
        #pragma omp barrier

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

bool resolve_collision_threaded(std::vector<Particle>& particles, const int start, const int end) {
    bool local_updates = false;

    for (const int x : iota(start, end)) {
        for (const int y : iota(0, bin_num)) {
            for (const int nx : iota(x - 1, x + 2)) {
                if (nx < 0 || bin_num <= nx) continue;
                for (const int ny : iota(y - 1, y + 2)) {
                    if (ny < 0 || bin_num <= ny) continue;

                    auto& bin = bins[f(x, y)];
                    auto& other = bins[f(nx, ny)];

                    for (const auto bi : bin) {
                        for (const auto bj : other) {
                            auto& [i, loc1, vel1] = particles[bi];
                            auto& [j, loc2, vel2] = particles[bj];

                            if (i < j && is_particle_collision(loc1, vel1, loc2, vel2, radius)) {
                                resolve_particle_collision(loc1, vel1, loc2, vel2);
                                local_updates = true;
                            }
                        }
                    }
                }
            }
        }
    }

    return local_updates;
}
