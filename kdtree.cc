#include "kdtree.h"

#include "collision.h"

#include <ranges>
#include <algorithm>
#include <numeric>

KdNode* build(
    const std::vector<Particle>& particles,
    std::vector<int>& indices,
    const int left_range,
    const int right_range,
    const int depth
) {
    if (indices.empty() || left_range >= right_range) {
        return nullptr;
    }
    if (indices.size() == 1) {
        return new KdNode(indices[0]);
    }

    const auto mid = std::midpoint(left_range, right_range);
    std::span span(indices.begin() + left_range, indices.begin() + right_range);

    std::ranges::sort(span, [&](const int i, const int j) {
        if (depth % MOD == HORIZONTAL) {
            return particles[i].loc.x < particles[j].loc.x;
        } else {
            return particles[i].loc.y < particles[j].loc.y;
        }
    });

    const auto ret = new KdNode(
        indices[mid],
        build(particles, indices, left_range, mid, depth + 1),
        build(particles, indices, mid + 1, right_range, depth + 1)
    );

    return ret;
}

std::vector<int> search(
    const Vec2 loc,
    const KdNode* node,
    const std::vector<Particle>& particles,
    const int scan_length,
    const int depth
) {
    if (node == nullptr) return {};

    const Vec2 split_loc = particles[node->split].loc;
    std::vector<int> ret;

    if (sq_dist(loc, split_loc) <= scan_length * scan_length) {
        ret.push_back(node->split);
    }

    if (depth % MOD == HORIZONTAL) {
        if (loc.x - scan_length < split_loc.x) {
            const std::vector<int> subret = search(loc, node->left, particles, scan_length, depth + 1);
            std::ranges::copy(subret, std::back_inserter(ret));
        }
        if (loc.x + scan_length > split_loc.x) {
            const std::vector<int> subret = search(loc, node->right, particles, scan_length, depth + 1);
            std::ranges::copy(subret, std::back_inserter(ret));
        }
    } else {
        if (loc.y - scan_length < split_loc.y) {
            const std::vector<int> subret = search(loc, node->left, particles, scan_length, depth + 1);
            std::ranges::copy(subret, std::back_inserter(ret));
        }
        if (loc.y + scan_length > split_loc.y) {
            const std::vector<int> subret = search(loc, node->right, particles, scan_length, depth + 1);
            std::ranges::copy(subret, std::back_inserter(ret));
        }
    }

    return ret;
}
