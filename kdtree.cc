#include "kdtree.h"

#include "collision.h"

#include <ranges>
#include <algorithm>

KdNode* build(
    const std::vector<Particle>& particles,
    std::vector<int> indices,
    const int depth
) {
    if (indices.empty()) {
        return nullptr;
    }
    if (indices.size() == 1) {
        return new KdNode(indices[0]);
    }

    const auto mid = indices.size() / 2;

    std::ranges::sort(indices, [&](const int i, const int j) {
        if (depth % MOD == HORIZONTAL) {
            return particles[i].loc.x < particles[j].loc.x;
        } else {
            return particles[i].loc.y < particles[j].loc.y;
        }
    });

    const std::vector left_indices(indices.begin(), indices.begin() + mid);
    const std::vector right_indices(indices.begin() + std::min(mid + 1, indices.size()), indices.end());

    const auto ret = new KdNode(
        indices[mid],
        build(particles, left_indices, depth + 1),
        build(particles, right_indices, depth + 1)
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
