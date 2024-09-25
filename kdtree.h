#pragma once

#include "io.h"

constexpr int MOD = 16;

enum { HORIZONTAL, VERTICAL };

struct KdNode {
    int split{};
    KdNode* left = nullptr;
    KdNode* right = nullptr;

    ~KdNode() {
        delete left;
        delete right;
    }
};

KdNode* build(
    const std::vector<Particle>& particles,
    std::vector<int>& indices,
    int left_range,
    int right_range,
    int depth
);

std::vector<int> search(
    Vec2 loc,
    const KdNode* node,
    const std::vector<Particle>& particles,
    int scan_length,
    int depth = 0
);
