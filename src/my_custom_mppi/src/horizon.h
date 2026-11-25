#pragma once
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

// curve contains the calibrated easy threshold and minimum fraction.
// The caller supplies full-length totals including the terminal critic.
inline int active_steps(std::vector<double> costs, int full, const double* curve, double& raw) {
    const auto count = (costs.size() + 19) / 20;
    std::partial_sort(costs.begin(), costs.begin() + count, costs.end());
    raw = std::accumulate(costs.begin(), costs.begin() + count, 0.) / count / full;
    double threshold = curve[0];
    double fraction = raw <= threshold ? 1. : std::max(curve[1], threshold / raw);
    return std::max(1, static_cast<int>(std::ceil(full * fraction)));
}
