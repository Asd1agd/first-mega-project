#pragma once
#include <cstdint>
extern "C" int mppi_compute(int samples, int horizon, double dt, double temperature,
    double tracking_weight, double tracking_min_fraction,
    std::uint64_t seed, const double* pose, const double* grid, int rows, int cols,
    double resolution, double ox, double oy, const double* path, int path_size,
    double goal_yaw, double* controls, double* command, double* chosen,
    double* displayed, const double* motion, const double* reference_data, double* debug, double* weighted, const double* curve, double* horizon_info, double* inspection, double* probabilities) noexcept;
