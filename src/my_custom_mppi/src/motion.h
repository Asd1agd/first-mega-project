// Shared signed velocity / acceleration constraints for C++ and CUDA.
#pragma once
#include <cmath>
#ifdef __CUDACC__
#define MPPI_HD __host__ __device__
#else
#define MPPI_HD
#endif
struct Motion {
    // Same field order as mppi.MOTION_DEFAULTS, followed by applied [v,w].
    double v_min,v_max,v_std,a_max,a_min,w_min,w_max,w_std,alpha_max,alpha_min,pv,pw;
    MPPI_HD double limit(double requested,double previous,int d,double dt) const {
        double low=d?w_min:v_min, high=d?w_max:v_max;
        double lower=fmax(low,previous+(d?alpha_min:a_min)*dt);
        double upper=fmin(high,previous+(d?alpha_max:a_max)*dt);
        return fmin(high,fmax(low,fmin(upper,fmax(lower,requested))));
    }
};
