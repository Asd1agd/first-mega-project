// Shared CPU/CUDA scoring. Weight order must match critics.CRITICS.
#pragma once
#include "motion.h"
constexpr int CRITIC_COUNT=14;
constexpr int INSPECT_STRIDE=3+CRITIC_COUNT;
MPPI_HD inline double critic_wrap(double a) {
    constexpr double pi=3.14159265358979323846;
    return a-2*pi*floor((a+pi)/(2*pi));
}
MPPI_HD inline double critic_square(double a) { return a*a; }
MPPI_HD inline double critic_step(const double* weights, double x, double y, double yaw,
    double cell, double rx, double ry, double v, double w,
    double old_v, double old_w, double dt, double speed, double vmax,
    bool recovery, const double* segment, double rejoin_distance, double* raw=nullptr) {
    double q=fmin(1.,fmax(0.,cell/100.)), dx=rx-x, dy=ry-y;
    double distance2=dx*dx+dy*dy;
    double ex=segment[2]-segment[0], ey=segment[3]-segment[1], length2=ex*ex+ey*ey;
    double ox=x-segment[0], oy=y-segment[1];
    double fraction=fmin(1.,fmax(0.,(ox*ex+oy*ey)/fmax(length2,1e-12)));
    double lateral2=critic_square(ox-fraction*ex)+critic_square(oy-fraction*ey);
    double angle2=length2>1e-12 ? critic_square(critic_wrap(yaw-atan2(ey,ex))) : 0.;
    const double terms[] = {
        double(cell<0 || cell>=100), fmax(cell,0.), fmax(cell,0.)*v*v,
        q*q/(1.1-q), distance2, lateral2, angle2/(1.+4.*lateral2)*(!recovery),
        w*w, critic_square((v-old_v)/dt), critic_square((w-old_w)/dt), critic_square(v-speed),
        critic_square(fmax(fabs(v)-vmax*(1.-.7*q),0.)),
        critic_square(critic_wrap(atan2(dy,dx)-yaw))*(distance2>critic_square(rejoin_distance))*recovery
    };
    double total=0.;
    for(int i=0;i<CRITIC_COUNT-1;++i) {
        if(raw) raw[i]=terms[i];
        total+=weights[i]*terms[i];
    }
    return total;
}
