// Native CPU rollout and selection kernel.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>
#include <cstring>
#include <numeric>
#include "horizon.h"
#include "motion.h"
#include "critics.h"

#ifdef MPPI_CUDA
void cuda_plan(Motion,int,int,double,double,double,double,std::uint64_t,const double*,const double*,int,int,
               double,double,double,const double*,const double*,const double*,double*,double*,double*,double*,const double*,int&,double&,double*,double*);
#endif

namespace {
constexpr double pi = 3.14159265358979323846;
double wrap(double a) { return a - 2*pi*std::floor((a+pi)/(2*pi)); }
struct State { double x, y, yaw; };
State step(State s, double v, double w, double dt) {
    double mid = s.yaw + .5*w*dt;
    return {s.x+v*std::cos(mid)*dt, s.y+v*std::sin(mid)*dt, wrap(s.yaw+w*dt)};
}
}

// Returns 1 for full diagnostics, 0 for stop/alignment, -1 on native failure.
extern "C" int mppi_compute(int samples, int horizon, double dt, double temperature,
    double tracking_weight, double tracking_min_fraction,
    std::uint64_t seed, const double* pose, const double* grid, int rows, int cols,
    double resolution, double ox, double oy, const double* path, int path_size,
    double goal_yaw, double* controls, double* command, double* chosen,
    double* displayed, const double* motion, const double* reference_data, double* debug, double* weighted, const double* curve, double* horizon_info, double* inspection, double* probabilities) noexcept {
    try {
        int active=horizon;
        double& raw_cost=horizon_info[1];
        Motion m; static_assert(sizeof(m)==12*sizeof(double)); std::memcpy(&m,motion,sizeof(m));
        command[0] = command[1] = 0.;
        if (!path_size) return 0;
        State initial{pose[0], pose[1], pose[2]};
        auto cell = [&](State s) {
            double x = std::floor((s.x-ox)/resolution), y = std::floor((s.y-oy)/resolution);
            if (x<0 || y<0 || x>=cols || y>=rows) return -1.;
            return grid[static_cast<int>(y)*cols+static_cast<int>(x)];
        };
        auto unsafe = [&](State s) { double c=cell(s); return c<0 || c>=100; };
        auto command_unsafe = [&](double v, double w) {
            State s=initial;
            for (int i=0; i<10; ++i) {
                s=step(s,v,w,dt/10);
                if (unsafe(s)) return true;
            }
            return false;
        };
        if (std::hypot(initial.x-path[2*(path_size-1)], initial.y-path[2*(path_size-1)+1]) < .16) {
            command[0]=m.limit(0.,m.pv,0,dt);
            command[1]=m.limit(2.5*wrap(goal_yaw-initial.yaw),m.pw,1,dt);
            if(command_unsafe(command[0],command[1])) command[0]=command[1]=0.;
            return 0;
        }
        std::vector<double> refs(reference_data, reference_data+3*horizon+2+CRITIC_COUNT);
        const double* speeds=refs.data()+2*horizon;
        const double* critic_weights=refs.data()+3*horizon+2;
        // A fresh path-based proposal permits immediate changes of direction,
        // even when the stored plan still describes the previous turn.
        std::vector<double> guided(2*horizon);
        State guide_state=initial;
        double guide_v=m.pv,guide_w=m.pw;
        for (int t=0; t<horizon; ++t) {
            double dx=refs[2*t]-guide_state.x, dy=refs[2*t+1]-guide_state.y;
            double distance=std::hypot(dx,dy);
            double error=wrap(std::atan2(dy,dx)-guide_state.yaw);
            double v=distance>.02 ? std::min(speeds[t],distance/dt)*std::max(0.,std::cos(error)) : 0.;
            double w=distance>.02 ? std::clamp(2.5*error,m.w_min,m.w_max) : 0.;
            v=m.limit(v,guide_v,0,dt); w=m.limit(w,guide_w,1,dt);
            guide_v=v; guide_w=w;
            guided[2*t]=v; guided[2*t+1]=w;
            guide_state=step(guide_state,v,w,dt);
        }
#ifdef MPPI_CUDA
        std::vector<double> updated(2*horizon), candidates(2*horizon);
        std::vector<State> trajectories(horizon);
        cuda_plan(m,samples,horizon,dt,temperature,tracking_weight,tracking_min_fraction,seed,pose,grid,rows,cols,resolution,
                  ox,oy,refs.data(),controls,guided.data(),updated.data(),candidates.data(),displayed,debug,curve,active,raw_cost,inspection,probabilities);
        int best=0;
        State best_state=initial;
        for(int t=0;t<active;++t) {
            best_state=step(best_state,candidates[2*t],candidates[2*t+1],dt);
            trajectories[t]=best_state;
        }
#else
        std::mt19937_64 rng(seed);
        std::normal_distribution<double> normal;
        static thread_local std::vector<double> candidate_storage, cost_storage;
        auto &candidates = candidate_storage; auto &costs = cost_storage;
        candidates.resize(static_cast<size_t>(samples)*horizon*2);
        costs.assign(samples, 0.);
        std::vector<double> updated(2*horizon,0.);
        // Generate in the original sample/time/channel order before parallel scoring.
        for(double &value : candidates) value = normal(rng);
        static thread_local std::vector<State> trajectory_storage;
        auto &trajectories = trajectory_storage;
        trajectories.resize(static_cast<size_t>(samples)*horizon);
        std::vector<double> prefix(curve[1]>0 ? static_cast<size_t>(samples)*horizon : 0);
        #pragma omp parallel for schedule(static) num_threads(6) if(samples*horizon >= 4096)
        for (int k=0; k<samples; ++k) {
            State s=initial;
            double previous[2]={0.,0.}, velocity[2]={m.pv,m.pw};
            for (int t=0; t<horizon; ++t) {
                size_t offset=(static_cast<size_t>(k)*horizon+t)*2;
                double old_v=velocity[0], old_w=velocity[1];
                for (int d=0; d<2; ++d) {
                    double noise=candidates[offset+d]*(d==0 ? m.v_std : m.w_std);
                    if (t) noise=.65*previous[d]+.76*noise;
                    previous[d]=noise;
                    double baseline=k>=samples/2 ? guided[2*t+d] : controls[2*t+d];
                    double u=std::clamp(baseline+noise,d==0 ? m.v_min : m.w_min,d==0 ? m.v_max : m.w_max);
                    if (k==0) u=controls[2*t+d];
                    if (k==1) u=0.;
                    if (k==2) u=guided[2*t+d];
                    u=m.limit(u,velocity[d],d,dt);
                    velocity[d]=u;
                    candidates[offset+d]=u;
                }
                double v=candidates[offset], w=candidates[offset+1];
                bool swept = !t && command_unsafe(v,w);
                if (swept) costs[k]+=critic_weights[0];
                s=step(s,v,w,dt);
                trajectories[static_cast<size_t>(k)*horizon+t]=s;
                double c=cell(s);
                int gx=std::clamp(static_cast<int>(std::floor((s.x-ox)/resolution)),0,cols-1);
                int gy=std::clamp(static_cast<int>(std::floor((s.y-oy)/resolution)),0,rows-1);
                const double* segment=grid+rows*cols+4*(gy*cols+gx);
                double* entry=inspection ? inspection+(offset/2)*INSPECT_STRIDE : nullptr;
                costs[k]+=critic_step(critic_weights,s.x,s.y,s.yaw,c,refs[2*t],refs[2*t+1],
                    v,w,old_v,old_w,dt,speeds[t],m.v_max,
                    refs[3*horizon]>0.,segment,refs[3*horizon+1],entry ? entry+3 : nullptr);
                if(entry) {
                    entry[0]=s.x; entry[1]=s.y; entry[2]=s.yaw;
                    entry[3]+=swept;
                }
                if (!prefix.empty()) {
                    double dx=s.x-refs[2*t], dy=s.y-refs[2*t+1];
                    prefix[offset/2]=costs[k]+critic_weights[13]*(dx*dx+dy*dy);
                }
            }
            double dx=s.x-refs[2*(horizon-1)], dy=s.y-refs[2*(horizon-1)+1];
            costs[k]+=critic_weights[13]*(dx*dx+dy*dy);
        }
        if (!prefix.empty()) {
            active=active_steps(costs,horizon,curve,raw_cost);
            for (int k=0;k<samples;++k)
                costs[k]=prefix[static_cast<size_t>(k)*horizon+active-1];
        }
        int best=static_cast<int>(std::min_element(costs.begin(),costs.end())-costs.begin());
        if (debug || probabilities) {
        std::vector<int> order(samples);
        std::iota(order.begin(),order.end(),0);
        std::stable_sort(order.begin(),order.end(),[&](int a,int b){return costs[a]<costs[b];});
        double weight_sum=0.;
        for(double cost:costs) weight_sum+=std::exp(-(cost-costs[best])/temperature);
        if(probabilities)
            for(int k=0;k<samples;++k) probabilities[k]=std::exp(-(costs[k]-costs[best])/temperature)/weight_sum;
        int steps=horizon, width=3+2*steps;
        for(int rank=0;debug && rank<std::min(5,samples);++rank) {
            int k=order[rank];
            double* row=debug+rank*width;
            row[0]=k; row[1]=costs[k]; row[2]=std::exp(-(costs[k]-costs[best])/temperature)/weight_sum;
            for(int j=0;j<2*active;++j) row[3+j]=candidates[static_cast<size_t>(k)*horizon*2+j];
        }
        }
        double minimum=costs[best], sum=0.;
        for (double& cost:costs) { cost=std::exp(-(cost-minimum)/temperature); sum+=cost; }
        for (int k=0; k<samples; ++k)
            for (int j=0; j<2*active; ++j)
                updated[j]+=(costs[k]/sum)*candidates[static_cast<size_t>(k)*2*horizon+j];
#endif
        horizon_info[0]=active;
        double velocity[2]={m.pv,m.pw};
        for(int t=0;t<active;++t) for(int d=0;d<2;++d) {
            updated[2*t+d]=m.limit(updated[2*t+d],velocity[d],d,dt);
            velocity[d]=updated[2*t+d];
        }
        weighted[0]=updated[0]; weighted[1]=updated[1];
        int status=1;
        std::vector<State> selected(horizon);
        State s=initial;
        bool blocked=command_unsafe(updated[0],updated[1]);
        for (int t=0; t<active; ++t) {
            s=step(s,updated[2*t],updated[2*t+1],dt);
            selected[t]=s;
            blocked=blocked || unsafe(s);
        }
#ifndef MPPI_CUDA
        int stride=std::max(1,samples/45), index=0;
        for (int k=0; displayed && k<samples; k+=stride)
            for (int t=0; t<horizon; ++t) {
                State state=trajectories[static_cast<size_t>(k)*horizon+t];
                displayed[index++]=state.x; displayed[index++]=state.y; displayed[index++]=state.yaw;
            }
#endif
        if (blocked) {
            status=2;
            blocked=command_unsafe(candidates[static_cast<size_t>(best)*horizon*2],
                                   candidates[static_cast<size_t>(best)*horizon*2+1]);
            for (int t=0; t<active; ++t) {
                selected[t]=trajectories[static_cast<size_t>(best)*horizon+t];
                blocked=blocked || unsafe(selected[t]);
                for (int d=0; d<2; ++d) updated[2*t+d]=candidates[(static_cast<size_t>(best)*horizon+t)*2+d];
            }
            if (blocked) {
                for (int t=0; t<horizon; ++t) { controls[2*t]=std::min(m.v_max,m.a_max*dt*(t+1)); controls[2*t+1]=0.; }
                return 3;
            }
        }
        command[0]=updated[0]; command[1]=updated[1];
        for (int t=0; t<horizon; ++t) {
            int next=std::min(t+1,active-1);
            controls[2*t]=updated[2*next]; controls[2*t+1]=updated[2*next+1];
            if(t>=active) continue;
            chosen[3*t]=selected[t].x; chosen[3*t+1]=selected[t].y; chosen[3*t+2]=selected[t].yaw;
        }
        return status;
    } catch (...) { return -1; }
}
