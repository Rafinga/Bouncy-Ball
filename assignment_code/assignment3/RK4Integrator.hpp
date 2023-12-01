#ifndef RK4_INTEGRATOR_H_
#define RK4_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO {
template <class TSystem, class TState>
class RK4Integrator : public IntegratorBase<TSystem, TState> {
  TState Integrate(const TSystem& system,
                   const TState& state,
                   float start_time,
                   float dt) const override {
    // TODO: Here we are returning the state at time t (which is NOT what we
    // want). Please replace the line below by the state at time t + dt using
    // forward Euler integration.

      ParticleState& k1 = system.ComputeTimeDerivative(state, start_time);
      ParticleState& k2 = system.ComputeTimeDerivative(state + dt/2 * k1, dt + start_time);
      ParticleState& k3 = system.ComputeTimeDerivative(state + dt / 2 * k2, dt + start_time);
      ParticleState& k4 = system.ComputeTimeDerivative(state + dt / 2 * k3, dt + start_time);


          ParticleState & new_states = state + dt / 6 * (k1 + 2*k2 + k4);
      return new_states;
  }
};
}  // namespace GLOO

#endif
