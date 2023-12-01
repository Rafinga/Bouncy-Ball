#ifndef TRAPEZOID_RULE_INTEGRATOR_H_
#define TRAPEZOID_RULE_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO {
template <class TSystem, class TState>
class TrapezoidRuleIntegrator : public IntegratorBase<TSystem, TState> {
  TState Integrate(const TSystem& system,
                   const TState& state,
                   float start_time,
                   float dt) const override {
    // TODO: Here we are returning the state at time t (which is NOT what we
    // want). Please replace the line below by the state at time t + dt using
    // forward Euler integration.
      
      ParticleState& f0 = system.ComputeTimeDerivative(state, start_time);
      ParticleState& f1 = system.ComputeTimeDerivative(state + dt * f0, dt + start_time);


      ParticleState& new_states = state + dt /2 *(f0+f1);
    return new_states;
  }
};
}  // namespace GLOO

#endif
