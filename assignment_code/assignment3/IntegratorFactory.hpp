#ifndef INTEGRATOR_FACTORY_H_
#define INTEGRATOR_FACTORY_H_

#include "IntegratorBase.hpp"

#include <stdexcept>

#include "gloo/utils.hpp"

#include "IntegratorType.hpp"
#include "ForwardEulerIntegrator.hpp"
#include "TrapezoidalIntegrator.hpp"
#include "RungeKuttaIntegrator.hpp"

namespace GLOO {
    class IntegratorFactory {
    public:
        template <class TSystem, class TState>
        static std::unique_ptr<IntegratorBase<TSystem, TState>> CreateIntegrator(
            IntegratorType type) {
            if (type == IntegratorType::Euler) {
                return std::move(make_unique<ForwardEulerIntegrator<TSystem, TState>>());
            }
            else if (type == IntegratorType::Trapezoidal) {
                return std::move(make_unique<TrapezoidalIntegrator<TSystem, TState>>());
            }
            else {
                return std::move(make_unique<RungeKuttaIntegrator<TSystem, TState>>());
            }
        }
    };
}  // namespace GLOO

#endif
