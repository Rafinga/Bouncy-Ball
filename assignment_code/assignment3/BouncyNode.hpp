#ifndef BOUNCY_NODE_H_
#define BOUNCY_NODE_H_

#include "ParticleState.hpp"
#include "gloo/SceneNode.hpp"
#include "BouncySystem.hpp"
#include "IntegratorBase.hpp"

namespace GLOO {
    class BouncyNode : public SceneNode {
    public:
        BouncyNode(std::unique_ptr<IntegratorBase<BouncySystem, ParticleState>> integrator,
            double integration_step,
            glm::vec3 center, float radius, int n);

        void BouncyNode::Update(double delta_time) override;

    private:
        std::unique_ptr<IntegratorBase<BouncySystem, ParticleState>> integrator_;
        BouncySystem system_;
        ParticleState state_;
        double current_time_;
        double time_left_over_;
        double integration_step_;

        glm::vec3 floor_normal_;
        glm::vec3 floor_point_;
    };
}  // namespace GLOO

#endif