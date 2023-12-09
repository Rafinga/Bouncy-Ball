#ifndef BOUNCY_NODE_3D_H_
#define BOUNCY_NODE_3D_H_

#include "ParticleState.hpp"
#include "gloo/SceneNode.hpp"
#include "BouncySystem.hpp"
#include "IntegratorBase.hpp"

namespace GLOO {
    class BouncyNode3D : public SceneNode {
    public:
        BouncyNode3D(std::unique_ptr<IntegratorBase<BouncySystem, ParticleState>> integrator,
            double integration_step,
            glm::vec3 center, float radius, int n);

        void BouncyNode3D::Update(double delta_time) override;

    private:
        std::unique_ptr<IntegratorBase<BouncySystem, ParticleState>> integrator_;
        BouncySystem system_;
        ParticleState state_;
        double current_time_;
        double time_left_over_;
        double integration_step_;

        glm::vec3 floor_normal_;
        glm::vec3 floor_point_;

        std::vector<int> collided_indices_;
        std::vector<glm::vec3> initial_heights_;
        std::vector<bool> falling_;
        std::vector<bool> initial_height_set_;

        float g = 9.81;
        float coefficient_of_restitution_ = 0.9f;
    };
}  // namespace GLOO

#endif