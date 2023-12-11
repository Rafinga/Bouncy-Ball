#ifndef BOUNCY_NODE_3D_H_
#define BOUNCY_NODE_3D_H_

#include "ParticleState.hpp"
#include "gloo/SceneNode.hpp"
#include "BouncySystem.hpp"
#include "BouncySystem2.hpp"
#include "IntegratorBase.hpp"

namespace GLOO {
    class BouncyNode3D : public SceneNode {
    public:
        BouncyNode3D(std::unique_ptr<IntegratorBase<BouncySystem, ParticleState>> integrator,
        //BouncyNode3D(std::unique_ptr<IntegratorBase<BouncySystem2, ParticleState>> integrator,
            double integration_step,
            glm::vec3 center, float radius, int n);

        void BouncyNode3D::Update(double delta_time) override;

    private:
        std::unique_ptr<IntegratorBase<BouncySystem, ParticleState>> integrator_;
        //std::unique_ptr<IntegratorBase<BouncySystem2, ParticleState>> integrator_;
        BouncySystem system_;
        //BouncySystem2 system_;
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

        std::vector<int> horizontal_structural_indices_;
        std::vector<int> vertical_structural_indices_;

        std::vector<int> pressure_indices_;

        std::vector<int> bottom_diagonal_structural_indices_;
        std::vector<int> top_diagonal_structural_indices_;

        std::vector<glm::vec3> sphere_positions_;
        std::vector<glm::vec3> sphere_normals_;

        std::vector<int> triangle_indices_;  // Uses bottom-diagonal structural layout

        bool wireframe_enabled_ = true;

        bool horizontal_structurals_active_ = true;
        bool vertical_structurals_active_ = true;
        bool pressure_active_ = false;
        bool bottom_diagonals_active_ = true;
        bool top_diagonals_active_ = true;

        bool triangles_enabled_ = true;
        
        int num_of_children_;

        bool r_prev_released_ = false;
        bool s_prev_released_ = false;
        bool t_prev_released_ = false;
        bool w_prev_released_ = false;
        bool h_prev_released_ = false;
        bool v_prev_released_ = false;
        bool p_prev_released_ = false;
        bool b_prev_released_ = false;
        bool o_prev_released_ = false;

    };
}  // namespace GLOO

#endif