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
        //BouncyNode3D(std::unique_ptr<IntegratorBase<BouncySystem, ParticleState>> integrator,
        BouncyNode3D(std::unique_ptr<IntegratorBase<BouncySystem2, ParticleState>> integrator,
            double dt,
            glm::vec3 center, float radius, int n,
            glm::vec3 floor_position, glm::vec3 floor_normal,
            glm::vec3 initial_velocity
        );

        void BouncyNode3D::Update(double delta_time) override;

    private:
        void UpdateStates(ParticleState& new_states);
        bool IsTouchingGround(glm::vec3 pos) const;
        glm::vec3 RemovedTableVelocityComp(glm::vec3& velocity);
        void ResetSystem();
        void UpdateVertexObjectPositions(int child_idx);
        void UpdateActiveVertexObjectPositions();

        //std::unique_ptr<IntegratorBase<BouncySystem, ParticleState>> integrator_;
        std::unique_ptr<IntegratorBase<BouncySystem2, ParticleState>> integrator_;
        //BouncySystem system_;
        BouncySystem2 system_;
        ParticleState state_;
        double start_time_;
        double total_time_elapsed_;
        double dt_;

        int num_children_;

        glm::vec3 floor_normal_;
        glm::vec3 floor_point_;

        std::vector<glm::vec3> sphere_positions_;
        std::vector<glm::vec3> initial_velocities_;
        std::vector<glm::vec3> sphere_normals_;

        std::vector<int> horizontal_structural_indices_;
        std::vector<int> vertical_structural_indices_;

        std::vector<int> pressure_indices_;

        std::vector<int> bottom_diagonal_structural_indices_;
        std::vector<int> top_diagonal_structural_indices_;

        std::vector<int> triangle_indices_;  // Uses bottom-diagonal structural layout

        bool wireframe_enabled_ = true;

        bool horizontal_structurals_active_ = true;
        bool vertical_structurals_active_ = true;
        bool pressure_active_ = false;
        bool bottom_diagonals_active_ = true;
        bool top_diagonals_active_ = true;

        bool triangles_enabled_ = true;

        bool r_prev_released_ = false;
        bool s_prev_released_ = false;
        bool t_prev_released_ = false;

        bool h_prev_released_ = false;
        bool v_prev_released_ = false;
        bool p_prev_released_ = false;
        bool b_prev_released_ = false;
        bool o_prev_released_ = false;
        bool w_prev_released_ = false;

        bool time_frozen_ = false;

    };
}  // namespace GLOO

#endif
