#ifndef BOUNCY_SYSTEM_H_
#define BOUNCY_SYSTEM_H_

#include "ParticleSystemBase.hpp"
#include "ParticleState.hpp"
#include "glm/gtx/string_cast.hpp"

namespace GLOO {
    class BouncySystem : public ParticleSystemBase {
    public:
        /*BouncySystem::BouncySystem(glm::vec3 floor_normal, glm::vec3 floor_point) {
            floor_normal_ = floor_normal;
            floor_point_ = floor_point;
        }*/

        ParticleState ComputeTimeDerivative(const ParticleState& state,
            float time) const override {
            ParticleState new_state;
            new_state.positions = state.velocities;

            float epsilon = 0.000001f;
            for (int i = 0; i < particle_masses_.size(); i++) {
                //if (collided_indices_[i]) {
                //    std::cout << "got here!!" << std::endl;
                //    new_state.velocities.emplace_back(glm::vec3(0, 10, 0));
                //    continue; // 
                //}
                glm::vec3& gravitational_force = particle_masses_[i] * glm::vec3(0, -gravitational_constant_, 0);
                glm::vec3& drag_force = -drag_constant_ * state.velocities[i];
                glm::vec3& net_spring_force = glm::vec3(0);
                for (int j = 0; j < springs_.size(); j++) {
                    std::tuple<int, int, float, float> spring = springs_[j];
                    float rest_length = std::get<2>(spring);
                    float spring_constant = std::get<3>(spring);
                    if (i == std::get<0>(spring) || i == std::get<1>(spring)) {
                        int k;
                        if (i == std::get<0>(spring)) k = std::get<1>(spring);
                        else k = std::get<0>(spring);
                        glm::vec3 xi = state.positions[i];
                        glm::vec3 xk = state.positions[k];
                        glm::vec3 d = xi - xk;
                        glm::vec3 S_ik = -spring_constant * (glm::length(d) - rest_length) * glm::normalize(d);
                        net_spring_force += S_ik;
                    }
                }
                glm::vec3 particle_net_force = gravitational_force + drag_force + net_spring_force;

                new_state.velocities.emplace_back(particle_net_force / particle_masses_[i]);
            }

            return new_state;
        }

        void AddParticle(float particle_mass) {
            particle_masses_.emplace_back(particle_mass);
        }

        void AddSpring(int particle_1_idx, int particle_2_idx, float rest_length, float stiffness) {
            std::tuple<int, int, float, float> spring(particle_1_idx, particle_2_idx, rest_length, stiffness);
            springs_.emplace_back(spring);
        }

        void InitializeIndices(int n) {
            for (int i = 0; i < n; ++i) {
                collided_indices_.emplace_back(false);
            }
        }

        void IndexCollided(int i) {
            collided_indices_[i] = true;
        }

        void IndexUncollided(int i) {
            collided_indices_[i] = false;
        }

    private:
        std::vector<float> particle_masses_;
        std::vector<std::tuple<int, int, float, float>> springs_; // Each spring stores particle indices, rest length, and stiffness.

        float drag_constant_ = 5;
        float gravitational_constant_ = 9.81;

        glm::vec3 floor_normal_ = glm::vec3(0, 1, 0);
        glm::vec3 floor_point_ = glm::vec3(0, -1, 0);

        std::vector<bool> collided_indices_;
    };
}  // namespace GLOO

#endif