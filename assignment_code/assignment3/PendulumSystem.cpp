#include "PendulumSystem.hpp"

namespace GLOO {

	void PendulumSystem::AddParticle(const float mass) {
		particle_masses.emplace_back(mass);
		springs.push_back(std::vector<SpringConnector>());
	}
	void PendulumSystem::FixParticle(const int index) {
		fixed_particles.insert(index);
	}
	void PendulumSystem::AddSpring(const int spring_index1, const int spring_index2, float spring_constant, const float rest_length) {
		SpringConnector first_spring;

		first_spring.parent_spring = spring_index2;
		first_spring.rest_length = rest_length;
		first_spring.spring_constant = spring_constant;

		SpringConnector second_spring = first_spring;
		second_spring.parent_spring = spring_index1;


		springs.at(spring_index1).push_back(first_spring);
		springs.at(spring_index2).push_back(second_spring);

	}
	ParticleState PendulumSystem::ComputeTimeDerivative(const ParticleState& state, float time) const {

		ParticleState new_states;


		for (int i = 0; i < state.positions.size(); i++) {
			if (fixed_particles.count(i) > 0) {
				//vel
				new_states.positions.push_back(glm::vec3(0));
				//accel
				new_states.velocities.push_back(glm::vec3(0));
				continue;
			}

			const float& mass = particle_masses.at(i);


			const glm::vec3 g_force(0, -g * mass, 0);
			const glm::vec3& d_force = -drag_constant * state.velocities.at(i);
			const glm::vec3& spring_force = CalculateSpringForce(state, i);

			glm::vec3& net_force = g_force + d_force + spring_force;

			if (wind_activated) {
				net_force += wind_strength * glm::vec3(0, 1, 0);
			}

			//const glm::vec3& net_force = d_force + spring_force;
			const glm::vec3& net_accel = 1.0f / mass * net_force;
			new_states.positions.push_back(state.velocities.at(i));
			new_states.velocities.push_back(net_accel);
		}

		return new_states;
	}

	glm::vec3 PendulumSystem::CalculateSpringForce(const ParticleState& state, const int particle_index) const {

	glm::vec3 net_force(0, 0, 0);
	const glm::vec3& current_pos = state.positions.at(particle_index);
	for (SpringConnector spring : springs.at(particle_index)) {
				const int& parent_index = spring.parent_spring;
				const float& spring_const = spring.spring_constant;
				const float& rest_length = spring.rest_length;
				const glm::vec3& parent_pos = state.positions.at(parent_index);
				const glm::vec3& distance_vector = current_pos - parent_pos;
				const float new_size = glm::length(distance_vector);
				glm::vec3 extra_force = -spring_const * (new_size - rest_length) * distance_vector / new_size;
				net_force += extra_force;
		}
	return net_force;
	}




}  // namespace GLOO
