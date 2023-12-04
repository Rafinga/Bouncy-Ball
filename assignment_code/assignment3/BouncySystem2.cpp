#include "BouncySystem2.hpp"

namespace GLOO {

	void BouncySystem2::AddParticle(const float mass) {
		particle_masses.emplace_back(mass);
		springs.push_back(std::vector<SpringConnector>());
	}
	void BouncySystem2::FixParticle(const int index) {
		fixed_particles.insert(index);
	}
	//void BouncySystem2::UnfixParticle(const int index) {
	//	std::remove(fixed_particles.begin(), fixed_particles.end(), index), fixed_particles.end();
	//}



	void BouncySystem2::AddSpring(const int spring_index1, const int spring_index2, float spring_constant, const float rest_length) {
		SpringConnector first_spring;

		first_spring.parent_spring = spring_index2;
		first_spring.rest_length = rest_length;
		first_spring.spring_constant = spring_constant;

		SpringConnector second_spring = first_spring;
		second_spring.parent_spring = spring_index1;


		springs.at(spring_index1).push_back(first_spring);
		springs.at(spring_index2).push_back(second_spring);

	}
	ParticleState BouncySystem2::ComputeTimeDerivative(const ParticleState& state, float time) const {

		ParticleState new_states;
		glm::vec3 giga_spring_force = GigaSpringForce(state);

		for (int i = 0; i < state.positions.size(); i++) {
			//if (fixed_particles.count(i) > 0) {
			//	//vel
			//	new_states.positions.push_back(glm::vec3(0));
			//	//accel
			//	new_states.velocities.push_back(glm::vec3(0));
			//	continue;
			//}

			const float& mass = particle_masses.at(i);


			const glm::vec3 g_force(0, -g * mass, 0);
			const glm::vec3& d_force = -drag_constant * state.velocities.at(i);
			const glm::vec3& spring_force = CalculateSpringForce(state, i);

			glm::vec3& net_force = g_force + d_force + spring_force;
			bool touched_the_ground = false;
			if (wind_activated) {
				net_force += wind_strength * glm::vec3(0, 1, 0);
			}

			//const glm::vec3& net_force = d_force + spring_force;
			const glm::vec3& net_accel = 1.0f / mass * net_force;

			if ( IsTouchingGround(state.positions[i])&& state.velocities[i].y <0) {
				new_states.positions.push_back(glm::vec3(0));
				glm::vec3 frictional_force = CalculateFriction(-g_force, state.velocities[i]);
				glm::vec3 net_ground_force = frictional_force + giga_spring_force;
				glm::vec3 net_ground_accel = (frictional_force + net_ground_force) / mass;


				


				new_states.velocities.push_back(net_ground_accel);
				touched_the_ground = true;
				continue;
			}

			new_states.positions.push_back(state.velocities.at(i));
			new_states.velocities.push_back(net_accel);
		}

		return new_states;
	}

	glm::vec3 BouncySystem2::CalculateSpringForce(const ParticleState& state, const int particle_index) const {

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

	bool BouncySystem2::IsTouchingGround(glm::vec3 pos) const {
		
		float delta = 0.01;
		glm::vec3 displacemnet_vector = pos - floor_surface_point;
		return glm::dot(displacemnet_vector, floor_normal) <= 0;
		
	}
	glm::vec3 BouncySystem2::GigaSpringForce(const ParticleState& state) const {

		glm::vec3 center_pos(0, 0, 0);

		for (int i = 0; i < state.positions.size(); i++) {
			center_pos += state.positions[i];
		}
		center_pos = (1.0f / state.positions.size()) * center_pos;
		float giga_constant = 900;
		float radius = 1;

		float giga_compression = 1 - center_pos.y;
		return floor_normal * giga_compression * giga_constant;


	}

	glm::vec3 BouncySystem2::CalculateFriction(const glm::vec3& normal_force, const  glm::vec3& velocity)const  {

		float magnitude = glm::length(normal_force);


		glm::vec3 parallel_vel_component = glm::dot(floor_normal, velocity) * floor_normal;

		glm::vec3& fric_direction =-glm::normalize( velocity - parallel_vel_component);

		return friction_constant * fric_direction * magnitude;



	}



}  // namespace GLOO
