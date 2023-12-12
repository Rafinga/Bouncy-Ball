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
			const glm::vec3& spring_force = CalculateDampSpringForce(state, i, time);


			glm::vec3& net_force = g_force + d_force + spring_force;
			if (wind_activated) {
				net_force += wind_strength * glm::vec3(0, 1, 0);
			}




			glm::vec3 net_accel = net_force / mass;

			if (IsTouchingGround(state.positions[i]) && IsMovingThroughGround(state.velocities.at(i))) {

				//new_states.positions.push_back(glm::vec3(0));
			//	
				//glm::vec3 recalced_force(0, 0, 0);
				//if (giga_spring_force.y > 0) {
					//recalced_force += giga_spring_force;
				//}
				//else {
					//recalced_force += FloorCompressionForce(state.positions[i]);
				//}


				//glm::vec3 recalced_acel = recalced_force / mass;

				//new_states.velocities.push_back(recalced_acel);




				//continue;
			//}

			//new_states.positions.push_back(state.velocities.at(i));
			//new_states.velocities.push_back(net_accel);

			//glm::vec3 friction_force = CalculateFriction(mass,-g_force, state.velocities.at(i), time);
			//net_accel += friction_force/mass;
				new_states.positions.push_back(glm::vec3(0));
				new_states.velocities.push_back(net_accel);
				continue;


			}

			new_states.positions.push_back(state.velocities.at(i));
			new_states.velocities.push_back(net_accel);
		}

		return new_states;
	}

	glm::vec3 BouncySystem2::CalculateSpringForce(const ParticleState& state, const int particle_index) const {

		float dampening_constant = 2;
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


	glm::vec3 BouncySystem2::CalculateDampSpringForce(const ParticleState& state, const int particle_index, float dt) const {

		glm::vec3 net_force(0, 0, 0);
		const glm::vec3& current_pos = state.positions.at(particle_index);
		for (SpringConnector spring : springs.at(particle_index)) {
			const int& parent_index = spring.parent_spring;
			const float& spring_const = spring.spring_constant;
			const float& rest_length = spring.rest_length;
			const glm::vec3& parent_pos = state.positions.at(parent_index);
			const glm::vec3& distance_vector = current_pos - parent_pos;


			//float dampening_constant = 2*std::pow(spring_const*particle_masses.at(parent_index),0.5);
			float dampening_constant = 64 * std::pow(spring_const * particle_masses.at(parent_index), 0.5);


			const float new_size = glm::length(distance_vector);
			float current_length = new_size - rest_length;
			if (abs(current_length) <= 0.00001) {
				current_length = 0;
			}

			glm::vec3 extra_force = -spring_const * (current_length) * distance_vector / new_size;
			net_force += extra_force;
			glm::vec3 damp_force = -dampening_constant * (current_length) / dt * (distance_vector / new_size);
			net_force += damp_force;

		}
		return net_force;
	}

	bool BouncySystem2::IsTouchingGround(glm::vec3 pos) const {



		float delta = 0.00000001;
		glm::vec3 displacemnet_vector = pos - floor_surface_point;
		return glm::dot(displacemnet_vector, floor_normal) <= delta;

	}

	bool BouncySystem2::IsMovingThroughGround(const glm::vec3& vel)const {
		return glm::dot(vel, floor_normal) < 0;
	}


}  // namespace GLOO