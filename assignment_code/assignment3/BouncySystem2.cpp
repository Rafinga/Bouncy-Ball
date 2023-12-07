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
			const glm::vec3& spring_force = CalculateDampSpringForce(state, i,time);


			glm::vec3& net_force = g_force + d_force + spring_force;
			if (wind_activated) {
				net_force += wind_strength * glm::vec3(0, 1, 0);
			}




			glm::vec3 net_accel = net_force / mass ;

			if ( i != state.positions.size()-1 && IsTouchingGround(state.positions[i]) && state.velocities[i].y < 0 ) {

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
			float dampening_constant = 2.5 * std::pow(spring_const * particle_masses.at(parent_index), 0.5);


			const float new_size = glm::length(distance_vector);
			glm::vec3 extra_force = -spring_const * (new_size - rest_length) * distance_vector / new_size;
			net_force += extra_force;
			glm::vec3 damp_force = -dampening_constant*(new_size - rest_length) / dt * (distance_vector / new_size);
			net_force += damp_force;

		}
		return net_force;
	}

	bool BouncySystem2::IsTouchingGround(glm::vec3 pos) const {



		float delta = 0.00000001;
		glm::vec3 displacemnet_vector = pos - floor_surface_point;
		return glm::dot(displacemnet_vector, floor_normal) <= delta;
		
	}
	glm::vec3 BouncySystem2::GigaSpringForce(const ParticleState& state) const {

		glm::vec3 center_pos(0, 0, 0);

		for (int i = 0; i < state.positions.size(); i++) {
			center_pos += state.positions[i];
		}
		center_pos = (1.0f / state.positions.size()) * center_pos;
		float giga_constant = 900;
		float radius = 1;

		float stretch_length = center_pos.y - floor_surface_point.y;

		float giga_compression =(stretch_length-radius);
		return -floor_normal * giga_compression * giga_constant;


	}

	glm::vec3 BouncySystem2::CalculateFriction(float mass, const glm::vec3& normal_force, const  glm::vec3& velocity,float dt)const  {

		float magnitude = glm::length(normal_force);


		glm::vec3 parallel_vel_component = glm::dot(floor_normal, velocity) * floor_normal;


		if (glm::length(velocity - parallel_vel_component) == 0) {
			return glm::vec3(0,0,0);
		}


		glm::vec3& fric_direction =-glm::normalize( velocity - parallel_vel_component);

		glm::vec3 max_fric_force = friction_constant * fric_direction * magnitude;


		float max_change_vel = mass*glm::length(velocity - parallel_vel_component) / (dt);
		glm::vec3 vel_upper_bound_force = fric_direction * max_change_vel;


		return vel_upper_bound_force;

		//return max_change_vel > magnitude ? max_fric_force : vel_upper_bound_force;





	}

	glm::vec3 BouncySystem2::FloorCompressionForce(const glm::vec3 particle_pos) const {
	

		float  dist_from_floor = glm::dot(particle_pos, floor_normal);
		float delta = 0.01;


		float compression = delta - dist_from_floor;

		float giga_constant = 90000;

		return giga_constant * compression * floor_normal;
	}



}  // namespace GLOO
