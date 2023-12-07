#ifndef BOUNCY_SYSTEM2_H
#define BOUNCY_SYSTEM2_H

#include "ParticleSystemBase.hpp"
#include <set>

namespace GLOO {
	struct SpringConnector {
		int parent_spring;
		float rest_length;
		float spring_constant;
	};
	class BouncySystem2 : public  ParticleSystemBase {

	public:

		ParticleState ComputeTimeDerivative(const ParticleState& state,
			float time) const override;

		float g = 9.81;
		float drag_constant = 0.7;


		void AddParticle(const float mass);
		void AddSpring(const int spring_index1, const int spring_index2, float spring_constant, const float rest_length);
		void FixParticle(const int index);
		glm::vec3 CalculateDampSpringForce(const ParticleState& state, const int particle_index, float dt) const;
		//void UnfixParticle(const int index);



		ParticleState particle_states;
		void FlipWindActiveness() {

			wind_activated = !wind_activated;
		}

		void IncreaseWindStrength() {
			wind_strength += 0.001;
		}
		void DecreaseWindStrenght() {
			wind_strength -= 0.001;
		}

		void SetFloorNormal(glm::vec3 floor_normal) {
			this->floor_normal = floor_normal;
		}

		void SetFloorSurfacePoint(glm::vec3 floor_surface_point) {
			this->floor_surface_point = floor_surface_point;
		}


	private:

		std::vector<float> particle_masses;
		std::vector<std::vector<SpringConnector>> springs;
		std::set<int> fixed_particles;
		glm::vec3 CalculateSpringForce(const ParticleState& state, const int index) const;

		bool wind_activated = false;

		float wind_strength = 9;

		glm::vec3 floor_normal;
		glm::vec3 floor_surface_point;

		bool IsTouchingGround(glm::vec3 pos) const;




		glm::vec3 GigaSpringForce(const ParticleState& state) const;

		glm::vec3 FloorCompressionForce(const glm::vec3 particle_pos) const;


		float giga_dampening = 0.90;

		float friction_constant = 900;

		glm::vec3 CalculateFriction(float mass,const glm::vec3& normal_force,const  glm::vec3& velocity,float dt) const ;


	};
}  // namespace GLOO

#endif
