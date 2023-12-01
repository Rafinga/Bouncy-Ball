#ifndef PENDULUM_SYSTEM_
#define PENDULUM_SYSTEM_

#include "ParticleSystemBase.hpp"
#include <set>

namespace GLOO {
	struct SpringConnector {
		int parent_spring;
		float rest_length;
		float spring_constant;
	};
class PendulumSystem : public  ParticleSystemBase{

 public:


	 ParticleState ComputeTimeDerivative(const ParticleState& state,
		 float time) const override;

	 float g = 9.81;
	 float drag_constant = 5;


	 void AddParticle(const float mass);
	 void AddSpring(const int spring_index1, const int spring_index2, float spring_constant, const float rest_length);
	 void FixParticle(const int index);



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


private:
	
	std::vector<float> particle_masses;
	std::vector<std::vector<SpringConnector>> springs;
	std::set<int> fixed_particles;
	glm::vec3 CalculateSpringForce(const const ParticleState& state, const int index) const;

	bool wind_activated = false;

	float wind_strength = 9;



};
}  // namespace GLOO

#endif
