#ifndef CIRCULAR_SYSTEM_
#define CIRCULAR_SYSTEM_

#include "ParticleSystemBase.hpp"

namespace GLOO {
class CircularSystem : public  ParticleSystemBase{

 public:

	 ParticleState ComputeTimeDerivative(const ParticleState& state,
		 float time) const override {

		 const glm::vec3& particle_positions = state.positions.at(0);

		 glm::vec3 velocity(-particle_positions.y, particle_positions.x, 0);
		 glm::vec3 acceleration(0, 0, 0);

		 ParticleState new_state;
		 new_state.positions.push_back(velocity);
		 new_state.velocities.push_back(acceleration);
		 return new_state;
	 }


};
}  // namespace GLOO

#endif
