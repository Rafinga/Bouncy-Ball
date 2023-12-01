#ifndef PARTICLE_NODE_H_
#define PARTICLE_NODE_H_

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"
#include "ParticleState.hpp"
#include "CircularSystem.hpp"
#include "IntegratorBase.hpp"

#include <string>
#include <vector>

namespace GLOO {
class ParticleNode : public SceneNode {
 public:
  
	 std::unique_ptr<ParticleState> current_states;
	 double start_time = 0;
	 double total_time_elapsed = 0;

	 ParticleNode(std::unique_ptr<CircularSystem> forces, std::unique_ptr<IntegratorBase<CircularSystem, ParticleState>> integrator, double dt);

	 void Update(double delta_time) override;
	 void UpdateStates(ParticleState& new_states);


	 std::unique_ptr<CircularSystem> forces;
	 std::unique_ptr<IntegratorBase<CircularSystem,ParticleState>> integrator;
	 double dt;


private:
	bool in_mid_of_t_step = false;
	ParticleState full_t_step;
	ParticleState mid_step_start_state;
	bool prev_released = true;
	void ResetSystem();
};
}  // namespace GLOO

#endif
