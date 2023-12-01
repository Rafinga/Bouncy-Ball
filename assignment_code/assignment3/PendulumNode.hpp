#ifndef PENDULUM_NODE_H_
#define PENDULUM_NODE_H_

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"
#include "ParticleState.hpp"
#include "PendulumSystem.hpp"
#include "IntegratorBase.hpp"

#include <string>
#include <vector>

namespace GLOO {
	class PendulumNode : public SceneNode {
 public:
  
	 std::unique_ptr<ParticleState> current_states;
	 double start_time = 0;
	 double total_time_elapsed = 0;

	 PendulumNode(std::unique_ptr<PendulumSystem> forces, std::unique_ptr<IntegratorBase<PendulumSystem, ParticleState>> integrator, double dt);

	 void Update(double delta_time) override;
	 void UpdateStates(ParticleState& new_states);


	 std::unique_ptr<PendulumSystem> forces;
	 std::unique_ptr<IntegratorBase<PendulumSystem,ParticleState>> integrator;
	 double dt;
	 float spring_constant = 100;
	 float default_mass = 2;


private:
	bool in_mid_of_t_step = false;
	ParticleState full_t_step;
	ParticleState mid_step_start_state;
	void AddParticle(const glm::vec3& starting_position);
	PositionArray default_node_positions;
	void ResetSystem();
	bool prev_released = true;

};
}  // namespace GLOO

#endif
