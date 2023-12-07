#ifndef BOUNCY_NODE2_H_
#define BOUNCY_NODE2_H_

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"
#include "ParticleState.hpp"
#include "BouncySystem2.hpp"
#include "IntegratorBase.hpp"

#include <string>
#include <vector>

namespace GLOO {
	class BouncyNode2 : public SceneNode {
	public:

		std::unique_ptr<ParticleState> current_states;
		double start_time = 0;
		double total_time_elapsed = 0;
		double dimension = 2;
		unsigned int nodes_in_edge;



		BouncyNode2(
			std::unique_ptr<IntegratorBase<BouncySystem2, ParticleState>> integrator,
			double dt,
			glm::vec3 center,
			float radius,
			unsigned int circumference_partition,
			glm::vec3& throw_speed = glm::vec3(0)

		);

		void Update(double delta_time) override;
		void UpdateStates(ParticleState& new_states);



		std::unique_ptr<BouncySystem2> forces;
		std::unique_ptr<IntegratorBase<BouncySystem2, ParticleState>> integrator;
		double dt;
		float spring_constant = 630;
		float default_mass = 1;


	private:
		bool in_mid_of_t_step = false;
		ParticleState full_t_step;
		ParticleState mid_step_start_state;

		void AddParticle(const glm::vec3& starting_position, float mass = 1.0f);
		std::shared_ptr<VertexObject> ball_vertices;
		void UpdateSystem();
		void CreateBallSystem();

		bool IsTouchingGround(glm::vec3 pos) const;


		void AddStretchSprings();
		void AddDeformingStretchSprings();

		void AddStructuralSprings();
		void AddFlexSprings();
		void AddBounceSprings();
		void AddCentralSprings();

		void AddSeparatorSprings();

		void ResetSystem();


		glm::vec3 center;
		glm::vec3 throw_speed;
		float radius;
		bool prev_released = true;
		bool stop_pressed = false;

		unsigned int circumference_partition;
	};
}  // namespace GLOO

#endif
