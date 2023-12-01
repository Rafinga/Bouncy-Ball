#include "PendulumNode.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp";
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO {

	PendulumNode::PendulumNode(std::unique_ptr<PendulumSystem> forces, std::unique_ptr<IntegratorBase<PendulumSystem, ParticleState>> integrator, double dt) :
		forces(std::move(forces)), integrator(std::move(integrator)), dt(dt)
	{
		current_states = make_unique<ParticleState>();
		AddParticle(glm::vec3(0,1,0));

		AddParticle(glm::vec3(0, 0, 0));
		this-> forces->AddSpring(0, 1, spring_constant, 1);

		this->forces->FixParticle(0);

		AddParticle(glm::vec3(-2, 0, 0));

		AddParticle(glm::vec3(-2, -0.5, 0));

		this->forces->AddSpring(1, 2, spring_constant, 2);

		this->forces->AddSpring(2, 3, spring_constant, 0.5);



	}

	void PendulumNode::AddParticle(const glm::vec3& starting_position) {
		std::unique_ptr<VertexObject> particle = PrimitiveFactory::CreateSphere(0.1, 10, 10);
		std::unique_ptr<ShaderProgram> phongShader = make_unique<PhongShader>();
		std::unique_ptr<SceneNode> fixed_node = make_unique<SceneNode>();
		fixed_node->CreateComponent<RenderingComponent>(std::move(particle));
		fixed_node->CreateComponent<ShadingComponent>(std::move(phongShader));
		AddChild(std::move(fixed_node));
		glm::vec3 starting_speed(0, 0, 0);

		const int& child_index = GetChildrenCount()-1;
		GetChild(child_index).GetTransform().SetPosition(starting_position);
		current_states->positions.push_back(starting_position);
		current_states->velocities.push_back(glm::vec3(0, 0, 0));
		forces->AddParticle(default_mass);
		default_node_positions.push_back(starting_position );

	}

	void PendulumNode::Update(double delta_time) {

		total_time_elapsed += delta_time;

		while (start_time + dt < total_time_elapsed) {
			ParticleState& new_states = integrator->Integrate(*forces, *current_states, start_time, dt);
			UpdateStates(new_states);
			start_time += dt;

		}

		if (InputManager::GetInstance().IsKeyPressed('W')) {
			if (prev_released) {
				forces->FlipWindActiveness();
			}
			prev_released = false;
		}
		else if (InputManager::GetInstance().IsKeyReleased('W')) {
			prev_released = true;

		}

		if (InputManager::GetInstance().IsKeyPressed('R')) {
			if (prev_released) {
				ResetSystem();
			}
			prev_released = false;
		}
		else if (InputManager::GetInstance().IsKeyReleased('R')) {
			prev_released = true;

		}


		if (InputManager::GetInstance().IsKeyPressed(GLFW_KEY_UP)) {
			forces->IncreaseWindStrength();

		}
		if (InputManager::GetInstance().IsKeyPressed(GLFW_KEY_DOWN)) {
			forces->DecreaseWindStrenght();

		}

	}


	void PendulumNode::UpdateStates(ParticleState& new_states) {
		current_states->positions.clear();
		int child_index = 0;
		for (const glm::vec3& new_position : new_states.positions) {
			current_states->positions.push_back(new_position);
			GetChild(child_index).GetTransform().SetPosition(new_position);
			child_index++;
		}

		current_states->velocities.clear();
		for (const glm::vec3& new_velocity : new_states.velocities) {
			current_states->velocities.push_back(new_velocity);
		}

	}

	void PendulumNode::ResetSystem() {
		ParticleState default_pendulum_state;
		for (glm::vec3& position : default_node_positions) {
			default_pendulum_state.positions.push_back(position);
			default_pendulum_state.velocities.emplace_back(0, 0, 0);
		}
		current_states->positions = default_pendulum_state.positions;
		current_states->velocities = default_pendulum_state.velocities;
	}


}  // namespace GLOO
