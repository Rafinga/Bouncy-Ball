#include "ParticleNode.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp";
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO {

	ParticleNode::ParticleNode(std::unique_ptr<CircularSystem> forces, std::unique_ptr<IntegratorBase<CircularSystem, ParticleState>> integrator, double dt) :
		forces(std::move(forces)), integrator(std::move(integrator)), dt(dt)
	{
		current_states = make_unique<ParticleState>();
		std::unique_ptr<VertexObject> particle = PrimitiveFactory::CreateSphere(0.1, 10, 10);
		std::unique_ptr<ShaderProgram> phongShader = make_unique<PhongShader>();
		std::unique_ptr<SceneNode> particle_node = make_unique<SceneNode>();
		particle_node->CreateComponent<RenderingComponent>(std::move(particle));
		particle_node->CreateComponent<ShadingComponent>(std::move(phongShader));
		particle_node->CreateComponent < MaterialComponent >(make_unique < Material >(Material::GetDefault()));
		particle_node->GetComponentPtr<MaterialComponent>()->GetMaterial().SetDiffuseColor(glm::vec3(0.4, 0.4, 0));
		AddChild(std::move(particle_node));

		glm::vec3 starting_position(1, 0, 0);
		glm::vec3 cordinate_origin(-3, 0, 0);
		glm::vec3 starting_speed(0, 0, 0);
		GetTransform().SetPosition(starting_position  + cordinate_origin);

		current_states->positions.push_back(starting_position);
		current_states->velocities.push_back(starting_speed);


	}
	void ParticleNode::Update(double delta_time) {

		total_time_elapsed += delta_time;

		while (start_time + dt < total_time_elapsed) {
			ParticleState& new_states = integrator->Integrate(*forces, *current_states, start_time, dt);
			UpdateStates(new_states);
			start_time += dt;

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


	}



	void ParticleNode::UpdateStates(ParticleState& new_states) {
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
	void ParticleNode::ResetSystem() {
		glm::vec3 starting_position(1, 0, 0);
		glm::vec3 starting_speed(0, 0, 0);
		current_states->positions.at(0) = starting_position;
		current_states->velocities.at(0) = starting_speed;
	}


}  // namespace GLOO
