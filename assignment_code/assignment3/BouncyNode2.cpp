#include "BouncyNode2.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp";
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"
#include <gloo/shaders/SimpleShader.hpp>

namespace GLOO {

	BouncyNode2::BouncyNode2(
		std::unique_ptr<IntegratorBase<BouncySystem2, ParticleState>> integrator,
		double dt,
		glm::vec3 center,
		float radius,
		unsigned int circumference_partition,
		glm::vec3& throw_speed
	) :
		forces(std::move(make_unique<BouncySystem2>())), integrator(std::move(integrator)), dt(dt), circumference_partition(circumference_partition),
		center(center), radius(radius), throw_speed(throw_speed)
	{
		current_states = make_unique<ParticleState>();
		CreateBallSystem();
	}

	void BouncyNode2::AddParticle(const glm::vec3& starting_position, float mass ) {
		std::unique_ptr<VertexObject> particle = PrimitiveFactory::CreateSphere(0.03, 10, 10);
		std::unique_ptr<ShaderProgram> phongShader = make_unique<SimpleShader>();
		std::unique_ptr<SceneNode> fixed_node = make_unique<SceneNode>();
		fixed_node->CreateComponent<RenderingComponent>(std::move(particle));
		fixed_node->CreateComponent<ShadingComponent>(std::move(phongShader));
		fixed_node->GetTransform().SetPosition(starting_position);
		current_states->positions.push_back(starting_position);
		current_states->velocities.push_back(throw_speed);
		forces->AddParticle(mass);

		AddChild(std::move(fixed_node));

	}

	void BouncyNode2::Update(double delta_time) {

		total_time_elapsed += delta_time;

		while (start_time + dt < total_time_elapsed) {

			if (!stop_pressed) {
				ParticleState& new_states = integrator->Integrate(*forces, *current_states, start_time, dt);
				UpdateStates(new_states);
				start_time += dt;
				UpdateSystem();
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


			if (InputManager::GetInstance().IsKeyPressed('S')) {
				stop_pressed = true;
			}

			if (InputManager::GetInstance().IsKeyReleased('S')) {
				stop_pressed = false;

			}

		}
	}




	void BouncyNode2::UpdateStates(ParticleState& new_states) {
		current_states->positions.clear();
		int child_index = 0;

		std::vector<bool> touching_ground;
		for (const glm::vec3& new_position : new_states.positions) {

			current_states->positions.push_back(new_position);


			GetChild(child_index).GetTransform().SetPosition(new_position);
			child_index++;

			touching_ground.push_back(IsTouchingGround(new_position));
		}
		current_states->velocities.clear();
		child_index = 0;
		for (glm::vec3& new_velocity : new_states.velocities) {
			
			//if (touching_ground.at(child_index)) {
			//	new_velocity.y = new_velocity.y > 0 ? new_velocity.y : 0;

			//}
			if (touching_ground.at(child_index)) {

				//new_velocity = glm::vec3(0);
				new_velocity.y  = new_velocity.y >0? new_velocity.y : -new_velocity.y;



			}
			current_states->velocities.push_back(new_velocity);
			child_index++;
		}

	}





	void BouncyNode2::CreateBallSystem() {
		ball_vertices = std::make_shared<VertexObject>();
		std::unique_ptr<PositionArray> new_positions = make_unique<PositionArray>();
		std::unique_ptr<IndexArray> indexes = make_unique<IndexArray>();


		float rotation_radians = 2 * 3.14159265 /circumference_partition;

		for (int i = 0; i < circumference_partition; i++) {

			float current_angle = i * rotation_radians;

			glm::vec3 point_position = center + glm::vec3(radius*std::cos(current_angle), radius*std::sin(current_angle), 1);

			AddParticle(point_position);
		}
		AddParticle(center+glm::vec3(0,0,1), circumference_partition);
		forces->SetFloorNormal(glm::vec3(0, 1, 0));
		forces->SetFloorSurfacePoint(glm::vec3(0, -1, 0));

		AddStructuralSprings();
		AddFlexSprings();
		AddCentralSprings();





	}

	void BouncyNode2::UpdateSystem() {
		std::unique_ptr<PositionArray> new_positions = make_unique<PositionArray>();
		for (glm::vec3 position : current_states->positions) {
			new_positions->push_back(position);
		}
	}

	bool BouncyNode2::IsTouchingGround(glm::vec3 pos) const  {

		float delta = 0.00000001;
		glm::vec3 floor_normal(0, 1, 0);
		glm::vec3 floor_surface_point(0, -1, 0);
		glm::vec3 displacemnet_vector = pos - floor_surface_point;
		return glm::dot(displacemnet_vector, floor_normal) <= delta;
	}




	void BouncyNode2::AddCentralSprings() {


		float central_stretch_constant = 0.002;

		for (int i = 0; i < circumference_partition; i++) {
			forces->AddSpring(circumference_partition,i, central_stretch_constant, radius);
		}
	}

	void BouncyNode2::AddStructuralSprings() {


		float structural_strength = 500.0f;

		for (int i = 0; i < circumference_partition; i++) {
			int parent_index = i > 0 ? (i - 1) : circumference_partition - 1;
			float rest_length = glm::distance(GetChild(i).GetTransform().GetPosition(), GetChild(parent_index).GetTransform().GetPosition());

			forces->AddSpring(i, parent_index, 50000.0f, rest_length);
		}


	}

	void BouncyNode2::AddFlexSprings(){


		float flex_strength = 500.0f;

		for (int i = 2; i < circumference_partition; i++) {
			int parent_index = (i - 2) % circumference_partition;
			float rest_length = glm::distance(GetChild(i).GetTransform().GetPosition(), GetChild(parent_index).GetTransform().GetPosition());

			forces->AddSpring(i, parent_index, flex_strength, rest_length);
		}


		int parent_index = circumference_partition-1;
		float rest_length = glm::distance(GetChild(1).GetTransform().GetPosition(), GetChild(parent_index).GetTransform().GetPosition());

		forces->AddSpring(1, parent_index, flex_strength, rest_length);


		parent_index = circumference_partition - 2;
		rest_length = glm::distance(GetChild(2).GetTransform().GetPosition(), GetChild(parent_index).GetTransform().GetPosition());

		forces->AddSpring(2, parent_index, flex_strength, rest_length);
	}









	

	void BouncyNode2::ResetSystem() {

		
		std::unique_ptr<ParticleState> reset_states = make_unique<ParticleState>();

		std::unique_ptr<PositionArray> reset_pos = make_unique<PositionArray>();


		float rotation_radians = 2 * 3.14159265 / circumference_partition;

		for (int i = 0; i < circumference_partition; i++) {

			float current_angle = i * rotation_radians;

			glm::vec3 point_position = center + glm::vec3(radius * std::cos(current_angle), radius * std::sin(current_angle), 1);

			reset_states->positions.push_back(point_position);
			reset_states->velocities.push_back(throw_speed);
		}

		reset_states->positions.push_back(center + glm::vec3(0, 0, 1));
		reset_states->velocities.push_back(throw_speed);


		UpdateStates(*reset_states);


	}



}  // namespace GLOO
