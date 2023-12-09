#include "BouncyNode3D2.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp";
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"
#include <gloo/shaders/SimpleShader.hpp>

namespace GLOO {

	BouncyNode3D2::BouncyNode3D2(
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
		shader = std::make_shared<PhongShader>();
		floor_normal = glm::vec3(0, 1, 0);
		floor_surface_point = glm::vec3(0, -1, 0);

		CreateBallSystem();
	}

	void BouncyNode3D2::AddParticle(const glm::vec3& starting_position, float mass ) {
		current_states->positions.push_back(starting_position);
		current_states->velocities.emplace_back(throw_speed);
		forces->AddParticle(mass);



		std::unique_ptr<VertexObject> sphere_mesh = PrimitiveFactory::CreateSphere(0.035f, 25, 25);
		std::unique_ptr<SceneNode> rendering_node = make_unique<SceneNode>();
		rendering_node->CreateComponent<RenderingComponent>(std::move(sphere_mesh));
		rendering_node->CreateComponent<ShadingComponent>(shader);
		rendering_node->GetTransform().SetPosition(starting_position);
		AddChild(std::move(rendering_node));
	}

	void BouncyNode3D2::Update(double delta_time) {

		total_time_elapsed += delta_time;

		while (start_time + dt < total_time_elapsed) {

			if (!stop_pressed) {
				ParticleState& new_states = integrator->Integrate(*forces, *current_states, start_time, dt);
				UpdateStates(new_states);
				start_time += dt;
			}


		//	if (InputManager::GetInstance().IsKeyPressed('R')) {
		//		if (prev_released) {
		//			ResetSystem();
		//		}
		//		prev_released = false;
		//	}
		//	else if (InputManager::GetInstance().IsKeyReleased('R')) {
		//		prev_released = true;

			}
		//}
	}




	void BouncyNode3D2::UpdateStates(ParticleState& new_states) {
		current_states->positions.clear();
		current_states->positions = new_states.positions;
		std::unique_ptr<PositionArray> copy_vertices = make_unique<PositionArray>(new_states.positions);
		//ball_vertices-> UpdatePositions(std::move(copy_vertices));
		current_states->velocities.clear();
		int child_index = 0;
		for (glm::vec3& new_velocity : new_states.velocities) {
			
			//if (IsTouchingGround(current_states->positions.at(child_index))) {
			//	new_velocity.y  = new_velocity.y >0? new_velocity.y : -new_velocity.y;
			//}
			current_states->velocities.push_back(new_velocity);

			GetChild(child_index).GetTransform().SetPosition(current_states->positions.at(child_index));


			child_index++;
		}

	}





	void BouncyNode3D2::CreateBallSystem() {

		SetupSphereMesh();
		forces->SetFloorNormal(floor_normal);
		forces->SetFloorSurfacePoint(floor_surface_point);



	}


	bool BouncyNode3D2::IsTouchingGround(glm::vec3 pos) const  {

		float delta = 0.00000001;
		glm::vec3 displacemnet_vector = pos - floor_surface_point;
		return glm::dot(displacemnet_vector, floor_normal) <= delta;
	}




	void BouncyNode3D2::AddCentralSprings() {


		float central_stretch_constant = 0.002;

		for (int i = 0; i < circumference_partition; i++) {
			forces->AddSpring(circumference_partition,i, central_stretch_constant, radius);
		}
	}

	void BouncyNode3D2::AddStructuralSprings() {


		float structural_strength = 500.0f;

		for (int i = 0; i < circumference_partition; i++) {
			int parent_index = i > 0 ? (i - 1) : circumference_partition - 1;
			float rest_length = glm::distance(GetChild(i).GetTransform().GetPosition(), GetChild(parent_index).GetTransform().GetPosition());

			forces->AddSpring(i, parent_index, 50000.0f, rest_length);
		}


	}

	void BouncyNode3D2::AddFlexSprings(){


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







	void BouncyNode3D2::SetupSphereMesh() {
		float rotation_radians = 2 * 3.14159265 / circumference_partition;
		float cos_angle = cos(rotation_radians);
		float sin_angle = sin(rotation_radians);
		glm::mat3 circle_rotation_matrix(cos_angle, sin_angle, 0,
			-sin_angle, cos_angle, 0,
			0, 0, 1); // z-coordinate stays fixed, this is for rotation in the xy-plane.

		glm::mat3 sphere_rotation_matrix(cos_angle, 0, sin_angle,
			0, 1, 0,
			-sin_angle, 0, cos_angle); // z-coordinate stays fixed, this is for rotation in the xy-plane.

		PositionArray untrans_pos_array;




		glm::vec3 current_point(0, radius, 0);  // Construct initial circle
		for (int i = 0; i < circumference_partition - 1; ++i) {
			glm::vec3 rotated_point = circle_rotation_matrix * current_point ;
			if (i != circumference_partition / 2 - 1) {

				untrans_pos_array.push_back(rotated_point);
			}
			current_point = rotated_point;
		}

		for (int i = 0; i < circumference_partition / 2 - 1; ++i) {  // Iterate over the number of rotations
			int current_size = untrans_pos_array.size();
			for (int j = current_size - (circumference_partition - 2); j < current_size; ++j) {  // Iterate over the last n-2 indices in the state_.positions array

				glm::vec3 rotated_point = sphere_rotation_matrix * untrans_pos_array[j];
				untrans_pos_array.push_back(rotated_point);
			}
		}


		glm::vec3 top_point(0, radius , 0);
		glm::vec3 bottom_point(0, -radius , 0);


		untrans_pos_array.push_back(top_point);
		untrans_pos_array.push_back(bottom_point);




		for (glm::vec3& pos : untrans_pos_array) {
			AddParticle(pos + center, 1.0f);
		}

	}

	

	void BouncyNode3D2::ResetSystem() {

		
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
