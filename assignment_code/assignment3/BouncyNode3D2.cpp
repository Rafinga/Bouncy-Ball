#include "BouncyNode3D2.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp";
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"
#include <gloo/shaders/SimpleShader.hpp>
#include "glm/gtx/string_cast.hpp"

namespace GLOO {

	BouncyNode3D2::BouncyNode3D2(
		std::unique_ptr<IntegratorBase<BouncySystem2, ParticleState>> integrator,
		double dt,
		glm::vec3 center,
		float radius,
		unsigned int circumference_partition,
		glm::vec3 floor_normal,
		glm::vec3 floor_surface_point,
		glm::vec3& throw_speed
	) :
		forces(std::move(make_unique<BouncySystem2>())), integrator(std::move(integrator)), dt(dt), circumference_partition(circumference_partition),
		center(center), radius(radius),floor_normal(floor_normal),floor_surface_point(floor_surface_point), throw_speed(throw_speed)
	{
		current_states = make_unique<ParticleState>();
		shader = std::make_shared<PhongShader>();

		CreateBallSystem();
	}

	void BouncyNode3D2::AddParticle(const glm::vec3& starting_position, float mass ) {
		current_states->positions.push_back(starting_position);
		current_states->velocities.emplace_back(throw_speed);
		forces->AddParticle(mass);



		//std::unique_ptr<VertexObject> sphere_mesh = PrimitiveFactory::CreateSphere(0.035f, 25, 25);
		//std::unique_ptr<SceneNode> rendering_node = make_unique<SceneNode>();
		//rendering_node->CreateComponent<RenderingComponent>(std::move(sphere_mesh));
		//rendering_node->CreateComponent<ShadingComponent>(shader);
		//rendering_node->GetTransform().SetPosition(starting_position);
		//AddChild(std::move(rendering_node));
	}

	void BouncyNode3D2::Update(double delta_time) {

		total_time_elapsed += delta_time;

		if (InputManager::GetInstance().IsKeyPressed('S')) {
			if (prev_released2) {
				stop_pressed = !stop_pressed;
			}
			prev_released2 = false;
		}
		else if (InputManager::GetInstance().IsKeyReleased('S')) {
			prev_released2  = true;
		}
	

		while (start_time + dt < total_time_elapsed) {

			if (!stop_pressed) {
				ParticleState& new_states = integrator->Integrate(*forces, *current_states, start_time, dt);
				UpdateStates(new_states);
				UpdateWireframe();
			}
			start_time += dt;


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
	}




	void BouncyNode3D2::UpdateStates(ParticleState& new_states) {
		current_states->positions.clear();
		current_states->positions = new_states.positions;
		std::unique_ptr<PositionArray> copy_vertices = make_unique<PositionArray>(new_states.positions);
		//ball_vertices-> UpdatePositions(std::move(copy_vertices));
		current_states->velocities.clear();
		int child_index = 0;
		for (glm::vec3& new_velocity : new_states.velocities) {
			
			if (IsTouchingGround(current_states->positions.at(child_index))) {
				//new_velocity.y  = new_velocity.y >0? new_velocity.y : 0;
				new_velocity = RemovedTableVelocityComp(new_velocity);
			}
			current_states->velocities.push_back(new_velocity);

			//GetChild(child_index).GetTransform().SetPosition(current_states->positions.at(child_index));


			child_index++;
		}

	}





	void BouncyNode3D2::CreateBallSystem() {

		SetupSphereMesh();
		forces->SetFloorNormal(floor_normal);
		forces->SetFloorSurfacePoint(floor_surface_point);
		AddVertexObject();
		AddStructuralSprings();
		SetupBasePositions();








	}


	bool BouncyNode3D2::IsTouchingGround(glm::vec3 pos) const  {

		float delta = 0.00000001;
		glm::vec3 displacemnet_vector = pos - floor_surface_point;
		return glm::dot(displacemnet_vector, floor_normal) <= delta;
	}




	void BouncyNode3D2::AddCentralSprings(IndexArray& spring_indexes) {


		float central_stretch_constant = 0.02f;
		int center_index = current_states->positions.size() - 1;


		for (int i = 0; i < current_states->positions.size()-1; i++) {
			forces->AddSpring(center_index,i, central_stretch_constant, radius);
			//spring_indexes.push_back(i);
			//spring_indexes.push_back(center_index);
		}
	}

	void BouncyNode3D2::AddStructuralSprings() {


		float structural_strength = 50.0f;

		std::unique_ptr<IndexArray> spring_indexes = make_unique<IndexArray>();

		AddSpringsInSameCircles(structural_strength,*spring_indexes);
		AddDifferentCircleSprings(20*structural_strength, *spring_indexes);
		AddCentralSprings(*spring_indexes);
		AddDiagonalSprings(0.01*structural_strength,*spring_indexes);
		ball_vertices->UpdateIndices(std::move(spring_indexes));
	}


	void  BouncyNode3D2::AddSpringsInSameCircles(float structural_strength, IndexArray& spring_indexes) {
		int num_points_on_circle_slice = circumference_partition - 2;

		for (int i = 0; i <current_states->positions.size() - 3; i++) {
			int relative_index = i % num_points_on_circle_slice;

			if (relative_index == num_points_on_circle_slice / 2 - 1 || relative_index == num_points_on_circle_slice - 1) {
				continue;
			}

			float rest_length = glm::distance(current_states->positions.at(i), current_states->positions.at(i + 1));

			forces->AddSpring(i, i + 1, structural_strength, rest_length);

			spring_indexes.push_back(i);
			spring_indexes.push_back(i + 1);


		}





		int top_sphere_index = current_states->positions.size() - 3;
		AddCapSprings(top_sphere_index, structural_strength, 0,spring_indexes);

		int bottom_cap_index = top_sphere_index + 1;
		AddCapSprings(bottom_cap_index, structural_strength, num_points_on_circle_slice / 2 - 1,spring_indexes);


	}

	void BouncyNode3D2::AddDifferentCircleSprings(float structural_strength, IndexArray& spring_indexes) {
		int num_points_on_circle_slice = circumference_partition - 2;

		int total_num_points = current_states->positions.size()-3;


		for (int i = 0; i < num_points_on_circle_slice; i++) {

			int current_index = i;

			while (current_index + num_points_on_circle_slice < current_states->positions.size() - 3) {

				int conncting_index = current_index + num_points_on_circle_slice;



				const float original_length = glm::distance(current_states->positions.at(current_index), current_states->positions.at(conncting_index));


				forces->AddSpring(current_index, conncting_index, structural_strength, original_length);

				spring_indexes.push_back(current_index);
				spring_indexes.push_back(conncting_index);

				current_index += num_points_on_circle_slice;



			}
			int conncting_index = num_points_on_circle_slice - 1 - i;
			const float original_length = glm::distance(current_states->positions.at(current_index), current_states->positions.at(conncting_index));
			
			forces->AddSpring(current_index, conncting_index, structural_strength, original_length);
			spring_indexes.push_back(current_index);
			spring_indexes.push_back(conncting_index);


		}
	}

	void BouncyNode3D2::AddDiagonalSprings(float structural_strength, IndexArray& spring_indexes) {
		int num_points_on_circle_slice = circumference_partition - 2;

		int total_num_points = current_states->positions.size() - 3;


		for (int i = 0; i < total_num_points; ++i) {
			if (i % num_points_on_circle_slice != circumference_partition / 2 - 2 && (i % num_points_on_circle_slice) != num_points_on_circle_slice - 1) {
				int bottom_diagonal_i = i + num_points_on_circle_slice + 1;
				if (bottom_diagonal_i >= total_num_points) {
					bottom_diagonal_i = num_points_on_circle_slice - 1 - (bottom_diagonal_i % num_points_on_circle_slice);
				}
				float bottom_rest_length = glm::distance(current_states->positions[bottom_diagonal_i], current_states->positions[i]);
				forces->AddSpring(i, bottom_diagonal_i,spring_constant, bottom_rest_length);  

				spring_indexes.emplace_back(i);
				spring_indexes.emplace_back(bottom_diagonal_i);
			}

			if (i % num_points_on_circle_slice != 0 && (i % num_points_on_circle_slice) != circumference_partition / 2 - 1) {
				int top_diagonal_i = i + num_points_on_circle_slice - 1;
				if (top_diagonal_i >= total_num_points) {
					top_diagonal_i = num_points_on_circle_slice - 1 - (top_diagonal_i % num_points_on_circle_slice);
				}
				float top_rest_length = glm::length(current_states->positions[top_diagonal_i] - current_states->positions[i]);
				forces->AddSpring(i, top_diagonal_i, structural_strength,top_rest_length); 

				spring_indexes.emplace_back(i);
				spring_indexes.emplace_back(top_diagonal_i);
			}
		}
	}

	void BouncyNode3D2::AddCapSprings(int cap_index, float structural_strength, int left_connected_node_index,IndexArray& spring_indexes) {
		int num_points_on_circle_slice = circumference_partition - 2;
		//the  GetChildrenCount() - 3 is when the 2 cap indexes and the center index start

		int right_connected_node_index = num_points_on_circle_slice - 1 - left_connected_node_index;
		while (left_connected_node_index < current_states->positions.size() - 3) {
			float rest_length = glm::distance(current_states->positions.at(cap_index), current_states->positions.at(left_connected_node_index));
			forces->AddSpring(left_connected_node_index, cap_index, structural_strength, rest_length);


			float right_rest_length = glm::distance(current_states->positions.at(cap_index), current_states->positions.at(right_connected_node_index));
			forces->AddSpring(right_connected_node_index, cap_index, structural_strength, rest_length);


			spring_indexes.push_back(left_connected_node_index);
			spring_indexes.push_back(cap_index);
			spring_indexes.push_back(right_connected_node_index);
			spring_indexes.push_back(cap_index);

			left_connected_node_index += num_points_on_circle_slice;
			right_connected_node_index += num_points_on_circle_slice;
		}

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

		AddParticle(center, 1.0f);

	}

	

	void BouncyNode3D2::ResetSystem() {

		
		std::unique_ptr<ParticleState> reset_states = make_unique<ParticleState>();

		std::unique_ptr<PositionArray> reset_pos = make_unique<PositionArray>(*base_positions);
		std::unique_ptr<PositionArray> reset_vels = make_unique<PositionArray>(*base_velocities);



		reset_states->positions = *reset_pos;
		reset_states->velocities = *reset_vels;


		UpdateStates(*reset_states);


	}


	void BouncyNode3D2::AddVertexObject() {

		ball_vertices = std::make_shared<VertexObject>();
		std::unique_ptr<PositionArray> positions = make_unique<PositionArray>();
		std::unique_ptr<NormalArray> normals = make_unique<NormalArray>();
		std::unique_ptr<IndexArray> indexes = make_unique<IndexArray>();
		for (glm::vec3 position : current_states->positions) {
			positions->push_back(position);
		}


		ball_vertices->UpdatePositions(std::move(positions));
		ball_vertices->UpdateNormals(std::move(normals));
		ball_vertices->UpdateIndices(std::move(indexes));

		std::unique_ptr<SceneNode> extra = make_unique<SceneNode>();
		extra->CreateComponent<RenderingComponent>(ball_vertices);

		extra->GetComponentPtr<RenderingComponent>()->SetDrawMode(DrawMode::Lines);
		extra->CreateComponent<ShadingComponent>(std::move(make_unique<SimpleShader>()));



		//extra->SetActive(false);

		AddChild(std::move(extra));

	}


	void BouncyNode3D2::UpdateWireframe() {
		
		std::unique_ptr<PositionArray> positions = make_unique<PositionArray>();
		for (glm::vec3 position : current_states->positions) {
			positions->push_back(position);
		}
		ball_vertices->UpdatePositions(std::move(positions));

	}

	glm::vec3 BouncyNode3D2::RemovedTableVelocityComp(glm::vec3& velocity) {
		float  projected_vector_length = glm::dot(velocity, floor_normal);

		if (projected_vector_length >= 0) {
			return velocity;
		}
		return velocity - projected_vector_length * floor_normal;
	}
	void BouncyNode3D2::SetupBasePositions() {
		base_positions = make_unique<PositionArray>();
		base_velocities = make_unique<PositionArray>();
		int child_index = 0;
		for (glm::vec3& pos : current_states->positions) {
			base_positions->push_back(pos);
			base_velocities->push_back(current_states->velocities.at(child_index));
			child_index++;
		}
	}


}  // namespace GLOO
