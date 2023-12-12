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
		simple_shader = std::make_shared<SimpleShader>();

		CreateBallSystem();
	}

	void BouncyNode3D2::AddParticle(const glm::vec3& starting_position, float mass ) {
		current_states->positions.push_back(starting_position);
		current_states->velocities.emplace_back(throw_speed);
		forces->AddParticle(mass);
	}

	void BouncyNode3D2::Update(double delta_time) {

		// Keybinds:
		//		'R' -> Reset to initial state
		//		'S' -> Stop (freeze) time
		//		'T' -> Toggles triangles

		//		'H' -> Toggles horizontal structural springs
		//		'V' -> Toggles vertical structural springs
		//		'P' -> Toggles pressure springs
		//		'B' -> Toggles bottom-diagonal structural springs
		//		'O' -> Toggles top-diagonal structural springs
		//		'W' -> Toggles entire wireframe of springs


		// Keybind logic
	if (InputManager::GetInstance().IsKeyPressed('T')) {
			SceneNode& triangles_node = GetChild(0);

			if (t_prev_released_) {
				if (triangles_enabled_) {
					triangles_node.SetActive(false);
					triangles_enabled_ = false;
				}
				else {
					triangles_node.SetActive(true);
					triangles_enabled_ = true;
				}
			}
			t_prev_released_ = false;
		}
		else if (InputManager::GetInstance().IsKeyReleased('T')) {
			t_prev_released_ = true;
		}

		if (InputManager::GetInstance().IsKeyPressed('H')) {
			SceneNode& horizontal_node = GetChild(1);

			if (h_prev_released_ && wireframe_enabled_) {
				if (horizontal_structurals_active_) {
					horizontal_node.SetActive(false);
					horizontal_structurals_active_ = false;
				}
				else {
					horizontal_node.SetActive(true);
					horizontal_structurals_active_ = true;
				}
			}
			h_prev_released_ = false;
		}
		else if (InputManager::GetInstance().IsKeyReleased('H')) {
			h_prev_released_ = true;
		}

		if (InputManager::GetInstance().IsKeyPressed('V')) {
			SceneNode& vertical_node = GetChild(2);

			if (v_prev_released_ && wireframe_enabled_) {
				if (vertical_structurals_active_) {
					vertical_node.SetActive(false);
					vertical_structurals_active_ = false;
				}
				else {
					vertical_node.SetActive(true);
					vertical_structurals_active_ = true;
				}
			}
			v_prev_released_ = false;
		}
		else if (InputManager::GetInstance().IsKeyReleased('V')) {
			v_prev_released_ = true;
		}

		if (InputManager::GetInstance().IsKeyPressed('P')) {
			SceneNode& pressure_node = GetChild(3);

			if (p_prev_released_ && wireframe_enabled_) {
				if (pressure_active_) {
					pressure_node.SetActive(false);
					pressure_active_ = false;
				}
				else {
					pressure_node.SetActive(true);
					pressure_active_ = true;
				}
			}
			p_prev_released_ = false;
		}
		else if (InputManager::GetInstance().IsKeyReleased('B')) {
			p_prev_released_ = true;
		}

		if (InputManager::GetInstance().IsKeyPressed('B')) {
			SceneNode& bottom_diagonal_node = GetChild(4);

			if (b_prev_released_ && wireframe_enabled_) {
				if (bottom_diagonals_active_) {
					bottom_diagonal_node.SetActive(false);
					bottom_diagonals_active_ = false;
				}
				else {
					bottom_diagonal_node.SetActive(true);
					bottom_diagonals_active_ = true;
				}
			}
			b_prev_released_ = false;
		}
		else if (InputManager::GetInstance().IsKeyReleased('B')) {
			b_prev_released_ = true;
		}

		if (InputManager::GetInstance().IsKeyPressed('O')) {
			SceneNode& top_diagonal_node = GetChild(5);

			if (o_prev_released_ && wireframe_enabled_) {
				if (top_diagonals_active_) {
					top_diagonal_node.SetActive(false);
					top_diagonals_active_ = false;
				}
				else {
					top_diagonal_node.SetActive(true);
					top_diagonals_active_ = true;
				}
			}
			o_prev_released_ = false;
		}
		else if (InputManager::GetInstance().IsKeyReleased('O')) {
			o_prev_released_ = true;
		}

		if (InputManager::GetInstance().IsKeyPressed('W')) {
			SceneNode& horizontal_node = GetChild(1);
			SceneNode& vertical_node = GetChild(2);
			SceneNode& pressure_node = GetChild(3);
			SceneNode& bottom_diagonal_node = GetChild(4);
			SceneNode& top_diagonal_node = GetChild(5);

			if (w_prev_released_) {
				if (wireframe_enabled_) {
					horizontal_node.SetActive(false);
					vertical_node.SetActive(false);
					pressure_node.SetActive(false);
					bottom_diagonal_node.SetActive(false);
					top_diagonal_node.SetActive(false);
					wireframe_enabled_ = false;
				}
				else {
					horizontal_node.SetActive(horizontal_structurals_active_);
					vertical_node.SetActive(vertical_structurals_active_);
					pressure_node.SetActive(pressure_active_);
					bottom_diagonal_node.SetActive(bottom_diagonals_active_);
					top_diagonal_node.SetActive(top_diagonals_active_);
					wireframe_enabled_ = true;
				}
			}
			w_prev_released_ = false;
		}
		else if (InputManager::GetInstance().IsKeyReleased('W')) {
			w_prev_released_ = true;
		}




















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
				UpdatePositions();
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
		current_states->velocities.clear();
		int child_index = 0;
		for (glm::vec3& new_velocity : new_states.velocities) {
			
			if (IsTouchingGround(current_states->positions.at(child_index))) {
				new_velocity = RemovedTableVelocityComp(new_velocity);
			}
			current_states->velocities.push_back(new_velocity);


			child_index++;
		}

	}





	void BouncyNode3D2::CreateBallSystem() {

		SetupSphereMesh();
		forces->SetFloorNormal(floor_normal);
		forces->SetFloorSurfacePoint(floor_surface_point);
		SetupBasePositions();
		CreateVertexNodes();
		AddVertexObject();
		AddStructuralSprings();
		num_children_ = GetChildrenCount();


	}


	bool BouncyNode3D2::IsTouchingGround(glm::vec3 pos) const  {

		float delta = 0.00000001;
		glm::vec3 displacemnet_vector = pos - floor_surface_point;
		return glm::dot(displacemnet_vector, floor_normal) <= delta;
	}




	void BouncyNode3D2::AddCentralSprings(IndexArray& spring_indexes) {
		
		std::unique_ptr<IndexArray> pressure_indexes = make_unique<IndexArray>(); 

		float central_stretch_constant = 0.002f;
		int center_index = current_states->positions.size() - 1;


		for (int i = 0; i < current_states->positions.size()-1; i++) {
			forces->AddSpring(center_index,i, central_stretch_constant, radius);
			pressure_indexes->emplace_back(i);
			pressure_indexes->emplace_back(center_index);
		}
		pressure_vertices->UpdateIndices(std::move(pressure_indexes));
	}

	void BouncyNode3D2::AddStructuralSprings() {


		float structural_strength = 50.0f;

		std::unique_ptr<IndexArray> spring_indexes = make_unique<IndexArray>();

		AddSpringsInSameCircles(structural_strength,*spring_indexes);
		AddDifferentCircleSprings(20*structural_strength, *spring_indexes);
		AddCentralSprings(*spring_indexes);
		AddDiagonalSprings(0.01*structural_strength,*spring_indexes);
	}


	void  BouncyNode3D2::AddSpringsInSameCircles(float structural_strength, IndexArray& spring_indexes) {
		std::unique_ptr<IndexArray> same_circle_indexes = make_unique<IndexArray>();

		int num_points_on_circle_slice = circumference_partition - 2;

		for (int i = 0; i <current_states->positions.size() - 3; i++) {
			int relative_index = i % num_points_on_circle_slice;

			if (relative_index == num_points_on_circle_slice / 2 - 1 || relative_index == num_points_on_circle_slice - 1) {
				continue;
			}

			float rest_length = glm::distance(current_states->positions.at(i), current_states->positions.at(i + 1));

			forces->AddSpring(i, i + 1, structural_strength, rest_length);

			same_circle_indexes->push_back(i);
			same_circle_indexes->push_back(i + 1);

		}
		int top_sphere_index = current_states->positions.size() - 3;
		AddCapSprings(top_sphere_index, structural_strength, 0,*same_circle_indexes);

		int bottom_cap_index = top_sphere_index + 1;
		AddCapSprings(bottom_cap_index, structural_strength, num_points_on_circle_slice / 2 - 1,*same_circle_indexes);

		same_circle_vertices->UpdateIndices(std::move(same_circle_indexes));


	}

	void BouncyNode3D2::AddDifferentCircleSprings(float structural_strength, IndexArray& spring_indexes) {

		std::unique_ptr<IndexArray> different_circle_indexes = make_unique<IndexArray>();

		int num_points_on_circle_slice = circumference_partition - 2;

		int total_num_points = current_states->positions.size()-3;


		for (int i = 0; i < num_points_on_circle_slice; i++) {

			int current_index = i;

			while (current_index + num_points_on_circle_slice < current_states->positions.size() - 3) {

				int conncting_index = current_index + num_points_on_circle_slice;



				const float original_length = glm::distance(current_states->positions.at(current_index), current_states->positions.at(conncting_index));


				forces->AddSpring(current_index, conncting_index, structural_strength, original_length);

				different_circle_indexes->push_back(current_index);
				different_circle_indexes->push_back(conncting_index);

				current_index += num_points_on_circle_slice;



			}
			int conncting_index = num_points_on_circle_slice - 1 - i;
			const float original_length = glm::distance(current_states->positions.at(current_index), current_states->positions.at(conncting_index));
			
			forces->AddSpring(current_index, conncting_index, structural_strength, original_length);
			different_circle_indexes->push_back(current_index);
			different_circle_indexes->push_back(conncting_index);
		}

		different_circle_vertices->UpdateIndices(std::move(different_circle_indexes));
	}

	void BouncyNode3D2::AddDiagonalSprings(float structural_strength, IndexArray& spring_indexes) {
		std::unique_ptr<IndexArray> upper_diagonal_indexes = make_unique<IndexArray>();
		std::unique_ptr<IndexArray> lower_diagonal_indexes = make_unique<IndexArray>();

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

				upper_diagonal_indexes->emplace_back(i);
				upper_diagonal_indexes->emplace_back(bottom_diagonal_i);
			}

			if (i % num_points_on_circle_slice != 0 && (i % num_points_on_circle_slice) != circumference_partition / 2 - 1) {
				int top_diagonal_i = i + num_points_on_circle_slice - 1;
				if (top_diagonal_i >= total_num_points) {
					top_diagonal_i = num_points_on_circle_slice - 1 - (top_diagonal_i % num_points_on_circle_slice);
				}
				float top_rest_length = glm::length(current_states->positions[top_diagonal_i] - current_states->positions[i]);
				forces->AddSpring(i, top_diagonal_i, structural_strength,top_rest_length); 

				lower_diagonal_indexes->emplace_back(i);
				lower_diagonal_indexes->emplace_back(top_diagonal_i);
			}
		}
		upper_diagonal_vertices->UpdateIndices(std::move(upper_diagonal_indexes));
		lower_diagonal_vertices->UpdateIndices(std::move(lower_diagonal_indexes));


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

		AddChild(std::move(extra));

	}

	void BouncyNode3D2::CreateVertexNodes() {
		CreateTriangleVertexNode();
		CreateDifferentCircleVertexNode();
		CreateSameCircleVertexNode();
		CreatePressureVertexNode();
		CreateLowerDiagonalVertexNode();
		CreateUpperDiagonalVertexNode();
	}


	void BouncyNode3D2::CreateTriangleVertexNode() {
		std::unique_ptr<PositionArray>  starting_positions = make_unique<PositionArray>(*base_positions);
		std::unique_ptr<NormalArray> starting_normals = make_unique<NormalArray>();
		std::unique_ptr<IndexArray> starting_indexes = make_unique<IndexArray>();
		int num_points_on_circle = circumference_partition - 2;

		triangle_vertices = std::make_shared<VertexObject>();



		// Logic for calculating all the traingles and their indices.
		for (int i = 0; i < current_states->positions.size() - 3; ++i) {
			if (i % num_points_on_circle != circumference_partition / 2 - 2 && (i % num_points_on_circle) != num_points_on_circle - 1) {
				int bottom_diagonal_i = i + num_points_on_circle + 1;
				int top_i;
				if (bottom_diagonal_i >= current_states->positions.size() - 3) {
					bottom_diagonal_i = num_points_on_circle - 1 - (bottom_diagonal_i % num_points_on_circle);
					top_i = bottom_diagonal_i + 1;
				}
				else {
					top_i = bottom_diagonal_i - 1;
				}
				int bottom_i = i + 1;

				starting_indexes->emplace_back(i);  // Bottom triangle in a given spherical square
				starting_indexes->emplace_back(bottom_diagonal_i);
				starting_indexes->emplace_back(i + 1);

				starting_indexes->emplace_back(i);  // Top triangle in a given spherical square
				starting_indexes->emplace_back(bottom_diagonal_i);
				starting_indexes->emplace_back(top_i);
			}
		}

		// Connect the triangles for the top-hat.
		int top_idx = current_states->positions.size() - 3;
		for (int triangle_idx = 0; triangle_idx < current_states->positions.size() - 3; triangle_idx += num_points_on_circle) {
			int neighbor_idx = triangle_idx + num_points_on_circle;

			int symmetric_triangle_idx = triangle_idx + num_points_on_circle - 1;  // Get the index for the point across from the bottom-point
			int symmetric_neighbor_idx = symmetric_triangle_idx + num_points_on_circle;
			// Handle the wrap-around for the last vertical-circle connecting to the first vertical-circle.
			if (symmetric_neighbor_idx > current_states->positions.size() - 3) {
				neighbor_idx = num_points_on_circle - 1;
				symmetric_neighbor_idx = 0;
			}

			// Triangle connecting top point and two adjacent points on the top-most level of the sphere.
			starting_indexes->emplace_back(top_idx);
			starting_indexes->emplace_back(triangle_idx);
			starting_indexes->emplace_back(neighbor_idx);

			// Calculate the triangle symmetric about the top point.
			starting_indexes->emplace_back(top_idx);
			starting_indexes->emplace_back(symmetric_triangle_idx);
			starting_indexes->emplace_back(symmetric_neighbor_idx);
		}

		// Connect the triangles for the bottom-hat.
		int bottom_idx = current_states->positions.size() - 2;
		for (int triangle_idx = circumference_partition / 2 - 2; triangle_idx < current_states->positions.size() - 3; triangle_idx += num_points_on_circle) {
			int neighbor_idx = triangle_idx + num_points_on_circle;

			int symmetric_triangle_idx = triangle_idx + 1;  // Get the index for the point across from the bottom-point
			int symmetric_neighbor_idx = symmetric_triangle_idx + num_points_on_circle;
			// Handle the wrap-around for the last vertical-circle connecting to the first vertical-circle.
			if (symmetric_neighbor_idx > current_states->positions.size() - 3) {
				neighbor_idx = circumference_partition / 2 - 1;
				symmetric_neighbor_idx = circumference_partition / 2 - 2;
			}

			// Triangle connecting bottom point and two adjacent points on the bottom-most level of the sphere.
			starting_indexes->emplace_back(bottom_idx);
			starting_indexes->emplace_back(triangle_idx);
			starting_indexes->emplace_back(neighbor_idx);

			// Calculate the triangle symmetric about the top point.
			starting_indexes->emplace_back(bottom_idx);
			starting_indexes->emplace_back(symmetric_triangle_idx);
			starting_indexes->emplace_back(symmetric_neighbor_idx);
		}


		for (glm::vec3& pos : *starting_positions) {

			glm::vec3 normal_vec = glm::normalize(pos - center);
			starting_normals->push_back(normal_vec);
		}



		triangle_vertices->UpdatePositions(std::move(starting_positions));
		triangle_vertices->UpdateNormals(std::move(starting_normals));
		triangle_vertices->UpdateIndices(std::move(starting_indexes));

		std::unique_ptr<SceneNode> mesh_node = make_unique<SceneNode>();

		mesh_node->CreateComponent<RenderingComponent>(triangle_vertices);
		mesh_node->CreateComponent<ShadingComponent>(std::move(make_unique<PhongShader>()));
		AddChild(std::move(mesh_node));

	}
	void BouncyNode3D2::CreateSameCircleVertexNode() {
		std::unique_ptr<PositionArray>  starting_positions = make_unique<PositionArray>(*base_positions);
		same_circle_vertices = std::make_shared<VertexObject>();
		same_circle_vertices->UpdatePositions(std::move(starting_positions));
		std::unique_ptr<SceneNode> mesh_node =  make_unique<SceneNode>();
		mesh_node->CreateComponent<RenderingComponent>(same_circle_vertices);
		mesh_node->GetComponentPtr<RenderingComponent>()->SetDrawMode(DrawMode::Lines);
		mesh_node->CreateComponent<ShadingComponent>(simple_shader);
		AddChild(std::move(mesh_node));
	}
	void BouncyNode3D2::CreateDifferentCircleVertexNode() {
		std::unique_ptr<PositionArray>  starting_positions = make_unique<PositionArray>(*base_positions);
		different_circle_vertices = std::make_shared<VertexObject>();
		different_circle_vertices->UpdatePositions(std::move(starting_positions));
		std::unique_ptr<SceneNode> mesh_node = make_unique<SceneNode>();
		mesh_node->CreateComponent<RenderingComponent>(different_circle_vertices);
		mesh_node->GetComponentPtr<RenderingComponent>()->SetDrawMode(DrawMode::Lines);
		mesh_node->CreateComponent<ShadingComponent>(simple_shader);
		AddChild(std::move(mesh_node));
	}
	void BouncyNode3D2::CreatePressureVertexNode() {
		std::unique_ptr<PositionArray>  starting_positions = make_unique<PositionArray>(*base_positions);
		pressure_vertices = std::make_shared<VertexObject>();
		pressure_vertices->UpdatePositions(std::move(starting_positions));
		std::unique_ptr<SceneNode> mesh_node = make_unique<SceneNode>();
		mesh_node->CreateComponent<RenderingComponent>(pressure_vertices);
		mesh_node->GetComponentPtr<RenderingComponent>()->SetDrawMode(DrawMode::Lines);
		mesh_node->CreateComponent<ShadingComponent>(simple_shader);
		AddChild(std::move(mesh_node));
	}
	void BouncyNode3D2::CreateLowerDiagonalVertexNode() {
		std::unique_ptr<PositionArray>  starting_positions = make_unique<PositionArray>(*base_positions);
		lower_diagonal_vertices = std::make_shared<VertexObject>();
		lower_diagonal_vertices->UpdatePositions(std::move(starting_positions));
		std::unique_ptr<SceneNode> mesh_node = make_unique<SceneNode>();
		mesh_node->CreateComponent<RenderingComponent>(lower_diagonal_vertices);
		mesh_node->GetComponentPtr<RenderingComponent>()->SetDrawMode(DrawMode::Lines);
		mesh_node->CreateComponent<ShadingComponent>(simple_shader);
		AddChild(std::move(mesh_node));
	}
	void BouncyNode3D2::CreateUpperDiagonalVertexNode() {
		std::unique_ptr<PositionArray>  starting_positions = make_unique<PositionArray>(*base_positions);
		upper_diagonal_vertices = std::make_shared<VertexObject>();
		upper_diagonal_vertices->UpdatePositions(std::move(starting_positions));
		std::unique_ptr<SceneNode> mesh_node = make_unique<SceneNode>();
		mesh_node->CreateComponent<RenderingComponent>(upper_diagonal_vertices);
		mesh_node->GetComponentPtr<RenderingComponent>()->SetDrawMode(DrawMode::Lines);
		mesh_node->CreateComponent<ShadingComponent>(simple_shader);
		AddChild(std::move(mesh_node));
	}





	void BouncyNode3D2::UpdatePositions() {
		
		std::unique_ptr<PositionArray> triangle_positions = make_unique<PositionArray>(current_states->positions);
		triangle_vertices->UpdatePositions(std::move(triangle_positions));
		std::unique_ptr<PositionArray> same_circle_positions = make_unique<PositionArray>(current_states->positions);
		same_circle_vertices->UpdatePositions(std::move(same_circle_positions));
		std::unique_ptr<PositionArray> different_circle_positions = make_unique<PositionArray>(current_states->positions);
		different_circle_vertices->UpdatePositions(std::move(different_circle_positions));
		std::unique_ptr<PositionArray> pressure_positions = make_unique<PositionArray>(current_states->positions);
		pressure_vertices->UpdatePositions(std::move(pressure_positions));
		std::unique_ptr<PositionArray> upper_diagonal_positions = make_unique<PositionArray>(current_states->positions);
		upper_diagonal_vertices->UpdatePositions(std::move(upper_diagonal_positions));
		std::unique_ptr<PositionArray> lower_diagonal_positions = make_unique<PositionArray>(current_states->positions);
		lower_diagonal_vertices->UpdatePositions(std::move(lower_diagonal_positions));

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
