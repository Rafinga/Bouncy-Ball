#include "BouncyNode3D.hpp"

#include "gloo/SceneNode.hpp"
#include "IntegratorBase.hpp"
#include "BouncySystem.hpp"
#include "BouncySystem2.hpp"
#include "ParticleState.hpp"
#include "IntegratorFactory.hpp"
#include "ForwardEulerIntegrator.hpp"

#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/shaders/SimpleShader.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"

#include "gloo/InputManager.hpp"

#include <cmath>


namespace GLOO {
	//BouncyNode3D::BouncyNode3D(std::unique_ptr<IntegratorBase<BouncySystem, ParticleState>> integrator,
	BouncyNode3D::BouncyNode3D(std::unique_ptr<IntegratorBase<BouncySystem2, ParticleState>> integrator,
		double dt,
		glm::vec3 center, float radius, int n,
		glm::vec3 floor_point, glm::vec3 floor_normal,
		glm::vec3 initial_velocity
	)
		: integrator_(std::move(integrator))
	{

		start_time_ = 0.0;
		total_time_elapsed_ = 0.0;
		dt_ = dt;
		std::vector<float> masses;

		floor_normal_ = floor_normal;
		floor_point_ = floor_point;

		float rotation_radians = 2 * 3.14159265 / (float)n;
		float cos_angle = cos(rotation_radians);
		float sin_angle = sin(rotation_radians);
		glm::mat3 circle_rotation_matrix(cos_angle, sin_angle, 0,
			-sin_angle, cos_angle, 0,
			0, 0, 1); // z-coordinate stays fixed, this is for rotation in the xy-plane.

		glm::mat3 sphere_rotation_matrix(cos_angle, 0, sin_angle,
			0, 1, 0,
			-sin_angle, 0, cos_angle); // y-coordinate stays fixed, this is for rotation in the xz-plane.

		glm::vec3 current_point(0, radius, 0);  // Construct initial circle
		for (int i = 0; i < n - 1; ++i) {
			glm::vec3 rotated_point = circle_rotation_matrix * current_point;
			if (i != n / 2 - 1) {
				state_.positions.emplace_back(rotated_point);
				state_.velocities.emplace_back(initial_velocity);
				initial_velocities_.emplace_back(initial_velocity);
				system_.AddParticle(1.0f);
			}
			current_point = rotated_point;
		}
		std::cout << 1 << std::endl;

		for (int i = 0; i < n / 2 - 1; ++i) {  // Iterate over the number of rotations
			int current_size = state_.positions.size();
			// Iterate over the last n-2 indices (i.e. the last circle) currently in the state_.positions array
			for (int j = current_size - (n - 2); j < current_size; ++j) {
				glm::vec3 rotated_point = sphere_rotation_matrix * state_.positions[j];
				state_.positions.emplace_back(rotated_point);
				state_.velocities.emplace_back(initial_velocity);
				initial_velocities_.emplace_back(initial_velocity);
				system_.AddParticle(1.0f);
			}
		}

		state_.positions.emplace_back(glm::vec3(0, radius, 0));  // Add the top point of the sphere.
		state_.velocities.emplace_back(initial_velocity);
		initial_velocities_.emplace_back(initial_velocity);
		system_.AddParticle(1.0f);

		state_.positions.emplace_back(glm::vec3(0, -radius, 0));  // Add the bottom point of the sphere.
		state_.velocities.emplace_back(initial_velocity);
		initial_velocities_.emplace_back(initial_velocity);
		system_.AddParticle(1.0f);

		state_.positions.emplace_back(glm::vec3(0, 0, 0));  // Add the center-point of the sphere.
		state_.velocities.emplace_back(initial_velocity);
		initial_velocities_.emplace_back(initial_velocity);
		system_.AddParticle(1.0f);


		// Vertical structural springs
		int num_points_on_circle = (n - 2);
		for (int i = 0; i < state_.positions.size() - 3; ++i) {
			// Sprigns from the top-point and bottom-point are handled separately in the next two for-loops.
			if ((i % num_points_on_circle) == n / 2 - 2 || (i % num_points_on_circle) == num_points_on_circle - 1) continue;

			float rest_length = glm::length(state_.positions[i + 1] - state_.positions[i]);
			system_.AddSpring(i, i + 1, rest_length, 50);  // Structural springs ("downwards" along a single circle)

			vertical_structural_indices_.emplace_back(i);  // Render "vertical" structural springs
			vertical_structural_indices_.emplace_back(i + 1);
		}

		//  Handle springs from top-point to each point on the "highest" horizontal ring.
		int top_point_index = state_.positions.size() - 3;
		float top_rest_length = glm::length(state_.positions[top_point_index] - state_.positions[0]);
		for (int i = 0; i < state_.positions.size() - 3; i = i + num_points_on_circle) {
			system_.AddSpring(top_point_index, i, top_rest_length, 50);  // Same structural springs for top point
			system_.AddSpring(top_point_index, i + n - 3, top_rest_length, 50);

			vertical_structural_indices_.emplace_back(top_point_index);
			vertical_structural_indices_.emplace_back(i);

			vertical_structural_indices_.emplace_back(top_point_index);
			vertical_structural_indices_.emplace_back(i + n - 3);
		}

		//  Handle springs from bottom-point to each point on the "lowest" horizontal ring.
		int bottom_point_index = state_.positions.size() - 2;
		float bottom_rest_length = glm::length(state_.positions[bottom_point_index] - state_.positions[num_points_on_circle / 2 - 1]);
		for (int i = n / 2 - 2; i < state_.positions.size() - 3; i = i + num_points_on_circle) {
			system_.AddSpring(bottom_point_index, i, bottom_rest_length, 50);  // Same structural springs for bottom point
			system_.AddSpring(bottom_point_index, i + 1, bottom_rest_length, 50);

			vertical_structural_indices_.emplace_back(bottom_point_index);
			vertical_structural_indices_.emplace_back(i);

			vertical_structural_indices_.emplace_back(bottom_point_index);
			vertical_structural_indices_.emplace_back(i + 1);
		}


		// Horizontal structural springs
		for (int i = 0; i < state_.positions.size() - 3; ++i) {
			int next_i = i + num_points_on_circle;
			if (next_i >= state_.positions.size() - 3) {  // The edge-case for when we're wrapping around
				// Need to subtract num_points_on_circle - 1 because the orientation of the next-circle over flips.
				next_i = num_points_on_circle - 1 - (next_i % num_points_on_circle);
			}

			float rest_length = glm::length(state_.positions[next_i] - state_.positions[i]);
			system_.AddSpring(i, next_i, rest_length, 1000.0);

			horizontal_structural_indices_.emplace_back(i);  // Render "horizontal" structural springs
			horizontal_structural_indices_.emplace_back(next_i);
		}

		// Pressure springs
		for (int i = 0; i < state_.positions.size() - 3; ++i) {
			float rest_length = radius;
			system_.AddSpring(i, state_.positions.size() - 1, rest_length, 0.02);

			pressure_indices_.emplace_back(i);
			pressure_indices_.emplace_back(state_.positions.size() - 1);  // Render presure springs
		}

		// Diagonal structural springs
		for (int i = 0; i < state_.positions.size() - 3; ++i) {
			//  We don't want to add bottom-diagonal springs if our bottom_diagonal_i index is the same as either
			//  the bottom or top point of the sphere (corresponding to the first and second conditionals in the `if`,
			//  respectively.
			if (i % num_points_on_circle != n / 2 - 2 && (i % num_points_on_circle) != num_points_on_circle - 1) {
				int bottom_diagonal_i = i + num_points_on_circle + 1;
				if (bottom_diagonal_i >= state_.positions.size() - 3) {
					bottom_diagonal_i = num_points_on_circle - 1 - (bottom_diagonal_i % num_points_on_circle);
				}
				float bottom_rest_length = glm::length(state_.positions[bottom_diagonal_i] - state_.positions[i]);
				system_.AddSpring(i, bottom_diagonal_i, bottom_rest_length, 0.5);  //

				bottom_diagonal_structural_indices_.emplace_back(i);
				bottom_diagonal_structural_indices_.emplace_back(bottom_diagonal_i);
			}

			//  We don't want to add top-diagonal springs if our bottom_diagonal_i index is the same as either
			//  the top or bottom point of the sphere (corresponding to the first and second conditionals in the `if`,
			//  respectively.
			if (i % num_points_on_circle != 0 && (i % num_points_on_circle) != n / 2 - 1) {
				int top_diagonal_i = i + num_points_on_circle - 1;
				if (top_diagonal_i >= state_.positions.size() - 3) {
					top_diagonal_i = num_points_on_circle - 1 - (top_diagonal_i % num_points_on_circle);
				}
				float top_rest_length = glm::length(state_.positions[top_diagonal_i] - state_.positions[i]);
				system_.AddSpring(i, top_diagonal_i, top_rest_length, 0.5);  //

				top_diagonal_structural_indices_.emplace_back(i);
				top_diagonal_structural_indices_.emplace_back(top_diagonal_i);
			}
		}

		// Logic for calculating all the traingles and their indices.
		for (int i = 0; i < state_.positions.size() - 3; ++i) {
			if (i % num_points_on_circle != n / 2 - 2 && (i % num_points_on_circle) != num_points_on_circle - 1) {
				int bottom_diagonal_i = i + num_points_on_circle + 1;
				int top_i;
				if (bottom_diagonal_i >= state_.positions.size() - 3) {
					bottom_diagonal_i = num_points_on_circle - 1 - (bottom_diagonal_i % num_points_on_circle);
					top_i = bottom_diagonal_i + 1;
				}
				else {
					top_i = bottom_diagonal_i - 1;
				}
				int bottom_i = i + 1;

				triangle_indices_.emplace_back(i);  // Bottom triangle in a given spherical square
				triangle_indices_.emplace_back(bottom_diagonal_i);
				triangle_indices_.emplace_back(i + 1);

				triangle_indices_.emplace_back(i);  // Top triangle in a given spherical square
				triangle_indices_.emplace_back(bottom_diagonal_i);
				triangle_indices_.emplace_back(top_i);
			}
		}

		// Connect the triangles for the top-hat.
		int top_idx = state_.positions.size() - 3;
		for (int triangle_idx = 0; triangle_idx < state_.positions.size() - 3; triangle_idx += num_points_on_circle) {
			int neighbor_idx = triangle_idx + num_points_on_circle;

			int symmetric_triangle_idx = triangle_idx + num_points_on_circle - 1;  // Get the index for the point across from the bottom-point
			int symmetric_neighbor_idx = symmetric_triangle_idx + num_points_on_circle;
			// Handle the wrap-around for the last vertical-circle connecting to the first vertical-circle.
			if (symmetric_neighbor_idx > state_.positions.size() - 3) {
				neighbor_idx = num_points_on_circle - 1;
				symmetric_neighbor_idx = 0;
			}

			// Triangle connecting top point and two adjacent points on the top-most level of the sphere.
			triangle_indices_.emplace_back(top_idx);
			triangle_indices_.emplace_back(triangle_idx);
			triangle_indices_.emplace_back(neighbor_idx);

			// Calculate the triangle symmetric about the top point.
			triangle_indices_.emplace_back(top_idx);
			triangle_indices_.emplace_back(symmetric_triangle_idx);
			triangle_indices_.emplace_back(symmetric_neighbor_idx);
		}

		// Connect the triangles for the bottom-hat.
		int bottom_idx = state_.positions.size() - 2;
		for (int triangle_idx = n / 2 - 2; triangle_idx < state_.positions.size() - 3; triangle_idx += num_points_on_circle) {
			int neighbor_idx = triangle_idx + num_points_on_circle;

			int symmetric_triangle_idx = triangle_idx + 1;  // Get the index for the point across from the bottom-point
			int symmetric_neighbor_idx = symmetric_triangle_idx + num_points_on_circle;
			// Handle the wrap-around for the last vertical-circle connecting to the first vertical-circle.
			if (symmetric_neighbor_idx > state_.positions.size() - 3) {
				neighbor_idx = n / 2 - 1;
				symmetric_neighbor_idx = n / 2 - 2;
			}

			// Triangle connecting bottom point and two adjacent points on the bottom-most level of the sphere.
			triangle_indices_.emplace_back(bottom_idx);
			triangle_indices_.emplace_back(triangle_idx);
			triangle_indices_.emplace_back(neighbor_idx);

			// Calculate the triangle symmetric about the top point.
			triangle_indices_.emplace_back(bottom_idx);
			triangle_indices_.emplace_back(symmetric_triangle_idx);
			triangle_indices_.emplace_back(symmetric_neighbor_idx);
		}



		//  Translate all the points on the sphere, which was originally centered at the origin, by `center`.
		for (int i = 0; i < state_.positions.size(); ++i) {
			glm::vec3 original_position = state_.positions[i];
			glm::vec3 translated_position = original_position + center;
			state_.positions[i] = translated_position;  //

			sphere_positions_.emplace_back(translated_position);

			sphere_normals_.emplace_back(original_position);
		}

		//  Add shaders and set the each vertices' rendering node transform to the correct position.
		std::shared_ptr<SimpleShader> shader = std::make_shared<SimpleShader>();
		for (int i = 0; i < state_.positions.size(); i++) {
			std::unique_ptr<VertexObject> sphere_mesh = PrimitiveFactory::CreateSphere(0.035f, 25, 25);
			auto rendering_node = make_unique<SceneNode>();
			//rendering_node->CreateComponent<RenderingComponent>(std::move(sphere_mesh));  // Render the vertices with sphere meshes.
			//rendering_node->CreateComponent<ShadingComponent>(shader);
			rendering_node->GetTransform().SetPosition(state_.positions[i]);
			AddChild(std::move(rendering_node));
		}


		// Logic to add the triangle node
		std::unique_ptr<PositionArray> triangle_positions = make_unique<PositionArray>();
		std::unique_ptr<IndexArray> triangle_indices = make_unique<IndexArray>();
		std::unique_ptr<NormalArray> triangle_normals = make_unique<NormalArray>();
		std::cout << 0 << std::endl;
		// Loop for adding the triangle indices of the ball.
		if (triangles_enabled_) {
			for (int i = 0; i < triangle_indices_.size(); ++i) {
				triangle_indices->emplace_back(triangle_indices_.at(i));
			}
			for (int i = 0; i < sphere_positions_.size(); ++i) {
				triangle_positions->emplace_back(sphere_positions_.at(i));
				triangle_normals->emplace_back(sphere_normals_.at(i));
			}
		}

		std::unique_ptr<SceneNode> triangle_node = make_unique<SceneNode>();

		std::shared_ptr<PhongShader> triangle_shader = std::make_shared<PhongShader>();
		triangle_node->CreateComponent<ShadingComponent>(triangle_shader);

		std::shared_ptr<VertexObject> triangles = std::make_shared<VertexObject>();
		triangles->UpdatePositions(std::move(triangle_positions));
		triangles->UpdateIndices(std::move(triangle_indices));
		triangles->UpdateNormals(std::move(triangle_normals));

		auto& triangle_rc = triangle_node->CreateComponent<RenderingComponent>(triangles);
		triangle_rc.SetDrawMode(DrawMode::Triangles);

		AddChild(std::move(triangle_node));


		// Logic to add the horizontal node
		std::unique_ptr<PositionArray> horizontal_structural_positions = make_unique<PositionArray>(state_.positions);
		std::unique_ptr<IndexArray> horizontal_structural_indices = make_unique<IndexArray>();
		std::unique_ptr<NormalArray> horizontal_structural_normals = make_unique<NormalArray>(sphere_normals_);
		for (int i = 0; i < horizontal_structural_indices_.size(); ++i) {
			horizontal_structural_indices->emplace_back(horizontal_structural_indices_[i]);
		}

		std::unique_ptr<SceneNode> horizontal_node = make_unique<SceneNode>();
		horizontal_node->CreateComponent<ShadingComponent>(shader);

		std::shared_ptr<VertexObject> horizontal_segments = std::make_shared<VertexObject>();
		horizontal_segments->UpdatePositions(std::move(horizontal_structural_positions));
		horizontal_segments->UpdateIndices(std::move(horizontal_structural_indices));
		horizontal_segments->UpdateNormals(std::move(horizontal_structural_normals));

		RenderingComponent& horizontal_rc = horizontal_node->CreateComponent<RenderingComponent>(horizontal_segments);
		horizontal_rc.SetDrawMode(DrawMode::Lines);

		AddChild(std::move(horizontal_node));


		// Logic to add the vertical node
		std::unique_ptr<PositionArray> vertical_structural_positions = make_unique<PositionArray>(state_.positions);
		std::unique_ptr<IndexArray> vertical_structural_indices = make_unique<IndexArray>();
		std::unique_ptr<NormalArray> vertical_structural_normals = make_unique<NormalArray>(sphere_normals_);
		for (int i = 0; i < vertical_structural_indices_.size(); ++i) {
			vertical_structural_indices->emplace_back(vertical_structural_indices_[i]);
		}

		std::unique_ptr<SceneNode> vertical_node = make_unique<SceneNode>();
		vertical_node->CreateComponent<ShadingComponent>(shader);

		std::shared_ptr<VertexObject> vertical_segments = std::make_shared<VertexObject>();
		vertical_segments->UpdatePositions(std::move(vertical_structural_positions));
		vertical_segments->UpdateIndices(std::move(vertical_structural_indices));
		vertical_segments->UpdateNormals(std::move(vertical_structural_normals));

		RenderingComponent& vertical_rc = vertical_node->CreateComponent<RenderingComponent>(vertical_segments);
		vertical_rc.SetDrawMode(DrawMode::Lines);

		AddChild(std::move(vertical_node));


		// Logic to add the pressure node
		std::unique_ptr<PositionArray> pressure_structural_positions = make_unique<PositionArray>(state_.positions);
		std::unique_ptr<IndexArray> pressure_structural_indices = make_unique<IndexArray>();
		std::unique_ptr<NormalArray> pressure_structural_normals = make_unique<NormalArray>(sphere_normals_);
		for (int i = 0; i < pressure_indices_.size(); ++i) {
			pressure_structural_indices->emplace_back(pressure_indices_[i]);
		}

		std::unique_ptr<SceneNode> pressure_node = make_unique<SceneNode>();
		pressure_node->CreateComponent<ShadingComponent>(shader);

		std::shared_ptr<VertexObject> pressure_segments = std::make_shared<VertexObject>();
		pressure_segments->UpdatePositions(std::move(pressure_structural_positions));
		pressure_segments->UpdateIndices(std::move(pressure_structural_indices));
		pressure_segments->UpdateNormals(std::move(pressure_structural_normals));

		RenderingComponent& pressure_rc = pressure_node->CreateComponent<RenderingComponent>(pressure_segments);
		pressure_rc.SetDrawMode(DrawMode::Lines);

		AddChild(std::move(pressure_node));


		// Logic to add the bottom-diagonal node
		std::unique_ptr<PositionArray> bottom_diagonal_positions = make_unique<PositionArray>(state_.positions);
		std::unique_ptr<IndexArray> bottom_diagonal_indices = make_unique<IndexArray>();
		std::unique_ptr<NormalArray> bottom_diagonal_normals = make_unique<NormalArray>(sphere_normals_);
		for (int i = 0; i < bottom_diagonal_structural_indices_.size(); ++i) {
			bottom_diagonal_indices->emplace_back(bottom_diagonal_structural_indices_[i]);
		}

		std::unique_ptr<SceneNode> bottom_diagonal_node = make_unique<SceneNode>();
		bottom_diagonal_node->CreateComponent<ShadingComponent>(shader);

		std::shared_ptr<VertexObject> bottom_diagonal_segments = std::make_shared<VertexObject>();
		bottom_diagonal_segments->UpdatePositions(std::move(bottom_diagonal_positions));
		bottom_diagonal_segments->UpdateIndices(std::move(bottom_diagonal_indices));
		bottom_diagonal_segments->UpdateNormals(std::move(bottom_diagonal_normals));

		RenderingComponent& bottom_diagonal_rc = bottom_diagonal_node->CreateComponent<RenderingComponent>(bottom_diagonal_segments);
		bottom_diagonal_rc.SetDrawMode(DrawMode::Lines);

		AddChild(std::move(bottom_diagonal_node));


		// Logic to add the top-diagonal node
		std::unique_ptr<PositionArray> top_diagonal_positions = make_unique<PositionArray>(state_.positions);
		std::unique_ptr<IndexArray> top_diagonal_indices = make_unique<IndexArray>();
		std::unique_ptr<NormalArray> top_diagonal_normals = make_unique<NormalArray>(sphere_normals_);
		for (int i = 0; i < top_diagonal_structural_indices_.size(); ++i) {
			top_diagonal_indices->emplace_back(top_diagonal_structural_indices_[i]);
		}

		std::unique_ptr<SceneNode> top_diagonal_node = make_unique<SceneNode>();
		top_diagonal_node->CreateComponent<ShadingComponent>(shader);

		std::shared_ptr<VertexObject> top_diagonal_segments = std::make_shared<VertexObject>();
		top_diagonal_segments->UpdatePositions(std::move(top_diagonal_positions));
		top_diagonal_segments->UpdateIndices(std::move(top_diagonal_indices));
		top_diagonal_segments->UpdateNormals(std::move(top_diagonal_normals));

		RenderingComponent& top_diagonal_rc = top_diagonal_node->CreateComponent<RenderingComponent>(top_diagonal_segments);
		top_diagonal_rc.SetDrawMode(DrawMode::Lines);

		AddChild(std::move(top_diagonal_node));

		num_children_ = GetChildrenCount();
		std::cout << 3 << std::endl;
	}

	void BouncyNode3D::Update(double delta_time) {
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
			SceneNode& triangles_node = GetChild(num_children_ - 6);

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
			SceneNode& horizontal_node = GetChild(num_children_ - 5);

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
			SceneNode& vertical_node = GetChild(num_children_ - 4);

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
			SceneNode& pressure_node = GetChild(num_children_ - 3);

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
			SceneNode& bottom_diagonal_node = GetChild(num_children_ - 2);

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
			SceneNode& top_diagonal_node = GetChild(num_children_ - 1);

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
			SceneNode& horizontal_node = GetChild(num_children_ - 5);
			SceneNode& vertical_node = GetChild(num_children_ - 4);
			SceneNode& pressure_node = GetChild(num_children_ - 3);
			SceneNode& bottom_diagonal_node = GetChild(num_children_ - 2);
			SceneNode& top_diagonal_node = GetChild(num_children_ - 1);

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

		total_time_elapsed_ += delta_time;

		if (InputManager::GetInstance().IsKeyPressed('S')) {
			if (s_prev_released_) {

				time_frozen_ = !time_frozen_;
			}
			s_prev_released_ = false;
		}
		else if (InputManager::GetInstance().IsKeyReleased('S')) {
			s_prev_released_ = true;
		}

		// Actual Update Logic
		while (start_time_ + dt_ < total_time_elapsed_) {

			if (!time_frozen_) {
				ParticleState& new_states = integrator_->Integrate(system_, state_, start_time_, dt_);
				UpdateStates(new_states);
			}

			start_time_ += dt_;
			if (InputManager::GetInstance().IsKeyPressed('R')) {
				if (r_prev_released_) {
					ResetSystem();
				}
				r_prev_released_ = false;
			}
			else if (InputManager::GetInstance().IsKeyReleased('R')) {
				r_prev_released_ = true;
			}
		}
	}

	void BouncyNode3D::UpdateStates(ParticleState& new_states) {
		state_.positions = new_states.positions;
		state_.velocities.clear();
		int child_index = 0;
		for (glm::vec3& new_velocity : new_states.velocities) {
			//if (IsTouchingGround(state_.positions[child_index])) {
			//	new_velocity = RemovedTableVelocityComp(new_velocity);
			//}
			state_.velocities.push_back(new_velocity);

			child_index++;
		}
	}

	bool BouncyNode3D::IsTouchingGround(glm::vec3 pos) const {
		float delta = 0.00000001;
		glm::vec3 displacement_vector = pos - floor_point_;
		return glm::dot(displacement_vector, floor_normal_) <= delta;
	}

	glm::vec3 BouncyNode3D::RemovedTableVelocityComp(glm::vec3& velocity) {
		float  projected_vector_length = glm::dot(velocity, floor_normal_);

		if (projected_vector_length >= 0) {
			return velocity;
		}
		return velocity - projected_vector_length * floor_normal_;
	}

	void BouncyNode3D::ResetSystem() {
		ParticleState reset_state;

		PositionArray reset_pos = sphere_positions_;
		PositionArray reset_vels = initial_velocities_;

		reset_state.positions = reset_pos;
		reset_state.velocities = reset_vels;

		UpdateStates(reset_state);
	}

	void BouncyNode3D::UpdateVertexObjectPositions(int child_idx) {
		// child_idx represents how many child-nodes from the last child-node in the tree
		VertexObject* vertex_obj = GetChild(num_children_ - child_idx).GetComponentPtr<RenderingComponent>()->GetVertexObjectPtr();
		std::unique_ptr<PositionArray> vertex_positions = make_unique<PositionArray>(state_.positions);

		vertex_obj->UpdatePositions(std::move(vertex_positions));
	}

	void BouncyNode3D::UpdateActiveVertexObjectPositions() {
		if (triangles_enabled_) {
			UpdateVertexObjectPositions(6);
		}

		if (wireframe_enabled_) {
			if (horizontal_structurals_active_) {
				UpdateVertexObjectPositions(5);
			}

			if (vertical_structurals_active_) {
				UpdateVertexObjectPositions(4);
			}

			if (pressure_active_) {
				UpdateVertexObjectPositions(3);
			}

			if (bottom_diagonals_active_) {
				UpdateVertexObjectPositions(2);
			}

			if (top_diagonals_active_) {
				UpdateVertexObjectPositions(1);
			}
		}
	}
}