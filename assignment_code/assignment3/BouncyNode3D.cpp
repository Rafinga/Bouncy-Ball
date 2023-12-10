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

#include <cmath>


namespace GLOO {
	BouncyNode3D::BouncyNode3D(std::unique_ptr<IntegratorBase<BouncySystem, ParticleState>> integrator,
	//BouncyNode3D::BouncyNode3D(std::unique_ptr<IntegratorBase<BouncySystem2, ParticleState>> integrator,
		double integration_step,
		glm::vec3 center, float radius, int n)
		: integrator_(std::move(integrator))
	{

		current_time_ = 0.0;
		time_left_over_ = 0.0;
		integration_step_ = integration_step;
		std::vector<float> masses;

		floor_normal_ = glm::vec3(0, 1, 0);
		floor_point_ = glm::vec3(0, -1, 0);

		float rotation_radians = 2 * 3.14159265 / (float)n;
		float cos_angle = cos(rotation_radians);
		float sin_angle = sin(rotation_radians);
		glm::mat3 circle_rotation_matrix(cos_angle, sin_angle, 0,
										-sin_angle, cos_angle, 0,
										   0,         0,       1); // z-coordinate stays fixed, this is for rotation in the xy-plane.

		glm::mat3 sphere_rotation_matrix(cos_angle, 0, sin_angle,
										     0,     1,     0,
										-sin_angle, 0, cos_angle); // z-coordinate stays fixed, this is for rotation in the xy-plane.

		glm::vec3 current_point(0, radius, 0);  // Construct initial circle
		for (int i = 0; i < n - 1; ++i) {
			glm::vec3 rotated_point = circle_rotation_matrix * current_point;
			if (i != n / 2 - 1) {
				state_.positions.emplace_back(rotated_point);
				state_.velocities.emplace_back(glm::vec3(0));
				system_.AddParticle(1.0f);
			}
			current_point = rotated_point;
		}
		std::cout << 1 << std::endl;

		for (int i = 0; i < n/2 - 1; ++i) {  // Iterate over the number of rotations
			int current_size = state_.positions.size();
			// Iterate over the last n-2 indices (i.e. the last circle) currently in the state_.positions array
			for (int j = current_size - (n-2); j < current_size; ++j) {
				glm::vec3 rotated_point = sphere_rotation_matrix * state_.positions[j];
				state_.positions.emplace_back(rotated_point);
				state_.velocities.emplace_back(glm::vec3(0));
				system_.AddParticle(1.0f);
			}
		}

		state_.positions.emplace_back(glm::vec3(0, radius, 0));  // Add the top point of the sphere.
		state_.velocities.emplace_back(glm::vec3(0));
		system_.AddParticle(1.0f);

		state_.positions.emplace_back(glm::vec3(0, -radius, 0));  // Add the bottom point of the sphere.
		state_.velocities.emplace_back(glm::vec3(0));
		system_.AddParticle(1.0f);

		state_.positions.emplace_back(glm::vec3(0, 0, 0));  // Add the center-point of the sphere.
		state_.velocities.emplace_back(glm::vec3(0));
		system_.AddParticle(1.0f);


		std::unique_ptr<NormalArray> vertex_normals = make_unique<NormalArray>();  // For the vertex object
		std::unique_ptr<PositionArray> vertex_positions = make_unique<PositionArray>();
		std::unique_ptr<IndexArray> line_segment_indices = make_unique<IndexArray>();


		// Vertical structural springs
		int num_points_on_circle = (n - 2);
		for (int i = 0; i < state_.positions.size() - 3; ++i) {
			// Sprigns from the top-point and bottom-point are handled separately in the next two for-loops.
			if ((i % num_points_on_circle) == n / 2 - 2 || (i % num_points_on_circle) == num_points_on_circle - 1) continue;

			float rest_length = glm::length(state_.positions[i + 1] - state_.positions[i]);
			system_.AddSpring(i, i + 1, rest_length, 1000);  // Structural springs ("downwards" along a single circle)

			line_segment_indices->emplace_back(i);  // Render "vertical" structural springs
			line_segment_indices->emplace_back(i + 1);
		}

		//  Handle springs from top-point to each point on the "highest" horizontal ring.
		int top_point_index = state_.positions.size() - 3;
		float top_rest_length = glm::length(state_.positions[top_point_index] - state_.positions[0]);
		for (int i = 0; i < state_.positions.size() - 3; i = i + num_points_on_circle) {
			system_.AddSpring(top_point_index, i, top_rest_length, 1000);  // Same structural springs for top point
			system_.AddSpring(top_point_index, i + n - 3, top_rest_length, 1000);

			line_segment_indices->emplace_back(top_point_index);
			line_segment_indices->emplace_back(i);

			line_segment_indices->emplace_back(top_point_index);
			line_segment_indices->emplace_back(i + n - 3);
		}

		//  Handle springs from bottom-point to each point on the "lowest" horizontal ring.
		int bottom_point_index = state_.positions.size() - 2;
		float bottom_rest_length = glm::length(state_.positions[bottom_point_index] - state_.positions[num_points_on_circle/2 - 1]);
		for (int i = n/2 - 2; i < state_.positions.size() - 3; i = i + num_points_on_circle) {
			system_.AddSpring(bottom_point_index, i, bottom_rest_length, 1000);  // Same structural springs for bottom point
			system_.AddSpring(bottom_point_index, i + 1, bottom_rest_length, 1000);

			line_segment_indices->emplace_back(bottom_point_index);
			line_segment_indices->emplace_back(i);

			line_segment_indices->emplace_back(bottom_point_index);
			line_segment_indices->emplace_back(i + 1);
		}


		// Horizontal structural springs
		for (int i = 0; i < state_.positions.size() - 3; ++i) { 
			int next_i = i + num_points_on_circle;
			if (next_i >= state_.positions.size() - 3) {  // The edge-case for when we're wrapping around
				// Need to subtract num_points_on_circle - 1 because the orientation of the next-circle over flips.
				next_i = num_points_on_circle - 1 - (next_i % num_points_on_circle);
			}

			float rest_length = glm::length(state_.positions[next_i] - state_.positions[i]);
			system_.AddSpring(i, next_i, rest_length, 1000);

			line_segment_indices->emplace_back(i);  // Render "horizontal" structural springs
			line_segment_indices->emplace_back(next_i);
		}

		// Pressure springs
		for (int i = 0; i < state_.positions.size() - 3; ++i) {
			float rest_length = radius;
			system_.AddSpring(i, state_.positions.size() - 1, rest_length, 1000);

			//if (i == 0 || i == n - 2 || i == state_.positions.size() - 3 - 2) {
			//	line_segment_indices->emplace_back(i);  //  Render pressure springs
			//	line_segment_indices->emplace_back(state_.positions.size() - 1);
			//	std::cout << "BRUH: " << i << std::endl;
			//}

			//if (i < num_points_on_circle) {
			//	line_segment_indices->emplace_back(i);  //  Render pressure springs
			//	line_segment_indices->emplace_back(state_.positions.size() - 1);
			//}
		}

		// Diagonal structural springs
		for (int i = 0; i < state_.positions.size() - 3; ++i) {
			//  We don't want to add bottom-diagonal springs if our bottom_diagonal_i index is the same as either
			//  the bottom or top point of the sphere (corresponding to the first and second conditionals in the `if`,
			//  respectively.
			if (i % num_points_on_circle != n/2 - 2 && (i % num_points_on_circle) != num_points_on_circle - 1) {
				int bottom_diagonal_i = i + num_points_on_circle + 1;
				if (bottom_diagonal_i >= state_.positions.size() - 3) {
					bottom_diagonal_i = num_points_on_circle - 1 - (bottom_diagonal_i % num_points_on_circle);
				}
				if (abs(bottom_diagonal_i - i) > num_points_on_circle + 1)
					std::cout << "bot_diag_i: " << bottom_diagonal_i << ", i: " << i << std::endl;
				float bottom_rest_length = glm::length(state_.positions[bottom_diagonal_i] - state_.positions[i]);
				system_.AddSpring(i, bottom_diagonal_i, bottom_rest_length, 1000);  //

				line_segment_indices->emplace_back(i);
				line_segment_indices->emplace_back(bottom_diagonal_i);
			}

			//  We don't want to add top-diagonal springs if our bottom_diagonal_i index is the same as either
			//  the top or bottom point of the sphere (corresponding to the first and second conditionals in the `if`,
			//  respectively.
			if (i % num_points_on_circle != 0 && (i % num_points_on_circle) != n/2 - 1) {
				int top_diagonal_i = i + num_points_on_circle - 1;
				if (top_diagonal_i >= state_.positions.size() - 3) {
					top_diagonal_i = num_points_on_circle - 1 - (top_diagonal_i % num_points_on_circle);
				}
				float top_rest_length = glm::length(state_.positions[top_diagonal_i] - state_.positions[i]);
				system_.AddSpring(i, top_diagonal_i, top_rest_length, 1000);  //

				line_segment_indices->emplace_back(i);
				line_segment_indices->emplace_back(top_diagonal_i);
			}
		}


		for (int i = 0; i < state_.positions.size(); ++i) {
			glm::vec3 original_position = state_.positions[i];
			glm::vec3 translated_position = original_position + center;
			state_.positions[i] = translated_position;  //

			vertex_positions->emplace_back(translated_position);
			vertex_normals->emplace_back(original_position);
		}

		std::shared_ptr<SimpleShader> shader = std::make_shared<SimpleShader>();
		for (int i = 0; i < state_.positions.size(); i++) {
			std::unique_ptr<VertexObject> sphere_mesh = PrimitiveFactory::CreateSphere(0.035f, 25, 25);
			auto rendering_node = make_unique<SceneNode>();
			//rendering_node->CreateComponent<RenderingComponent>(std::move(sphere_mesh));  // Render the vertices as spheres.
			//rendering_node->CreateComponent<ShadingComponent>(shader);
			rendering_node->GetTransform().SetPosition(state_.positions[i]);
			AddChild(std::move(rendering_node));
		}

		std::unique_ptr<SceneNode> line_segments_node = make_unique<SceneNode>();
		line_segments_node->CreateComponent<ShadingComponent>(shader);

		std::shared_ptr<VertexObject> line_segments = std::make_shared<VertexObject>();
		line_segments->UpdatePositions(std::move(vertex_positions));
		line_segments->UpdateIndices(std::move(line_segment_indices)); //
		line_segments->UpdateNormals(std::move(vertex_normals));

		auto& rc = line_segments_node->CreateComponent<RenderingComponent>(line_segments);
		rc.SetDrawMode(DrawMode::Lines);

		// This node will be located at index state_.positions.size() of the children nodes.
		AddChild(std::move(line_segments_node));
		std::cout << 3 << std::endl;
	}

	void BouncyNode3D::Update(double delta_time) {
		//std::cout << 4 << std::endl;
		double time_after_steps = current_time_ + delta_time + time_left_over_;
		while (current_time_ + integration_step_ <= time_after_steps) {
			state_ = integrator_->Integrate(system_, state_, current_time_, integration_step_);
			current_time_ += integration_step_;
		}
		time_left_over_ = time_after_steps - current_time_;

		std::unique_ptr<PositionArray> new_vertex_positions = make_unique<PositionArray>();
		for (int i = 0; i < state_.positions.size(); i++) {
			glm::vec3 updated_particle_pos = state_.positions[i];

			//float epsilon = 0.000001f;
			//if (glm::length(state_.velocities[i]) <= epsilon) {
			//	std::cout << "odd!!!" << std::endl; // 
			//	initial_heights_[i] = updated_particle_pos;
			//}

			//if (state_.velocities[i].y > 0) {
			//	falling_[i] = false;
			//}
			//else if (!initial_height_set_[i]) {
			//	if (state_.positions[i].y - floor_point_.y > 0.00001) {
			//		std::cout << "yay!!! " << state_.positions[i].y << " " << state_.positions[i].y << " " << (state_.positions[i].y > floor_point_.y) << std::endl;
			//	}
			//	initial_heights_[i] = updated_particle_pos;
			//	initial_height_set_[i] = true;
			//}

			//float epsilon = 0.000001f;
			glm::vec3 floor_to_pos_vec = updated_particle_pos - floor_point_;
			float plane_dot_prdct = glm::dot(floor_normal_, floor_to_pos_vec);

			//if (collided_indices_[i] > 1) {
			//	//std::cout << "bruh... " << i << std::endl;
			//	collided_indices_[i] -= 1;
			//}
			//else if (collided_indices_[i]) {
			//	collided_indices_[i] = 0;
			//	system_.IndexUncollided(i);
			//}
			//if (collided_indices_[i]) {
			//	collided_indices_[i] = 0; //
			//	system_.IndexUncollided(i);
			//}

			//float epsilon = 0.000001f;
			//float epsilon = 1.0f;
			if (plane_dot_prdct <= 0) {
				//std::cout << "here" << std::endl;
				state_.positions[i].y = floor_point_.y; // TODO: Maybe generalize this to inclined surfaces

				//glm::vec3 velocity_before_bounce = glm::vec3(0, sqrt(2 * g * (initial_heights_[i].y - floor_point_.y)), 0); // 

				//glm::vec3 velocity_before_bounce = state_.velocities[i];
				//state_.velocities[i] = coefficient_of_restitution_ * velocity_before_bounce;
				//std::cout << "here...?" << std::endl;
				state_.velocities[i] = glm::vec3(0);
				//std::cout << "bruh.... " << g << " " << initial_heights_[i].y - floor_point_.y << " " << 2 * g * (initial_heights_[i].y - floor_point_.y) << " " << sqrt(2 * g * (initial_heights_[i].y - floor_point_.y)) << " " << glm::to_string(state_.velocities[i]) << std::endl;
				//state_.velocities[i] = glm::vec3(0);
				//system_.IndexCollided(i);
				//collided_indices_[i] = 1;

				//initial_height_set_[i] = false;
			}

			//std::cout << 5 << std::endl;
			new_vertex_positions->emplace_back(updated_particle_pos);
			Transform& node_transform = GetChild(i).GetTransform();
			node_transform.SetPosition(updated_particle_pos);
		}
		//std::cout << 6 << std::endl;
		//VertexObject* line_segment_vertex_obj =
		//	GetChild(state_.positions.size()).GetComponentPtr<RenderingComponent>()->GetVertexObjectPtr();  //
		//line_segment_vertex_obj->UpdatePositions(std::move(new_vertex_positions));
		//std::cout << "finished!" << std::endl;
	}
}
