#include "BouncyNode3D.hpp"

#include "gloo/SceneNode.hpp"
#include "IntegratorBase.hpp"
#include "BouncySystem.hpp"
#include "ParticleState.hpp"
#include "IntegratorFactory.hpp"
#include "ForwardEulerIntegrator.hpp"

#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"

#include <cmath>


namespace GLOO {
	BouncyNode3D::BouncyNode3D(std::unique_ptr<IntegratorBase<BouncySystem, ParticleState>> integrator,
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
		//state_.positions.emplace_back(initial_point);
		//state_.velocities.emplace_back(glm::vec3(0));  //
		//system_.AddParticle(1.0f);
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
			//std::cout << "BRUH" << std::endl;
			int current_size = state_.positions.size();
			for (int j = current_size - (n-2); j < current_size; ++j) {  // Iterate over the last n-2 indices in the state_.positions array
				//std::cout << "Odd... j: " << j << ", state_.positions.size() - (n-2): " << state_.positions.size() - (n - 2) << std::endl;
				
				glm::vec3 rotated_point = sphere_rotation_matrix * state_.positions[j];
				state_.positions.emplace_back(rotated_point);
				state_.velocities.emplace_back(glm::vec3(0));
				system_.AddParticle(1.0f);
			}
		}
		state_.positions.emplace_back(glm::vec3(0, radius, 0));  // Add the top point of the sphere
		state_.velocities.emplace_back(glm::vec3(0));
		system_.AddParticle(1.0f);

		state_.positions.emplace_back(glm::vec3(0, -radius, 0));  // Add the bottom point of the sphere
		state_.velocities.emplace_back(glm::vec3(0));
		system_.AddParticle(1.0f);


		std::unique_ptr<NormalArray> vertex_normals = make_unique<NormalArray>();  // For the vertex object
		std::unique_ptr<PositionArray> vertex_positions = make_unique<PositionArray>();
		std::unique_ptr<IndexArray> line_segment_indices = make_unique<IndexArray>();

		//float initial_adjacent_distance = glm::length(state_.positions[1] - state_.positions[0]);
		//for (int i = 0; i < n; ++i) {
		//	if (n == 1) break;

		//	glm::vec3 translated_position = state_.positions[i] + center;
		//	state_.positions[i] = translated_position;

		//	initial_heights_.emplace_back(translated_position);
		//	falling_.emplace_back(true);
		//	initial_height_set_.emplace_back(true);
		//	vertex_positions->emplace_back(translated_position);
		//	vertex_normals->emplace_back(1, 0, 0);

		//	//if (i != 0) {
		//	//	//system_.AddSpring(i - 1, i, initial_adjacent_distance, 300);
		//	//}

		//	if (i < n - 1) {
		//		line_segment_indices->emplace_back(i);
		//		line_segment_indices->emplace_back(i + 1);

		//	}
		//	else {
		//		line_segment_indices->emplace_back(i);
		//		line_segment_indices->emplace_back(0); // Connect last point to first point, which is at index 0.
		//	}
		//	//system_.AddSpring(i, n, radius, 500); // The center point's index is n, by construction (the last point in the array)
		//	collided_indices_.emplace_back(0);
		//}
		//system_.AddSpring(n-1, 0, initial_adjacent_distance, 300); // Connect a spring between the last point and the first point
		//collided_indices_.emplace_back(0);
		//std::cout << 2 << std::endl;
		//for (int i = 0; i < n; ++i) {
		//	if (n == 1) break;
		//	for (int j = i + 1; j < n; ++j) {
		//		float initial_distance = glm::length(state_.positions[j] - state_.positions[i]);
		//		system_.AddSpring(i, j, initial_distance, 100); //
		//	}
		//}

		std::shared_ptr<PhongShader> shader = std::make_shared<PhongShader>();
		for (int i = 0; i < state_.positions.size(); i++) {
			std::unique_ptr<VertexObject> sphere_mesh = PrimitiveFactory::CreateSphere(0.035f, 25, 25);
			auto rendering_node = make_unique<SceneNode>();
			rendering_node->CreateComponent<RenderingComponent>(std::move(sphere_mesh));
			rendering_node->CreateComponent<ShadingComponent>(shader);
			rendering_node->GetTransform().SetPosition(state_.positions[i]);
			AddChild(std::move(rendering_node));
		}

		//std::unique_ptr<SceneNode> line_segments_node = make_unique<SceneNode>();
		//line_segments_node->CreateComponent<ShadingComponent>(shader);

		//std::shared_ptr<VertexObject> line_segments = std::make_shared<VertexObject>();
		//line_segments->UpdatePositions(std::move(vertex_positions));
		//line_segments->UpdateIndices(std::move(line_segment_indices)); //
		//line_segments->UpdateNormals(std::move(vertex_normals));

		//auto& rc = line_segments_node->CreateComponent<RenderingComponent>(line_segments);
		//rc.SetDrawMode(DrawMode::Lines);

		// This node will be located at index state_.positions.size() of the children nodes.
		//AddChild(std::move(line_segments_node));
		std::cout << 3 << std::endl;
	}

	void BouncyNode3D::Update(double delta_time) {
		//double time_after_steps = current_time_ + delta_time + time_left_over_;
		//while (current_time_ + integration_step_ <= time_after_steps) {
		//	state_ = integrator_->Integrate(system_, state_, current_time_, integration_step_);
		//	current_time_ += integration_step_;
		//}
		//time_left_over_ = time_after_steps - current_time_;

		//std::unique_ptr<PositionArray> new_vertex_positions = make_unique<PositionArray>();
		//for (int i = 0; i < state_.positions.size(); i++) {
		//	glm::vec3 updated_particle_pos = state_.positions[i];

		//	//float epsilon = 0.000001f;
		//	//if (glm::length(state_.velocities[i]) <= epsilon) {
		//	//	std::cout << "odd!!!" << std::endl; // 
		//	//	initial_heights_[i] = updated_particle_pos;
		//	//}

		//	if (state_.velocities[i].y > 0) {
		//		falling_[i] = false;
		//	}
		//	else if (!initial_height_set_[i]) {
		//		if (state_.positions[i].y - floor_point_.y > 0.00001) {
		//			std::cout << "yay!!! " << state_.positions[i].y << " " << state_.positions[i].y << " " << (state_.positions[i].y > floor_point_.y) << std::endl;
		//		}
		//		initial_heights_[i] = updated_particle_pos;
		//		initial_height_set_[i] = true;
		//	}

		//	//float epsilon = 0.000001f;
		//	glm::vec3 floor_to_pos_vec = updated_particle_pos - floor_point_;
		//	float plane_dot_prdct = glm::dot(floor_normal_, floor_to_pos_vec);

		//	//if (collided_indices_[i] > 1) {
		//	//	//std::cout << "bruh... " << i << std::endl;
		//	//	collided_indices_[i] -= 1;
		//	//}
		//	//else if (collided_indices_[i]) {
		//	//	collided_indices_[i] = 0;
		//	//	system_.IndexUncollided(i);
		//	//}
		//	if (collided_indices_[i]) {
		//		collided_indices_[i] = 0;
		//		system_.IndexUncollided(i);
		//	}

		//	//float epsilon = 0.000001f;
		//	//float epsilon = 1.0f;
		//	if (plane_dot_prdct <= 0) {
		//		//std::cout << "here" << std::endl;
		//		state_.positions[i].y = floor_point_.y; // TODO: Maybe generalize this to inclined surfaces
		//		glm::vec3 velocity_before_bounce = glm::vec3(0, sqrt(2 * g * (initial_heights_[i].y - floor_point_.y)), 0); // 
		//		//glm::vec3 velocity_before_bounce = state_.velocities[i];
		//		state_.velocities[i] = coefficient_of_restitution_ * velocity_before_bounce;
		//		std::cout << "bruh.... " << g << " " << initial_heights_[i].y - floor_point_.y << " " << 2 * g * (initial_heights_[i].y - floor_point_.y) << " " << sqrt(2 * g * (initial_heights_[i].y - floor_point_.y)) << " " << glm::to_string(state_.velocities[i]) << std::endl;
		//		//state_.velocities[i] = glm::vec3(0);
		//		system_.IndexCollided(i);
		//		collided_indices_[i] = 1;

		//		initial_height_set_[i] = false;
		//	}

		//	new_vertex_positions->emplace_back(updated_particle_pos);
		//	Transform& node_transform = GetChild(i).GetTransform();
		//	node_transform.SetPosition(updated_particle_pos);
		//}
		//VertexObject* line_segment_vertex_obj =
		//	GetChild(state_.positions.size()).GetComponentPtr<RenderingComponent>()->GetVertexObjectPtr();
		//line_segment_vertex_obj->UpdatePositions(std::move(new_vertex_positions));
	}
}
