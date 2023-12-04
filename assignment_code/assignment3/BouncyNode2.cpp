#include "BouncyNode2.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp";
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO {

	BouncyNode2::BouncyNode2(
		std::unique_ptr<IntegratorBase<BouncySystem2, ParticleState>> integrator,
		double dt,
		glm::vec3 center,
		float radius,
		unsigned int circumference_partition
	) :
		forces(std::move(make_unique<BouncySystem2>())), integrator(std::move(integrator)), dt(dt), circumference_partition(circumference_partition),
		center(center), radius(radius)
	{
		current_states = make_unique<ParticleState>();
		CreateBallSystem();
	}

	void BouncyNode2::AddParticle(const glm::vec3& starting_position) {
		std::unique_ptr<VertexObject> particle = PrimitiveFactory::CreateSphere(0.03, 10, 10);
		std::unique_ptr<ShaderProgram> phongShader = make_unique<PhongShader>();
		std::unique_ptr<SceneNode> fixed_node = make_unique<SceneNode>();
		fixed_node->CreateComponent<RenderingComponent>(std::move(particle));
		fixed_node->CreateComponent<ShadingComponent>(std::move(phongShader));
		fixed_node->GetTransform().SetPosition(starting_position);
		glm::vec3 starting_speed(0, 0, 0);
		current_states->positions.push_back(starting_position);
		current_states->velocities.push_back(glm::vec3(0, 0, 0));
		forces->AddParticle(default_mass);

		AddChild(std::move(fixed_node));

	}

	void BouncyNode2::Update(double delta_time) {

		total_time_elapsed += delta_time;

		while (start_time + dt < total_time_elapsed) {
			ParticleState& new_states = integrator->Integrate(*forces, *current_states, start_time, dt);
			UpdateStates(new_states);
			start_time += dt;
			UpdateSystem();

		}

	}




	void BouncyNode2::UpdateStates(ParticleState& new_states) {
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
		//AddParticle(center+glm::vec3(0,0,1));
		forces->SetFloorNormal(glm::vec3(0, 1, 0));
		forces->SetFloorSurfacePoint(glm::vec3(0, -1, 0));

		AddStretchSprings();
		//AddStructuralSprings();
		//AddCentralSprings();







	}

	void BouncyNode2::UpdateSystem() {
		std::unique_ptr<PositionArray> new_positions = make_unique<PositionArray>();
		for (glm::vec3 position : current_states->positions) {
			new_positions->push_back(position);
		}


	}





	void BouncyNode2::AddStretchSprings() {

		for (int i = 0; i < circumference_partition; i++) {

			for (int j = i+1; j < circumference_partition; j++) {

				float rest_length = glm::distance(GetChild(i).GetTransform().GetPosition(), GetChild(j).GetTransform().GetPosition());

				float division_factor =j-i;

				forces->AddSpring(i, j,300000.0f/(division_factor), rest_length);
			}
		}
	}

	void BouncyNode2::AddCentralSprings() {

		for (int i = 0; i < circumference_partition; i++) {
			forces->AddSpring(circumference_partition,i, 500, radius);
		}
	}

	void BouncyNode2::AddStructuralSprings() {

		for (int i = 0; i < circumference_partition; i++) {
			int parent_index = (i - 1) % circumference_partition;
			float rest_length = glm::distance(GetChild(i).GetTransform().GetPosition(), GetChild(parent_index).GetTransform().GetPosition());

			forces->AddSpring(i, parent_index, 50000, rest_length);
		}
	}

	

	void BouncyNode2::ResetSystem() {

		//ParticleState reset_sates;
		//double side_scale_factor = 1.0 / nodes_in_edge * dimension;
		//for (int y = 0; y < nodes_in_edge; y++) {
		//	double y_pos = side_scale_factor * y;

		//	for (int x = 0; x < nodes_in_edge; x++) {
		//		double x_pos = side_scale_factor * x;

		//		glm::vec3 particle_pos(x_pos, 0, y_pos);
		//		reset_sates.positions.push_back(particle_pos);
		//		reset_sates.velocities.emplace_back(0, 0, 0);
		//	}
		//}

	}



}  // namespace GLOO
