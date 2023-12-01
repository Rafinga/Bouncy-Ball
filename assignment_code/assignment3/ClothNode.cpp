#include "ClothNode.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp";
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO {

	ClothNode::ClothNode(
		std::unique_ptr<PendulumSystem> forces,
		std::unique_ptr<IntegratorBase<PendulumSystem, ParticleState>> integrator,
		double dt,
		unsigned int nodes_in_edge
	) :
		forces(std::move(forces)), integrator(std::move(integrator)), dt(dt),nodes_in_edge(nodes_in_edge)

	{
		current_states = make_unique<ParticleState>();
		CreateGrid();
		CreateStructuralSprings();
		CreateShearSprings();
		CreateFlexSprings();
		CreateSheet();
		FixCenter(13);


		



	}

	void ClothNode::AddParticle(const glm::vec3& starting_position) {
		std::unique_ptr<VertexObject> particle = PrimitiveFactory::CreateSphere(0.1, 10, 10);
		std::unique_ptr<ShaderProgram> phongShader = make_unique<PhongShader>();
		std::unique_ptr<SceneNode> fixed_node = make_unique<SceneNode>();
		fixed_node->CreateComponent<RenderingComponent>(std::move(particle));
		fixed_node->CreateComponent<ShadingComponent>(std::move(phongShader));
		glm::vec3 starting_speed(0, 0, 0);
		current_states->positions.push_back(starting_position);
		current_states->velocities.push_back(glm::vec3(0, 0, 0));
		forces->AddParticle(default_mass);

	}

	void ClothNode::Update(double delta_time) {

		total_time_elapsed += delta_time;

		while (start_time + dt < total_time_elapsed) {
			ParticleState& new_states = integrator->Integrate(*forces, *current_states, start_time, dt);
			UpdateStates(new_states);
			start_time += dt;
			UpdateSheet();

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




	void ClothNode::UpdateStates(ParticleState& new_states) {
		current_states->positions.clear();
		int child_index = 0;
		for (const glm::vec3& new_position : new_states.positions) {
			current_states->positions.push_back(new_position);
			child_index++;
		}

		current_states->velocities.clear();
		for (const glm::vec3& new_velocity : new_states.velocities) {
			current_states->velocities.push_back(new_velocity);
		}

	}


	void ClothNode::CreateGrid() {
		double side_scale_factor = 1.0 / nodes_in_edge * dimension;
		for (int y = 0; y < nodes_in_edge; y++) {
			double y_pos = side_scale_factor*y;

			for (int x = 0; x < nodes_in_edge; x++) {
				double x_pos = side_scale_factor * x;

				glm::vec3 particle_pos(x_pos,0,y_pos);
				AddParticle(particle_pos);
			}
		}
	}
	void ClothNode::CreateStructuralSprings() {

		double side_distance = GetNodeDistance(0, 0, 1, 0);

		for (int i = 0; i < nodes_in_edge; i++) {

			for (int j = 0; j < nodes_in_edge; j++) {
				if (i < nodes_in_edge - 1) {
					forces->AddSpring(GetNodeIndex(i, j), GetNodeIndex(i + 1, j), spring_constant,side_distance);
				}
				if (j < nodes_in_edge - 1) {
					forces->AddSpring(GetNodeIndex(i, j), GetNodeIndex(i, j+1), spring_constant, side_distance);
				}
			}
		}
	}

	void ClothNode::CreateShearSprings() {

		double side_distance = GetNodeDistance(0, 0, 1, 1);

		for (int i = 0; i < nodes_in_edge-1; i++) {
			for (int j = 0; j < nodes_in_edge-1; j++) {
				forces->AddSpring(GetNodeIndex(i, j), GetNodeIndex(i + 1, j+1), spring_constant, side_distance);
			}
		}

		for (int i = 0; i < nodes_in_edge - 1; i++) {
			for (int j = nodes_in_edge - 1; 0 < j; j--) {
				forces->AddSpring(GetNodeIndex(i, j), GetNodeIndex(i +1, j - 1), spring_constant, side_distance);
			}
		}

	}

	void ClothNode::CreateFlexSprings() {

		double side_distance = GetNodeDistance(0, 0, 2, 0);

		for (int i = 0; i < nodes_in_edge; i++) {

			for (int j = 0; j < nodes_in_edge; j++) {
				if (i + 2 < nodes_in_edge) {
					forces->AddSpring(GetNodeIndex(i, j), GetNodeIndex(i + 2, j), spring_constant, side_distance);
				}
				if (j + 2 < nodes_in_edge) {
					forces->AddSpring(GetNodeIndex(i, j), GetNodeIndex(i, j + 2), spring_constant, side_distance);
				}
			}
		}
	}


	double ClothNode::GetNodeDistance(int row1, int col1, int row2, int col2) {

		int delta_row = std::abs(row1 - row2);
		int delta_col = std::abs(col1 - col2);


		double side_scale_factor = 1.0 / nodes_in_edge * dimension;

		double delta_x = side_scale_factor * delta_row;
		double delta_y = side_scale_factor * delta_col;

		return std::sqrt(delta_x * delta_x + delta_y * delta_y);

	}


	void ClothNode::CreateSheet() {
		cloth_vertices = std::make_shared<VertexObject>();
		std::unique_ptr<PositionArray> new_positions = make_unique<PositionArray>();
		std::unique_ptr<IndexArray> indexes = make_unique<IndexArray>();


		for (glm::vec3 position : current_states->positions) {
			new_positions->push_back(position);
		}

		for (int i = 0; i < nodes_in_edge; i++) {
			for (int j = 0; j < nodes_in_edge; j++) {
				if (i + 1 < nodes_in_edge && j + 1 < nodes_in_edge) {
					int first_index = GetNodeIndex(i, j);
					int second_index = GetNodeIndex(i, j + 1);
					int third_index = GetNodeIndex(i + 1, j);
					indexes->emplace_back(first_index);
					indexes->emplace_back(second_index);
					indexes->emplace_back(third_index);

				}

				if (i - 1 < nodes_in_edge && j - 1 < nodes_in_edge) {
					int first_index = GetNodeIndex(i, j);
					int second_index = GetNodeIndex(i, j-1 );
					int third_index = GetNodeIndex(i -1, j);
					indexes->emplace_back(first_index);
					indexes->emplace_back(second_index);
					indexes->emplace_back(third_index);

				}
			}
		}
		cloth_vertices->UpdatePositions(std::move(new_positions));
		cloth_vertices->UpdateIndices(std::move(indexes));
		UpdateSheetNormals();



		std::unique_ptr<SceneNode> cloth_node = make_unique<SceneNode>();
		cloth_node->CreateComponent<RenderingComponent>(cloth_vertices);
		cloth_node->CreateComponent<ShadingComponent>(make_unique<PhongShader>());

		cloth_node->CreateComponent < MaterialComponent >(make_unique < Material >(Material::GetDefault()) );
		cloth_node->GetComponentPtr<MaterialComponent>()->GetMaterial().SetDiffuseColor(glm::vec3(0.7,0.7,0.7));


	
		AddChild(std::move(cloth_node));
	}

	void ClothNode::UpdateSheet() {
		std::unique_ptr<PositionArray> new_positions = make_unique<PositionArray>();
		for (glm::vec3 position : current_states->positions) {
			new_positions->push_back(position);
		}
		cloth_vertices->UpdatePositions(std::move(new_positions));
		UpdateSheetNormals();


	}
	void ClothNode::UpdateSheetNormals() {
		std::unique_ptr<NormalArray> updated_normals = make_unique<NormalArray>();
		const unsigned int& vertex_amount = cloth_vertices->GetPositions().size();
		updated_normals->reserve(vertex_amount);
		for (int i = 0; i < vertex_amount; i++) {
			updated_normals->emplace_back(0, 0, 0);
		}

		for (int i = 0; i < cloth_vertices->GetIndices().size() - 2; i += 3) {
			const unsigned int& triangle_vert_index_1 = cloth_vertices->GetIndices().at(i);
			const unsigned int& triangle_vert_index_2 = cloth_vertices->GetIndices().at(i + 1);
			const unsigned int& triangle_vert_index_3 = cloth_vertices->GetIndices().at(i + 2);
			const glm::vec3& triangle_vert_1 = cloth_vertices->GetPositions().at(triangle_vert_index_1);
			const glm::vec3& triangle_vert_2 = cloth_vertices->GetPositions().at(triangle_vert_index_2);
			const glm::vec3& triangle_vert_3 = cloth_vertices->GetPositions().at(triangle_vert_index_3);


			glm::vec3& face_normal = CalculateNormal(triangle_vert_1, triangle_vert_2, triangle_vert_3);
			const float& face_area = CalculateTriangleArea(triangle_vert_1, triangle_vert_2, triangle_vert_3);
			face_area* face_normal;
			const glm::vec3& weighted_normal = face_area * face_normal;


			updated_normals->at(triangle_vert_index_1) += weighted_normal;
			updated_normals->at(triangle_vert_index_2) += weighted_normal;
			updated_normals->at(triangle_vert_index_3) += weighted_normal;
		}
		for (int normal_index = 0; normal_index < updated_normals->size(); normal_index++) {
			updated_normals->at(normal_index) = glm::normalize(updated_normals->at(normal_index));
		}
		cloth_vertices->UpdateNormals(std::move(updated_normals));
	}

	glm::vec3 ClothNode::CalculateNormal(const glm::vec3& vertex1, const glm::vec3& vertex2, const glm::vec3 vertex3) {
		const glm::vec3& vect1 = vertex2 - vertex1;
		const glm::vec3& vect2 = vertex3 - vertex1;

		return -glm::normalize(glm::cross(vect1, vect2));
	}

	float ClothNode::CalculateTriangleArea(const glm::vec3& vertex1, const glm::vec3& vertex2, const glm::vec3 vertex3) {
		const glm::vec3& vect1 = vertex2 - vertex1;
		const glm::vec3& vect2 = vertex3 - vertex1;
		return glm::length(glm::cross(vect1, vect2));
	}

	void ClothNode::FixCenter(int spacing ) {
		for (int i = spacing; i < nodes_in_edge - spacing; i++) {
			for (int j = spacing; j < nodes_in_edge - spacing; j++) {
				int index = GetNodeIndex(i, j);
				forces->FixParticle(index);
			}
		}
		CreateTable(spacing);
	}
	void ClothNode::CreateTable(int& spacing) {
		std::unique_ptr<VertexObject> table = PrimitiveFactory::CreateQuad();

		double cloth_spacing = 0.01;

		std::unique_ptr<PositionArray> table_pos = make_unique<PositionArray>();


		int lower_left = GetNodeIndex(nodes_in_edge - spacing - 1,spacing );
		int lower_right = GetNodeIndex(nodes_in_edge - spacing - 1, nodes_in_edge - spacing - 1);
		int upper_left = GetNodeIndex(spacing, spacing);
		int upper_right = GetNodeIndex(spacing, nodes_in_edge - spacing - 1);

		const glm::vec3& lower_left_table_point = current_states->positions.at(lower_left) + glm::vec3(0, -cloth_spacing, 0);
		const glm::vec3& lower_right_table_point = current_states->positions.at(lower_right) + glm::vec3(0, -cloth_spacing, 0);
		const glm::vec3& upper_left_table_point = current_states->positions.at(upper_right) + glm::vec3(0, -cloth_spacing, 0);
		const glm::vec3& upper_right_table_point = current_states->positions.at(upper_left) + glm::vec3(0, -cloth_spacing, 0);


		table_pos->push_back(lower_left_table_point);
		table_pos->push_back(lower_right_table_point);
		table_pos->push_back(upper_left_table_point);
		table_pos->push_back(upper_right_table_point);


		table->UpdatePositions(std::move(table_pos));

		
		std::unique_ptr<SceneNode> table_node = make_unique<SceneNode>();
		table_node->CreateComponent<RenderingComponent>(std::move(table));
		std::shared_ptr<PhongShader> phong_shader = std::make_shared<PhongShader>();

		table_node->CreateComponent<ShadingComponent>(phong_shader);

		table_node->CreateComponent < MaterialComponent >(make_unique < Material >(Material::GetDefault()));
		table_node->GetComponentPtr<MaterialComponent>()->GetMaterial().SetDiffuseColor(glm::vec3(0.5, 0.3, 0.7));

		AddChild(std::move(table_node));



		const glm::vec3& cloth_center = 1.0f / 4 * (lower_left_table_point + lower_right_table_point + upper_left_table_point + upper_right_table_point);

		float sidelength = glm::length(lower_left_table_point - lower_right_table_point);

		float radius = sidelength / 5;

		
		std::unique_ptr<VertexObject> table_stool = PrimitiveFactory::CreateCylinder(radius,dimension,10);
		
		std::unique_ptr<SceneNode> stool_node = make_unique<SceneNode>();

		stool_node->CreateComponent<RenderingComponent>(std::move(table_stool));

		stool_node->CreateComponent<ShadingComponent>(phong_shader);

		stool_node->CreateComponent < MaterialComponent >(make_unique < Material >(Material::GetDefault()));
		stool_node->GetComponentPtr<MaterialComponent>()->GetMaterial().SetDiffuseColor(glm::vec3(0.1, 0.3, 0.1));


		stool_node->GetTransform().SetPosition(cloth_center + glm::vec3(0,-dimension,0));

		AddChild(std::move(stool_node));


	



		

	}

	void ClothNode::ResetSystem() {

		ParticleState reset_sates;
		double side_scale_factor = 1.0 / nodes_in_edge * dimension;
		for (int y = 0; y < nodes_in_edge; y++) {
			double y_pos = side_scale_factor * y;

			for (int x = 0; x < nodes_in_edge; x++) {
				double x_pos = side_scale_factor * x;

				glm::vec3 particle_pos(x_pos, 0, y_pos);
				reset_sates.positions.push_back(particle_pos);
				reset_sates.velocities.emplace_back(0, 0, 0);
			}
		}



		UpdateStates(reset_sates);

	}



}  // namespace GLOO
