#include "TableNode.hpp"

#include "gloo/SceneNode.hpp"

#include "gloo/shaders/SimpleShader.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include <cmath>


namespace GLOO {
	TableNode::TableNode(glm::vec3& center, float sidelength, glm::vec3& normal_direction):normal_direction(glm::normalize(normal_direction)) {
		float epsilon = 0.01;
		CreateBasicTable(sidelength);
		RotateTable();
		GetTransform().SetPosition(center - normal_direction * epsilon);
	}


	void TableNode::CreateBasicTable(float sidelenght) {

		float table_height = 6;

		float pi = 3.1415;


		std::unique_ptr<SceneNode> box_node = make_unique<SceneNode>();

		std::shared_ptr<VertexObject> table_vertices = CreateBasicPlane(sidelenght);


		std::shared_ptr<VertexObject> height_vertices = CreateBasicRect(sidelenght, table_height);


		std::unique_ptr<SceneNode> top_face_node = make_unique<SceneNode>();
		top_face_node->GetTransform().SetPosition(glm::vec3(0, table_height / 2,0));
		top_face_node->CreateComponent<RenderingComponent>(table_vertices);
		box_node->AddChild(std::move(top_face_node));

		std::unique_ptr<SceneNode> bot_face_node = make_unique<SceneNode>();
		bot_face_node->GetTransform().SetPosition(glm::vec3(0, -table_height / 2,0));
		bot_face_node->CreateComponent<RenderingComponent>(table_vertices);
		box_node->AddChild(std::move(bot_face_node));


		std::unique_ptr<SceneNode> left_face_node = make_unique<SceneNode>();
		left_face_node->GetTransform().SetRotation(glm::vec3(0, 0, 1), pi / 2);
		left_face_node->GetTransform().SetRotation(RotationTransform(glm::vec3(1, 0, 0), pi / 2)* left_face_node->GetTransform().GetRotation());
		left_face_node->GetTransform().SetPosition(glm::vec3(-sidelenght / 2,  0,0));
		left_face_node->CreateComponent<RenderingComponent>(height_vertices);
		box_node->AddChild(std::move(left_face_node));



		std::unique_ptr<SceneNode> right_face_node = make_unique<SceneNode>();
		right_face_node->GetTransform().SetRotation(glm::vec3(0, 0, 1), pi / 2);
		right_face_node->GetTransform().SetRotation(RotationTransform(glm::vec3(1, 0, 0), pi / 2) * right_face_node->GetTransform().GetRotation());
		right_face_node->GetTransform().SetPosition(glm::vec3(sidelenght / 2, 0, 0));
		right_face_node->CreateComponent<RenderingComponent>(height_vertices);
		box_node->AddChild(std::move(right_face_node));


		std::unique_ptr<SceneNode> front_face_node = make_unique<SceneNode>();
		front_face_node->GetTransform().SetRotation(glm::vec3(1, 0, 0), pi / 2);
		front_face_node->GetTransform().SetPosition(glm::vec3(0, 0, sidelenght / 2));
		front_face_node->CreateComponent<RenderingComponent>(height_vertices);
		box_node->AddChild(std::move(front_face_node));


		std::unique_ptr<SceneNode> back_face_node = make_unique<SceneNode>();
		back_face_node->GetTransform().SetRotation(glm::vec3(1, 0, 0), pi / 2);
		back_face_node->GetTransform().SetPosition(glm::vec3(0, 0, -sidelenght / 2));
		back_face_node->CreateComponent<RenderingComponent>(height_vertices);
		box_node->AddChild(std::move(back_face_node));



		for (int i = 0; i < box_node->GetChildrenCount(); i++)
		{

			SceneNode& face_node = box_node->GetChild(i);
			face_node.CreateComponent<ShadingComponent>(std::move(make_unique<PhongShader>()));
			face_node.CreateComponent < MaterialComponent >(make_unique < Material >(Material::GetDefault()));
			face_node.GetComponentPtr<MaterialComponent>()->GetMaterial().SetDiffuseColor(glm::vec3(0.8, 0.8, 0.8));


		}
		box_node->GetTransform().SetPosition(glm::vec3(0, -table_height / 2, 0));

		AddChild(std::move(box_node));




	

	}

	std::shared_ptr<VertexObject> TableNode::CreateBasicPlane(float sidelength) {

		return CreateBasicRect(sidelength, sidelength);
	}

	std::shared_ptr<VertexObject> TableNode::CreateBasicRect(float length, float width) {

		std::unique_ptr<PositionArray> positions = make_unique<PositionArray>();
		positions->emplace_back(-length / 2, 0.0f, -width / 2);
		positions->emplace_back(length / 2, 0.0f, -width / 2);
		positions->emplace_back(length / 2, 0.0f, width / 2);
		positions->emplace_back(-length / 2, 0.0f, width / 2);


		std::unique_ptr<IndexArray> indices = make_unique<IndexArray>();
		indices->insert(indices->end(), { 0, 1, 2 });
		indices->insert(indices->end(), { 0, 2, 3 });


		std::unique_ptr<NormalArray> normals = make_unique<NormalArray>();
		for (int t = 0; t < 4; t++)
			normals->emplace_back(0.0f, 1.0f, 0.0f);
		std::unique_ptr<TexCoordArray> tex_coords = make_unique<TexCoordArray>();
		tex_coords->emplace_back(0.0f, 0.0f);
		tex_coords->emplace_back(1.0f, 0.0f);
		tex_coords->emplace_back(1.0f, 1.0f);
		tex_coords->emplace_back(0.0f, 1.0f);

		std::unique_ptr<VertexObject> table_obj = make_unique<VertexObject>();
		table_obj->UpdatePositions(std::move(positions));
		table_obj->UpdateNormals(std::move(normals));
		table_obj->UpdateIndices(std::move(indices));
		table_obj->UpdateTexCoord(std::move(tex_coords));

		return table_obj;
	}





	void TableNode::RotateTable() {
		if (normal_direction == glm::vec3(0, 1, 0)) {
			return;
		}

		glm::vec3 default_normal(0, 1.0, 0);

		glm::vec3& rot_axis = glm::normalize(glm::cross(default_normal, normal_direction));

		GetTransform().SetRotation(rot_axis, CalculateRotAngle());

	}





	float TableNode::CalculateRotAngle() {


		glm::vec3 default_normal(0, 1.0, 0);

		float cos_value = glm::dot(default_normal, normal_direction);

		return glm::acos(cos_value);

	}
	glm::quat TableNode::RotationTransform(const glm::vec3& axis, float angle) {

		return glm::quat(cosf(angle / 2), axis.x * sinf(angle / 2),
			axis.y * sinf(angle / 2), axis.z * sinf(angle / 2));
	}







}

