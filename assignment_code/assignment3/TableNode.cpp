#include "TableNode.hpp"

#include "gloo/SceneNode.hpp"

#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"

#include <cmath>


namespace GLOO {
	TableNode::TableNode(glm::vec3& center, float sidelenght, glm::vec3& normal_direction):normal_direction(glm::normalize(normal_direction)) {
		
		GetTransform().SetPosition(center);

		std::unique_ptr<VertexObject> table_vertices = CreateBasicTable(sidelenght);
		CreateComponent<RenderingComponent>(std::move(table_vertices));
		CreateComponent<ShadingComponent>(std::move(make_unique<PhongShader>()));
		




	}
	std::unique_ptr<VertexObject> TableNode::CreateBasicTable(float sidelenght) {
		
		std::unique_ptr<PositionArray> positions = make_unique<PositionArray>();
		positions->emplace_back(-sidelenght / 2,0.0f, -sidelenght / 2);
		positions->emplace_back(sidelenght / 2,0.0f, -sidelenght / 2);
		positions->emplace_back(sidelenght / 2,0.0f, sidelenght / 2);
		positions->emplace_back(-sidelenght /2,0.0f, sidelenght / 2);

		std::unique_ptr<IndexArray> indices = make_unique<IndexArray>();
		indices->insert(indices->end(), { 0, 1, 2 });
		indices->insert(indices->end(), { 0, 2, 3 });


		std::unique_ptr<NormalArray> normals = make_unique<NormalArray>();
		for (int t = 0; t < 4; t++)
			normals->emplace_back(0.0f, 1.0f,0.0f);
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





}

