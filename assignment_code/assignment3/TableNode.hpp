#ifndef TABLE_NODE_H_
#define TABLE_NODE_H_


#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"


namespace GLOO {
    class TableNode : public SceneNode {
    public:
        TableNode(glm::vec3& center, float sidelenght, glm::vec3& normal_direction);



    private:
        void TableNode::CreateBasicTable(float sidelenght);
        std::shared_ptr<VertexObject> TableNode::CreateBasicPlane(float sidelenght);
        std::shared_ptr<VertexObject> TableNode::CreateBasicRect(float length, float width);

        glm::vec3 normal_direction;
        void RotateTable();
        float CalculateRotAngle();

        glm::quat RotationTransform(const glm::vec3& axis, float angle);

    };
}  // namespace GLOO

#endif
