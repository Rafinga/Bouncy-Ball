#ifndef TABLE_NODE_H_
#define TABLE_NODE_H_


#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"


namespace GLOO {
    class TableNode : public SceneNode {
    public:
        TableNode(glm::vec3& center, float sidelenght, glm::vec3& normal_direction);



    private:
        std::unique_ptr<VertexObject> TableNode::CreateBasicTable(float sidelenght);
        glm::vec3 normal_direction;
    };
}  // namespace GLOO

#endif
