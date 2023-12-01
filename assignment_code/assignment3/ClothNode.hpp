#ifndef CLOTH_NODE_H_
#define CLOTH_NODE_H_

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"
#include "ParticleState.hpp"
#include "PendulumSystem.hpp"
#include "IntegratorBase.hpp"

#include <string>
#include <vector>

namespace GLOO {
	class ClothNode : public SceneNode {
 public:
  
	 std::unique_ptr<ParticleState> current_states;
	 double start_time = 0;
	 double total_time_elapsed = 0;
	 double dimension = 2;
	 unsigned int nodes_in_edge;



	 ClothNode(
		 std::unique_ptr<PendulumSystem> forces, 
		 std::unique_ptr<IntegratorBase<PendulumSystem, ParticleState>> integrator,
		 double dt,
		 unsigned int nodes_in_edge = 4);

	 void Update(double delta_time) override;
	 void UpdateStates(ParticleState& new_states);



	 std::unique_ptr<PendulumSystem> forces;
	 std::unique_ptr<IntegratorBase<PendulumSystem,ParticleState>> integrator;
	 double dt;
	 float spring_constant = 630;
	 float default_mass = 1;


private:
	bool in_mid_of_t_step = false;
	ParticleState full_t_step;
	ParticleState mid_step_start_state;
	void AddParticle(const glm::vec3& starting_position);
	void CreateSheet();

	void ResetSystem();

	void UpdateSheet();
	void CreateGrid();
	void UpdateSheetNormals();
	void CreateStructuralSprings();
	void CreateShearSprings();
	void CreateFlexSprings();
	int GetNodeIndex(int row, int col) {
		return (row)*nodes_in_edge + col;
	}
	double GetNodeDistance(int row1, int col1, int row2, int col2);
	std::shared_ptr<VertexObject> cloth_vertices;
	std::shared_ptr<VertexObject> wireframe_vertices;
	glm::vec3 CalculateNormal(const glm::vec3& vertex1, const glm::vec3& vertex2, const glm::vec3 vertex3);
	float CalculateTriangleArea(const glm::vec3& vertex1, const glm::vec3& vertex2, const glm::vec3 vertex3);
	void FixCenter(int spacing);
	void CreateTable(int& spacing);

	bool prev_released = true;

};
}  // namespace GLOO

#endif
