#include "SimulationApp.hpp"

#include "glm/gtx/string_cast.hpp"

#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/CameraComponent.hpp"
#include "gloo/components/LightComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/MeshLoader.hpp"
#include "gloo/lights/PointLight.hpp"
#include "gloo/lights/AmbientLight.hpp"
#include "gloo/cameras/ArcBallCameraNode.hpp"
#include "gloo/debug/AxisNode.hpp"

#include "IntegratorFactory.hpp"
#include "ForwardEulerIntegrator.hpp"
#include "TrapezoidalIntegrator.hpp"
#include "BouncyNode.hpp"


namespace GLOO {
    SimulationApp::SimulationApp(const std::string& app_name,
        glm::ivec2 window_size,
        IntegratorType integrator_type,
        float integration_step)
        : Application(app_name, window_size),
        integrator_type_(integrator_type),
        integration_step_(integration_step) {
        // TODO: remove the following two lines and use integrator type and step to
        // create integrators; the lines below exist only to suppress compiler
        // warnings.
    }

    void SimulationApp::SetupScene() {
        SceneNode& root = scene_->GetRootNode();

        auto camera_node = make_unique<ArcBallCameraNode>(45.f, 0.75f, 5.0f);
        scene_->ActivateCamera(camera_node->GetComponentPtr<CameraComponent>());
        root.AddChild(std::move(camera_node));

        root.AddChild(make_unique<AxisNode>('A'));

        auto ambient_light = std::make_shared<AmbientLight>();
        ambient_light->SetAmbientColor(glm::vec3(0.2f));
        root.CreateComponent<LightComponent>(ambient_light);

        auto point_light = std::make_shared<PointLight>();
        point_light->SetDiffuseColor(glm::vec3(0.8f, 0.8f, 0.8f));
        point_light->SetSpecularColor(glm::vec3(1.0f, 1.0f, 1.0f));
        point_light->SetAttenuation(glm::vec3(1.0f, 0.09f, 0.032f));
        auto point_light_node = make_unique<SceneNode>();
        point_light_node->CreateComponent<LightComponent>(point_light);
        point_light_node->GetTransform().SetPosition(glm::vec3(0.0f, 2.0f, 4.f));
        root.AddChild(std::move(point_light_node));


        glm::vec3 center(1, 1, 0);
        float radius(1.0f);
        int n = 20;

        //glm::vec3 floor_normal(0, 1, 0);
        //glm::vec3 floor_point(0, -1, 0) // Maybe allow passing these in to BouncyNode

        std::unique_ptr<BouncyNode> circle_bouncy_node = make_unique<BouncyNode>(
            std::move(IntegratorFactory::CreateIntegrator<BouncySystem, ParticleState>(integrator_type_)),
            integration_step_,
            center, radius, n
        );
        root.AddChild(std::move(circle_bouncy_node)); //
    }
}  // namespace GLOO
