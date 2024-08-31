#include "physics.hpp"

// The Jolt headers don't include Jolt.h. Always include Jolt.h before including
// any other Jolt header. You can use Jolt.h in your precompiled header to speed
// up compilation.
#include "Jolt/Physics/Character/CharacterVirtual.h"
#include "Jolt/Physics/Collision/Shape/CapsuleShape.h"
#include "Jolt/Physics/Collision/Shape/ConvexHullShape.h"
#include "Jolt/Physics/Collision/Shape/MeshShape.h"
#include "Jolt/Physics/StateRecorder.h"
#include <stdexcept>

#include "formatting.hpp"

//// Disable common warnings triggered by Jolt, you can use
/// JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to
/// store and restore the warning state
// JPH_SUPPRESS_WARNINGS

Physics::Physics() {
    this->initialize_engine();
    this->initialize_world_objects();
}

Physics::~Physics() { this->clean_up_world(); }

void Physics::initialize_engine() {
    JPH::RegisterDefaultAllocator();

    JPH::Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    // dynamic allocation, we don't worry about the rule of three since we never
    // copy the physics system, there is only ever one instance
    temp_allocator = new JPH::TempAllocatorImpl(10 * 1024 * 1024);
    job_system = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
                                              JPH::thread::hardware_concurrency() - 1);

    physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, broad_phase_layer_interface,
                        object_vs_broadphase_layer_filter, object_vs_object_layer_filter);

    body_activation_listener = new MyBodyActivationListener();
    contact_listener = new MyContactListener();

    physics_system.SetBodyActivationListener(body_activation_listener);
    physics_system.SetContactListener(contact_listener);

    spdlog::get(Systems::physics)->info("jolt physics engine successfully initialized");
}

JPH::Array<JPH::Vec3> fibonacci_sphere(int num_samples) {
    JPH::Array<JPH::Vec3> points;
    float phi = M_PI * (std::sqrt(5.0) - 1.0);

    for (int i = 0; i < num_samples; i++) {
        float y = 1 - ((float) i / ((float) num_samples - 1)) * 2;
        float radius = std::sqrt(1 - y * y);
        float theta = phi * (float) i;

        float x = std::cos(theta) * radius;
        float z = std::sin(theta) * radius;
        points.emplace_back(x, y, z);
    }
    return points;
}

void Physics::initialize_world_objects() {

    physics_system.SetGravity(JPH::Vec3(0, -40.0, 0));

    JPH::BodyInterface &body_interface = physics_system.GetBodyInterface();

    JPH::Array<JPH::Vec3> vertices = fibonacci_sphere(60);
    JPH::ConvexHullShapeSettings low_poly_ball_settings(vertices, JPH::cDefaultConvexRadius);
    JPH::ShapeSettings::ShapeResult ball_shape_result = low_poly_ball_settings.Create();

    if (!ball_shape_result.IsValid()) {
        throw std::runtime_error("ball shape is invalid");
    }

    JPH::ShapeRefC ball_shape = ball_shape_result.Get(); // We don't expect an error here, but you can check
    // floor_shape_result for HasError() / GetError()

    JPH::BodyCreationSettings ball_creation_settings(ball_shape, JPH::RVec3(5.0, 20.0, 5.0), JPH::Quat::sIdentity(),
                                                     JPH::EMotionType::Dynamic, Layers::MOVING);
    JPH::Body *ball = body_interface.CreateBody(ball_creation_settings); // Note that if we run out of bodies this can
    // return nullptr
    body_interface.AddBody(ball->GetID(), JPH::EActivation::Activate);
    created_body_ids.push_back(ball->GetID());
    body_interface.SetLinearVelocity(ball->GetID(), JPH::Vec3(0.0f, -5.0f, 0.0f));
}

/**
 * \brief For every mesh in this model, we create a physics object that
 * represents the mesh
 */
void Physics::load_model_into_physics_world(Model &model) {

    JPH::BodyInterface &body_interface = physics_system.GetBodyInterface();

    for (auto mesh : model.meshes) {

        JPH::TriangleList triangles;
        // Iterate through the indices 3 at a time
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            glm::vec3 v1 = mesh.vertex_positions[mesh.indices[i]];
            glm::vec3 v2 = mesh.vertex_positions[mesh.indices[i + 1]];
            glm::vec3 v3 = mesh.vertex_positions[mesh.indices[i + 2]];

//            // Output the triangle vertices
//            std::cout << "Triangle " << i / 3 << ": " << std::endl;
//            std::cout << "  Vertex 1: (" << v1.x << ", " << v1.y << ", " << v1.z << ")" << std::endl;
//            std::cout << "  Vertex 2: (" << v2.x << ", " << v2.y << ", " << v2.z << ")" << std::endl;
//            std::cout << "  Vertex 3: (" << v3.x << ", " << v3.y << ", " << v3.z << ")" << std::endl;

            JPH::Float3 jv1 = JPH::Float3(v1.x, v1.y, v1.z);
            JPH::Float3 jv2 = JPH::Float3(v2.x, v2.y, v2.z);
            JPH::Float3 jv3 = JPH::Float3(v3.x, v3.y, v3.z);

            JPH::Triangle tri = JPH::Triangle(jv1, jv2, jv3);

            triangles.push_back(tri);
        }

        JPH::MeshShapeSettings settings = JPH::MeshShapeSettings(triangles);

        JPH::Ref<JPH::Shape> mesh_shape;

        // Create shape
        JPH::Shape::ShapeResult result = settings.Create();
        if (result.IsValid()) {
            mesh_shape = result.Get();
        } else {
            throw std::runtime_error("couldn't get resulting shape");
        }

        JPH::BodyCreationSettings mesh_settings(mesh_shape, JPH::RVec3(0.0, 0.0, 0.0), JPH::Quat::sIdentity(),
                                                JPH::EMotionType::Static, Layers::NON_MOVING);
        JPH::Body *mesh_body = body_interface.CreateBody(mesh_settings); // Note that if we run out of bodies this can
        // return nullptr
        body_interface.AddBody(mesh_body->GetID(), JPH::EActivation::DontActivate);
        created_body_ids.push_back(mesh_body->GetID());
    }
    spdlog::get(Systems::physics)->info("successfully loaded in model");
}

/**
 * \brief create character controller for a user
 * \todo do I have to account for dynamic memory? Come back when you know what
 * ref is
 */
void Physics::create_character(unsigned int client_id) {
    JPH::Ref<JPH::CharacterVirtualSettings> settings = new JPH::CharacterVirtualSettings();
    settings->mShape = new JPH::CapsuleShape(0.5f * this->character_height, this->character_radius);
    settings->mSupportingVolume = JPH::Plane(JPH::Vec3::sAxisY(),
                                             -this->character_radius); // Accept contacts that touch the
    // lower sphere of the capsule

    JPH::Ref<JPH::CharacterVirtual> character =
            new JPH::CharacterVirtual(settings, JPH::RVec3(0.0f, 10.0f, 0.0f), JPH::Quat::sIdentity(), &physics_system);

    client_id_to_physics_character[client_id] = character;
    spdlog::get(Systems::physics)->info("just created a new physics character with id {}", client_id);
}

void Physics::delete_character(unsigned int client_id) { client_id_to_physics_character.erase(client_id);
    spdlog::get(Systems::physics)->info("just deleted physics character with id {}", client_id);
}

/**
 * \brief updates the objects part of this physics simulation
 */
void Physics::update(float delta_time) {
    this->update_characters_only(delta_time); // accounts for predicted inputs
    physics_system.Update(delta_time, cCollisionSteps, temp_allocator, job_system);
    // JPH::StateRecorderImpl physics_state_after_update;
    // this->physics_state_history.push
    // this->physics_system.SaveState(physics_state_after_update);
    // PhysicsFrame physics_frame_after_update = {physics_state_after_update};
    // this->physics_frames.put(physics_frame_after_update);
}

void Physics::refresh_contacts(JPH::Ref<JPH::CharacterVirtual> character) {
    // character->RefreshContacts(this->broad_phase_layer_interface, this->object_vs_object_layer_filter, const
    // BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter, TempAllocator &inAllocator)
    //
    character->RefreshContacts(physics_system.GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
                               physics_system.GetDefaultLayerFilter(Layers::MOVING), {}, {}, *temp_allocator);
}

/**
 * this function is designed to be called in a multithreaded context, you must use a mutex to ensure safe usage
 * it works by keeping track of the last time its called so that reconciliations events and regular prediction updates
 * use the correct deltas.
 * the ability to use a custom delta time allows for reconciliation to use previous deltas
 */
void Physics::update_characters_only(float delta_time) {

    // Settings for our update function
    JPH::CharacterVirtual::ExtendedUpdateSettings update_settings;
    // update_settings.mStickToFloorStepDown = character->GetUp() *
    // update_settings.mStickToFloorStepDown.Length();
    // update_settings.mWalkStairsStepUp = character->GetUp() *
    // update_settings.mWalkStairsStepUp.Length();
    //
    for (const auto &pair: client_id_to_physics_character) {
        JPH::Ref<JPH::CharacterVirtual> character = pair.second;
        character->ExtendedUpdate(delta_time, -character->GetUp() * physics_system.GetGravity().Length(),
                                  update_settings, physics_system.GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
                                  physics_system.GetDefaultLayerFilter(Layers::MOVING), {}, {}, *temp_allocator);
    }
    // this->physics_state_recorder
}

void Physics::update_specific_character(float delta_time, uint64_t client_id) {

    // Safely access and call the function if the key exists
    auto potential_character_pair = client_id_to_physics_character.find(client_id);
    if (potential_character_pair != client_id_to_physics_character.end()) {
        JPH::Ref<JPH::CharacterVirtual> requested_character =
            potential_character_pair->second; // Call the function with an argument
        JPH::CharacterVirtual::ExtendedUpdateSettings update_settings;
        spdlog::info("running physics update on {} with delta_time {} pos before {}", client_id, delta_time,
                     requested_character->GetPosition());
        requested_character->ExtendedUpdate(
            delta_time, -requested_character->GetUp() * physics_system.GetGravity().Length(), update_settings,
            physics_system.GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
            physics_system.GetDefaultLayerFilter(Layers::MOVING), {}, {}, *temp_allocator);
        spdlog::info("pos after {}", requested_character->GetPosition());
    } else {
        std::cout << "tried to update specific character in physics, but couldn't find them in the map" << std::endl;
    }
}

void Physics::clean_up_world() {
    JPH::BodyInterface &body_interface = physics_system.GetBodyInterface();

    for (auto body_id: created_body_ids) {
        body_interface.RemoveBody(body_id);
        body_interface.DestroyBody(body_id);
    }

    JPH::UnregisterTypes();

    // de-allocate dynamic memory
    delete temp_allocator;
    delete job_system;
    delete body_activation_listener;
    delete contact_listener;

    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;

    spdlog::get(Systems::physics)->info("successfully cleaned up physics world");
}
