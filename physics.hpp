#ifndef PHYSICS_H
#define PHYSICS_H

#ifndef JPH_DEBUG_RENDERER
#define JPH_DEBUG_RENDERER
#endif

#include "jolt_implementation.hpp"
#include "sbpt_generated_includes.hpp"

#include "Jolt/Physics/Character/CharacterVirtual.h"
#include "Jolt/Physics/StateRecorderImpl.h"

#include <chrono>

struct PhysicsFrame {
    JPH::StateRecorderImpl &physics_state;
};

class Physics {
  public:
    Physics();
    ~Physics();

    JPH::PhysicsSystem physics_system;

    void update_specific_character(float delta_time, uint64_t client_id);
    void update_characters_only(float delta_time);
    void update(float delta_time);

    JPH::BodyID sphere_id; // should be removed in a real program
    void refresh_contacts(JPH::Ref<JPH::CharacterVirtual>);
    void load_model_into_physics_world(std::vector<std::vector<glm::vec3>> &ordered_vertex_positions_for_each_mesh);
    std::unordered_map<unsigned, JPH::Ref<JPH::CharacterVirtual>> client_id_to_physics_character;
    void create_character(unsigned int client_id);
    void delete_character(unsigned int client_id);

  private:
    void initialize_engine();
    void initialize_world_objects();
    void clean_up_world();

    const unsigned int cMaxBodies = 1024;
    const unsigned int cNumBodyMutexes = 0;
    const unsigned int cMaxBodyPairs = 1024;
    const unsigned int cMaxContactConstraints = 1024;
    const int cCollisionSteps = 1;

    const float character_height = 2.0f;
    const float character_radius = 0.5f;

    JPH::TempAllocatorImpl *temp_allocator;
    JPH::JobSystemThreadPool *job_system;
    MyBodyActivationListener *body_activation_listener;
    MyContactListener *contact_listener;

    BPLayerInterfaceImpl broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
    ObjectLayerPairFilterImpl object_vs_object_layer_filter;

    std::vector<JPH::BodyID> created_body_ids;
};

#endif
