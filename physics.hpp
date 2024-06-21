#ifndef PHYSICS_H
#define PHYSICS_H

#ifndef JPH_DEBUG_RENDERER
#define JPH_DEBUG_RENDERER
#endif

#include "jolt_implementation.hpp"
#include "../../graphics/graphics.hpp"
#include "Jolt/Physics/Character/CharacterVirtual.h"
#include "Jolt/Physics/StateRecorderImpl.h"
#include "../../networked_input_snapshot/networked_input_snapshot.hpp"
#include "../../expiring_data_container/expiring_data_container.hpp"
#include "../../ring_buffer/ring_buffer.hpp"
#include <chrono>

struct PhysicsFrame {
    JPH::StateRecorderImpl &physics_state;
};

class Physics {
  public:
    Physics();
    ~Physics();

    JPH::PhysicsSystem physics_system;

    // RingBuffer<PhysicsFrame> physics_frames;

    void update_characters_only(float delta_time);
    void update(float delta_time);

    JPH::BodyID sphere_id; // should be removed in a real program
    std::unordered_map<uint64_t, JPH::Ref<JPH::CharacterVirtual>> client_id_to_physics_character;
    void refresh_contacts(JPH::Ref<JPH::CharacterVirtual>);
    // JPH::Ref<JPH::CharacterVirtual> character;

    void load_model_into_physics_world(Model *model);
    void create_character(uint64_t client_id);
    void delete_character(uint64_t client_id);

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
