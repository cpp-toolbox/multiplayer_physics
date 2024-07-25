#include "spdlog/fmt/ostr.h" // must be included

#include "formatting.hpp"

// Overload the << operator for the Physics class
std::ostream &operator<<(std::ostream &os, const Physics &physics) {
    os << "vvv Physics State vvv"
       << "\n";
    for (const auto &pair: physics.client_id_to_physics_character) {
        uint64_t client_id = pair.first;
        JPH::Ref<JPH::CharacterVirtual> character = pair.second;

        // Get position and velocity
        JPH::Vec3 position = character->GetPosition();
        JPH::Vec3 velocity = character->GetLinearVelocity();

        os << "Client ID: " << client_id << "\n";
        os << "Position: (" << position.GetX() << ", " << position.GetY() << ", " << position.GetZ() << ")\n";
        os << "Velocity: (" << velocity.GetX() << ", " << velocity.GetY() << ", " << velocity.GetZ() << ")\n";
        os << "On Ground: "
           << (character->GetGroundState() == JPH::CharacterVirtual::EGroundState::OnGround ? "1" : "0") << "\n";
    }
    return os;
}
