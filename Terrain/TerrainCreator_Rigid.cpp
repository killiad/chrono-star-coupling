#include "TerrainCreator_Rigid.h"

namespace chrono{
namespace vehicle{
    
TerrainCreator_Rigid::TerrainCreator_Rigid(std::string filename, std::shared_ptr<TrackedVehicle> veh) : TerrainCreator(filename, veh),
    terrain(chrono_types::make_shared<RigidTerrain>(vehicle->GetSystem(), GetDataFile(file))) { terrain->Initialize(); }

std::shared_ptr<ChTerrain> TerrainCreator_Rigid::GetTerrain() { return terrain; }

}
}
