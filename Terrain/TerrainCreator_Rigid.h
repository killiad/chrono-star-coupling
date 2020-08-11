#ifndef TERRAIN_CREATOR_RIGID_H
#define TERRAIN_CREATOR_RIGID_H 

#include <memory>
#include "TerrainCreator.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

namespace chrono{
namespace vehicle{

class TerrainCreator_Rigid : TerrainCreator {

    public:
        TerrainCreator_Rigid(std::string filename, std::shared_ptr<TrackedVehicle> veh); 

        virtual ~TerrainCreator_Rigid() {}

        virtual std::shared_ptr<ChTerrain> GetTerrain() override; 

    protected:
        std::shared_ptr<RigidTerrain> terrain;
};
        
}//end namespace vehicle
}//end namespace chrono

#endif
