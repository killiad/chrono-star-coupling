#ifndef TERRAIN_CREATOR_H 
#define TERRAIN_CREATOR_H 

#include <memory>

#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace chrono{
namespace vehicle{

enum class TerrainType { RIGID, SCM_DEFORMABLE };

class TerrainCreator{

    public:
        TerrainCreator(std::shared_ptr<ChTrackedVehicle> veh, TerrainType type, std::string filename);

        inline std::shared_ptr<ChTerrain> GetTerrain() { return terrain; }

    private:
        void InitializeRigid();

        void InitializeSCMDeformable();

        std::shared_ptr<ChTerrain> terrain;

        std::shared_ptr<ChTrackedVehicle> vehicle;

        TerrainType terrain_type;

        std::string file;
};
        
}//end namespace vehicle
}//end namespace chrono

#endif
