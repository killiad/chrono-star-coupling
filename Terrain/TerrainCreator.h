#ifndef TERRAIN_CREATOR_H 
#define TERRAIN_CREATOR_H 

#include <memory>

#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace chrono{
namespace vehicle{

enum class TerrainModel { RIGID, SCM_DEFORMABLE, FLAT };

class TerrainCreator{

    public:
        TerrainCreator(std::shared_ptr<ChTrackedVehicle> veh, TerrainModel type, std::string filename);

        inline std::shared_ptr<ChTerrain> GetTerrain() { return terrain; }

    private:
        void InitializeRigid();

        void InitializeSCMDeformable();

        void InitializeFlat();

        std::shared_ptr<ChTerrain> terrain;

        std::shared_ptr<ChTrackedVehicle> vehicle;

        TerrainModel terrain_model;

        std::string file;
};
        
}//end namespace vehicle
}//end namespace chrono

#endif
