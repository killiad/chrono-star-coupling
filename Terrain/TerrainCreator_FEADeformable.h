#ifndef TERRAIN_CREATOR_FEADEFORMABLE_H
#define TERRAIN_CREATOR_FEADEFORMABLE_H 

#include <memory>
#include "TerrainCreator.h"
#include "chrono_vehicle/terrain/FEADeformableTerrain.h"
#include "../CSV/CSVReader.h"

namespace chrono{
namespace vehicle{

class TerrainCreator_FEADeformable : TerrainCreator {

    public:
        TerrainCreator_FEADeformable(std::string filename, std::shared_ptr<TrackedVehicle> veh); 

        virtual ~TerrainCreator_FEADeformable() {}

        virtual std::shared_ptr<ChTerrain> GetTerrain() override; 

    protected:
        std::shared_ptr<FEADeformableTerrain> terrain;
};
        
}//end namespace vehicle
}//end namespace chrono

#endif
