#ifndef TERRAIN_CREATOR_RIDID_H
#define TERRAIN_CREATOR_RIDID_H 

#include <memory>
#include "TerrainCreator.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "../CSV/CSVReader.h"

namespace chrono{
namespace vehicle{

class TerrainCreator_SCMDeformable : TerrainCreator {

    public:
        TerrainCreator_SCMDeformable(std::string filename, std::shared_ptr<TrackedVehicle> veh); 

        virtual ~TerrainCreator_SCMDeformable() {}

        virtual std::shared_ptr<ChTerrain> GetTerrain() override; 

    protected:
        std::shared_ptr<SCMDeformableTerrain> terrain;
};
        
}//end namespace vehicle
}//end namespace chrono

#endif
