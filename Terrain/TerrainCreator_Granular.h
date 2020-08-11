#ifndef TERRAIN_CREATOR_GRANULAR_H
#define TERRAIN_CREATOR_GRANULAR_H 

#include <memory>
#include "TerrainCreator.h"
#include "../CSV/CSVReader.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"

namespace chrono{
namespace vehicle{

class TerrainCreator_Granular : TerrainCreator {

    public:
        TerrainCreator_Granular(std::string filename, std::shared_ptr<TrackedVehicle> veh); 

        virtual ~TerrainCreator_Granular() {}

        virtual std::shared_ptr<ChTerrain> GetTerrain() override; 

    protected:
        std::shared_ptr<GranularTerrain> terrain;
};
        
}//end namespace vehicle
}//end namespace chrono

#endif
