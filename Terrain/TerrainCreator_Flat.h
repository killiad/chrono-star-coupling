#ifndef TERRAIN_CREATOR_FLAT_H_
#define TERRAIN_CREATOR_FLAT_H

#include <memory>
#include "TerrainCreator.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "../CSV/CSVReader.h"

namespace chrono{
namespace vehicle{

class TerrainCreator_Flat : TerrainCreator {

    public:
        TerrainCreator_Flat(std::string filename, std::shared_ptr<TrackedVehicle> veh); 

        virtual ~TerrainCreator_Flat() {}

        virtual std::shared_ptr<ChTerrain> GetTerrain() override; 

    protected:
        std::shared_ptr<FlatTerrain> terrain;
};
        
}//end namespace vehicle
}//end namespace chrono

#endif
