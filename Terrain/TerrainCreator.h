#ifndef TERRAIN_CREATOR_H 
#define TERRAIN_CREATOR_H 

#include <memory>

#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono{
namespace vehicle{

class TerrainCreator{

    public:
        TerrainCreator(std::string filename, std::shared_ptr<TrackedVehicle> veh) : file(filename), vehicle(veh) {}

        virtual std::shared_ptr<ChTerrain> GetTerrain() = 0; 

        inline std::string GetTerrainFile() { return file; }

    protected:
        std::shared_ptr<TrackedVehicle> vehicle;

        std::string file;
};
        
}//end namespace vehicle
}//end namespace chrono

#endif
