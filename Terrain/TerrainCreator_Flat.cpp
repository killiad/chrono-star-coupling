#include "TerrainCreator_Flat.h"

namespace chrono{
namespace vehicle{
    
TerrainCreator_Flat::TerrainCreator_Flat(std::string filename, std::shared_ptr<TrackedVehicle> veh) : TerrainCreator(filename, veh),
    terrain(chrono_types::make_shared<FlatTerrain>(0,1)){
        
    CSVReader csv(GetDataFile(file));
    csv.GetLine();

    double height = csv.GetNumber();
    float friction = static_cast<float>(csv.GetNumber());

    terrain = chrono_types::make_shared<FlatTerrain>(height, friction);
}

std::shared_ptr<ChTerrain> TerrainCreator_Flat::GetTerrain() { return terrain; }

}
}
