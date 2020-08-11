#include "TerrainCreator_FEADeformable.h"

namespace chrono{
namespace vehicle{
    
TerrainCreator_FEADeformable::TerrainCreator_FEADeformable(std::string filename, std::shared_ptr<TrackedVehicle> veh) : TerrainCreator(filename, veh),
    terrain(chrono_types::make_shared<FEADeformableTerrain>(vehicle->GetSystem())) { 
    
    CSVReader csv(GetDataFile(file));
    csv.GetLine();

    //Get Data
    double rho = csv.GetNumber();
    double emod = csv.GetNumber();
    double nu = csv.GetNumber();
    double yield_stess = csv.GetNumber();
    double hardening_slope = csv.GetNumber();
    double friction_angle = csv.GetNumber();
    double dilatancy_angle = csv.GetNumber();
    auto start_point = csv.GetVector();
    auto terrain_dimension = csv.GetVector();
    auto terrain_discretization = csv.GetVector();

    terrain->SetSoilParametersFEA(rho, emod, nu, yield_stess, hardening_slope, friction_angle, dilatancy_angle);
    terrain->Initialize(start_point, terrain_dimension, terrain_discretization);
}

std::shared_ptr<ChTerrain> TerrainCreator_FEADeformable::GetTerrain() { return terrain; }

}
}
