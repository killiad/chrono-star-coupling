#include "TerrainCreator_SCMDeformable.h"

namespace chrono{
namespace vehicle{
    
TerrainCreator_SCMDeformable::TerrainCreator_SCMDeformable(std::string filename, std::shared_ptr<TrackedVehicle> veh) : TerrainCreator(filename, veh),
    terrain(chrono_types::make_shared<SCMDeformableTerrain>(vehicle->GetSystem())) { 
    
    CSVReader csv(GetDataFile(file));
    csv.GetLine();

    //Get Data
    std::string mesh_file = csv.GetString();
    double bekker_kphi = csv.GetNumber();
    double bekker_kc = csv.GetNumber();
    double bekker_n = csv.GetNumber();
    double mohr_cohesion = csv.GetNumber();
    double mohr_friction = csv.GetNumber();
    double janosi_shear = csv.GetNumber();
    double elastic_k = csv.GetNumber();
    double dampening_r = csv.GetNumber();


    terrain->SetSoilParameters(bekker_kphi, bekker_kc, bekker_n, mohr_cohesion,
            mohr_friction, janosi_shear, elastic_k, dampening_r);
    terrain->Initialize(GetDataFile(mesh_file));
}

std::shared_ptr<ChTerrain> TerrainCreator_SCMDeformable::GetTerrain() { return terrain; }

}
}
