#include "TerrainCreator.h"
#include "../CSV/CSVReader.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include <memory>

namespace chrono{
namespace vehicle{

    // add vehicle creator
TerrainCreator::TerrainCreator(std::shared_ptr<ChTrackedVehicle> veh, TerrainType type, std::string filename) :
    file(filename),terrain_type(type), vehicle(veh){
    
    switch(type){
        
        case TerrainType::RIGID:
            InitializeRigid();
            break;

        case TerrainType::SCM_DEFORMABLE:
            InitializeSCMDeformable();
            break;
    }           
}

void TerrainCreator::InitializeRigid(){
    terrain = chrono_types::make_shared<RigidTerrain>(vehicle->GetSystem(), GetDataFile(file));
    std::dynamic_pointer_cast<RigidTerrain>(terrain)->Initialize();
}

void TerrainCreator::InitializeSCMDeformable(){
    CSVReader csv(file);
    //Skip first line
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

    terrain = chrono_types::make_shared<SCMDeformableTerrain>(vehicle->GetSystem());
    std::dynamic_pointer_cast<SCMDeformableTerrain>(terrain)->Initialize(GetDataFile(mesh_file));
    std::dynamic_pointer_cast<SCMDeformableTerrain>(terrain)->SetSoilParameters(bekker_kphi, bekker_kc, bekker_n, mohr_cohesion,
            mohr_friction, janosi_shear, elastic_k, dampening_r);
}

}//end namespace vehicle
}//end namespace chrono
