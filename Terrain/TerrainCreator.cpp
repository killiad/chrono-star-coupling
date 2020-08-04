#include "TerrainCreator.h"
#include "../CSV/CSVReader.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include <memory>

namespace chrono{
namespace vehicle{

    // add vehicle creator
TerrainCreator::TerrainCreator(std::shared_ptr<ChTrackedVehicle> veh, TerrainModel type, std::string filename) :
    file(filename),terrain_model(type), vehicle(veh){
    
    switch(type){
        
        case TerrainModel::RIGID:
            InitializeRigid();
            break;

        case TerrainModel::SCM_DEFORMABLE:
            InitializeSCMDeformable();
            break;

        case TerrainModel::FLAT:
            InitializeFlat();
            break;
    }           
}

void TerrainCreator::InitializeRigid(){
    terrain = chrono_types::make_shared<RigidTerrain>(vehicle->GetSystem(), GetDataFile(file));
    std::dynamic_pointer_cast<RigidTerrain>(terrain)->Initialize();
}

void TerrainCreator::InitializeSCMDeformable(){
    CSVReader csv(file);
    std::cout << "Line: " << csv.GetRow() << std::endl;
    csv.GetLine();

    std::cout << "1" << std::endl;
    //Get Data
    std::cout << "Line: " << csv.GetRow() << std::endl;
    std::string mesh_file = csv.GetString();
    std::cout << "2" << std::endl;
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

void TerrainCreator::InitializeFlat(){
    CSVReader csv(file);
    csv.GetLine();

    double height = csv.GetNumber();
    float friction = static_cast<float>(csv.GetNumber());

    terrain = chrono_types::make_shared<FlatTerrain>(height, friction);
}

}//end namespace vehicle
}//end namespace chrono
