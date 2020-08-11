#include "TerrainCreator_Granular.h"
#include "physics/ChMaterialSurface.h"

namespace chrono{
namespace vehicle{
    
TerrainCreator_Granular::TerrainCreator_Granular(std::string filename, std::shared_ptr<TrackedVehicle> veh) : TerrainCreator(filename, veh),
    terrain(chrono_types::make_shared<GranularTerrain>(vehicle->GetSystem())) {
        
    CSVReader csv(GetDataFile(file));
    csv.GetLine();
    std::string method = csv.GetString();
    double s_friction = csv.GetNumber();
    double k_friction = csv.GetNumber();
    double r_friction = csv.GetNumber();
    double spin_friction = csv.GetNumber();
    double restitution = csv.GetNumber();
    double envelope = csv.GetNumber();
    double min_particles = csv.GetNumber();
    auto center = csv.GetVector();
    double length = csv.GetNumber();
    double width = csv.GetNumber();
    double layers = csv.GetNumber();
    double radius = csv.GetNumber();
    double density = csv.GetNumber();
    auto init_velocity = csv.GetVector();

    //setting terrain surface material
    std::shared_ptr<ChMaterialSurface> surface;
    if(method == "NSC"){
        surface = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    }
    else{
        surface = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    }
    surface->SetSfriction(s_friction);
    surface->SetKfriction(k_friction);
    surface->SetRollingFriction(r_friction);
    surface->SetSpinningFriction(spin_friction);
    surface->SetRestitution(restitution);
    terrain->SetContactMaterial(surface);

    terrain->SetCollisionEnvelope(envelope);
    terrain->SetMinNumParticles(min_particles);
    terrain->Initialize(center, length, width, layers, radius, density, init_velocity);
}

std::shared_ptr<ChTerrain> TerrainCreator_Granular::GetTerrain() { return terrain; }

}
}
