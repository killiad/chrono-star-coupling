#include "TrackedVehicleSimulator.h"
#include "core/ChTypes.h"

namespace chrono{
namespace vehicle{


TrackedVehicleSimulator::TrackedVehicleSimulator(std::shared_ptr<TrackedVehicleCreator> userVehicle) : 
    vehicleCreator(userVehicle), vehicle(userVehicle->GetVehicle()), tend(10.0), step_size(1e-3), 
    makeCSV(false), time_passed(0.0), terrain_exists(false), sim_initialized(false), 
    model_initialized(false), frameCount(0){}

void TrackedVehicleSimulator::SetTerrain(const std::string& filename, Terrain type){

    switch(type){
        
        case Terrain::RIGID:
            terrain = chrono_types::make_shared<RigidTerrain>(vehicle->GetSystem(), vehicle::GetDataFile(filename));
            std::static_pointer_cast<RigidTerrain>(terrain)->Initialize();
            terrain_exists = true;
            break;

        case Terrain::SCM_DEFORMABLE:
            terrain = chrono_types::make_shared<SCMDeformableTerrain>(vehicle->GetSystem());
            std::static_pointer_cast<SCMDeformableTerrain>(terrain)->Initialize(filename);
            terrain_exists = true;
            break;

    }
}

void TrackedVehicleSimulator::SetSimulationLength(double seconds){
    tend = seconds;
}

/*void TrackedVehicleSimulator::SetSaveProperties(int interval, std::string file_prefix){
    save_interval = interval;
    prefix = file_prefix;
    if(!filesystem::create_directory("../Outputs/Saves")){
        std::cout << "Error creating directory Outputs/Saves" << std::endl;
        return;
    }
}*/


void TrackedVehicleSimulator::SetTimeStep(double step) {
	step_size = step;
}

//set true to export the csv files
void TrackedVehicleSimulator::SetCSV(bool export_data) {
	makeCSV = export_data;
}

void TrackedVehicleSimulator::InitializeModel(){
    
    bool fixed = vehicle->GetChassis()->IsFixed();
    bool temp_csv = makeCSV;

    vehicle->GetChassis()->SetFixed(true);
    SetCSV(false);

    if(vehicleCreator->IsParallel()){
        while(time_passed < 0.50){
            std::cout << "INITIALIZING MODEL: NOT ACTUAL SIMULATION" << std::endl;
            DoStep();
        }
    }
    else{
        while(time_passed < 0.02){
            std::cout << "INITIALIZING MODEL: NOT ACTUAL SIMULATION" << std::endl;
            DoStep();
        }
    }
    
    vehicle->GetChassis()->SetFixed(fixed);
    SetCSV(temp_csv);
    vehicle->GetSystem()->SetChTime(0.0);
    time_passed = vehicle->GetChTime();
    frameCount = 0;
    model_initialized = true;
    std::cout << "INITIALIZION COMPLETE" << std::endl;
}

} //end namespace vehicle 
} //end namespace chrono

