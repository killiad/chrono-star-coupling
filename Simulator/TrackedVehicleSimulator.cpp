#include "TrackedVehicleSimulator.h"
#include "core/ChTypes.h"

namespace chrono{
namespace vehicle{


TrackedVehicleSimulator::TrackedVehicleSimulator(std::shared_ptr<TrackedVehicleCreator> userVehicle) : 
    vehicleCreator(userVehicle), vehicle(userVehicle->GetVehicle()), tend(10.0), step_size(1e-3), 
    makeCSV(false), terrain_exists(false), sim_initialized(false), 
    model_initialized(false), info_to_log(true), info_to_terminal(true), frameCount(0){}


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

void TrackedVehicleSimulator::SetLogInfo(bool toTerminal, bool toLog){
    info_to_terminal = toTerminal;
    info_to_log = toLog;
    remove("chrono_log.txt");
}

void TrackedVehicleSimulator::InitializeModel(){
    
    bool fixed = vehicle->GetChassis()->IsFixed();
    bool temp_csv = makeCSV;

    vehicle->GetChassis()->SetFixed(true);
    SetCSV(false);

    if(vehicleCreator->IsParallel()){
        while(vehicle->GetChTime() < 1.0){
            std::cout << "INITIALIZING MODEL: NOT ACTUAL SIMULATION" << std::endl;
            DoStep();
        }
    }
    else{
        while(vehicle->GetChTime() < 0.02){
            std::cout << "INITIALIZING MODEL: NOT ACTUAL SIMULATION" << std::endl;
            DoStep();
        }
    }
    
    vehicle->GetChassis()->SetFixed(fixed);
    SetCSV(temp_csv);
    vehicle->GetSystem()->SetChTime(0.0);
    frameCount = 0;
    model_initialized = true;
    std::cout << "INITIALIZION COMPLETE" << std::endl;
}

} //end namespace vehicle 
} //end namespace chrono

