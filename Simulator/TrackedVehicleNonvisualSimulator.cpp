#include "TrackedVehicleNonvisualSimulator.h"
#include <fstream>

namespace chrono{
namespace vehicle{

TrackedVehicleNonVisualSimulator::TrackedVehicleNonVisualSimulator(std::shared_ptr<TrackedVehicleCreator> userVehicle) : TrackedVehicleSimulator(userVehicle) {}

void TrackedVehicleNonVisualSimulator::InitializeSimulation(const std::string& driver_file) {

    driver = chrono_types::make_shared<ChDataDriver>(*vehicle, vehicle::GetDataFile(driver_file));
    driver->Initialize();

    // Inter-module communication data
    shoe_states_left = BodyStates(vehicle->GetNumTrackShoes(LEFT));
    shoe_states_right = BodyStates(vehicle->GetNumTrackShoes(RIGHT));
    shoe_forces_left = TerrainForces(vehicle->GetNumTrackShoes(LEFT));
    shoe_forces_right = TerrainForces(vehicle->GetNumTrackShoes(RIGHT));

    if(!filesystem::create_directory("../Outputs")){   
        std::cout << "Error creating directory Outputs" << std::endl;
        return;
    }
    if(!filesystem::create_directory("../Outputs/CSV")){
        std::cout << "Error creating directory Outputs/CSV" << std::endl;
        return;
    }
    if(!filesystem::create_directory("../Inputs")){   
        std::cout << "Error creating directory Inputs" << std::endl;
        return;
    }

    sim_initialized = true;
}

void TrackedVehicleNonVisualSimulator::DoStep(const std::vector<Parts> &parts_list) {

    char filename[100];
    std::string csv_dir("../Outputs/CSV");

    // Collect output data from modules (for inter-module communication)
    if(!model_initialized){
        driver = chrono_types::make_shared<ChDataDriver>(*vehicle, vehicle::GetDataFile("generic/driver/No_Maneuver.txt"));
        driver->Initialize();
    }
    ChDriver::Inputs driver_inputs = driver->GetInputs();
    vehicle->GetTrackShoeStates(LEFT, shoe_states_left);
    vehicle->GetTrackShoeStates(RIGHT, shoe_states_right);

    // Update modules (process inputs from other modules)
    driver->Synchronize(vehicle->GetChTime());
    vehicle->Synchronize(vehicle->GetChTime(), driver_inputs, shoe_forces_left, shoe_forces_right);
    if(terrain_exists){
        terrain->Synchronize(vehicle->GetChTime());
    }
   
    // Advance simulation for one timestep for all modules
    driver->Advance(step_size);
    vehicle->Advance(step_size);
    if(terrain_exists){
        terrain->Advance(step_size);
    }
    //do I only want this if it is in parallel
    vehicle->GetSystem()->DoStepDynamics(step_size);
 
    // Output data for STAR-CCM+
    if(makeCSV && model_initialized){
        sprintf(filename, "%s/chrono_to_star_%.3f.csv", csv_dir.c_str(), vehicle->GetChTime());
        std::string fn(filename);
        vehicleCreator->ExportData(parts_list, fn);
    }
    else{
        vehicleCreator->ExportData(parts_list);
    }

    //send to a log file (optional)
    if(info_to_terminal){
        std::cout << "Sim frame:       " << frameCount << std::endl;
        std::cout << "Time after step: " << vehicle->GetChTime() << std::endl;
        std::cout << "   Throttle: " << driver->GetThrottle() << "   steering: " << driver->GetSteering()
                  << "   braking:  " << driver->GetBraking() << std::endl;
        std::cout << "Vehicle position: " << vehicle->GetVehiclePos() << std::endl;
        std::cout << "Vehicle rotation: " << vehicle->GetVehicleRot() << std::endl;
        std::cout << std::endl;
    }
    if(info_to_log && model_initialized){
        std::ofstream log;
        log.open("chrono_log.txt");
        log << "Sim frame:       " << frameCount << "\n";
        log << "Time after step: " << vehicle->GetChTime() << "\n";
        log << "   Throttle: " << driver->GetThrottle() << "   steering: " << driver->GetSteering()
                  << "   braking:  " << driver->GetBraking() << "\n";
        log << "Vehicle position: " << vehicle->GetVehiclePos() << "\n";
        log << "Vehicle rotation: " << vehicle->GetVehicleRot() << "\n";
        log << "\n";
        log.close();
    }

    // Increment frame number
    frameCount++;
    
}

void TrackedVehicleNonVisualSimulator::RunSimulation(const std::string& driver_file, const std::vector<Parts> &vec) {
    
    if(!sim_initialized){
        InitializeSimulation(driver_file);
    }
    if(!model_initialized){
        InitializeModel();
        driver = chrono_types::make_shared<ChDataDriver>(*vehicle, vehicle::GetDataFile(driver_file));
        driver->Initialize();
    }

    while (vehicle->GetChTime() < tend) {
        DoStep(vec);
    }
}

void TrackedVehicleNonVisualSimulator::RunSyncedSimulation(const std::string& driver_file, const std::vector<Parts> &vec,
        const int file_ratio) {
   
    char filename[100];
    int spec_id;
    ChVector<> force;
    ChVector<> moment;
    Parts part;
    std::string data_file;


    if(!sim_initialized){
        InitializeSimulation(driver_file);
    }
    if(!model_initialized){
        InitializeModel();
        driver = chrono_types::make_shared<ChDataDriver>(*vehicle, vehicle::GetDataFile(driver_file));
        driver->Initialize();
    }
    
    DoStep(vec);
    while(vehicle->GetChTime() < tend){
        
        if(frameCount % file_ratio == 1 || file_ratio == 1){
            sprintf(filename, "../Inputs/star_to_chrono_%.3f.csv", vehicle->GetChTime());
            data_file = filename;
        }
        CSVReader reader(data_file);
        
        std::cout << "Searching for file: " << data_file << std::endl;
        while(!reader.Open(data_file)){
            std::cout << "Waiting for file: " << data_file << std::endl;
            sleep(1);
        }

        for(auto part_body : vec){
            vehicleCreator->ClearAddedForces(part_body);
        }

        reader.GetLine(); 
        while(reader.IsValidRow()){
            part = vehicleCreator->ID_To_Part(reader.GetNumber());
            spec_id = reader.GetNumber();
            force = reader.GetVector();
            moment = reader.GetVector();
            vehicleCreator->AddForce(part, spec_id, force, vehicle->GetChTime());
            vehicleCreator->AddTorque(part, spec_id, moment, vehicle->GetChTime());
            reader.GetLine();
        }
        DoStep(vec);
        reader.Close();
    }
}


}
}
