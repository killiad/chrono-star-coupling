#include "TrackedVehicleNonvisualSimulator.h"

namespace chrono{
namespace vehicle{

TrackedVehicleNonVisualSimulator::TrackedVehicleNonVisualSimulator(std::shared_ptr<TrackedVehicleCreator> userVehicle) : TrackedVehicleSimulator(userVehicle) {}

void TrackedVehicleNonVisualSimulator::InitializeSimulation(const std::string& driver_file) {

    driver = chrono_types::make_shared<ChDataDriver>(*vehicle, vehicle::GetDataFile(driver_file));
    driver->Initialize();

    auto camera = chrono_types::make_shared<ChCamera>();
    camera->SetPosition(ChVector<>(0,-6,0));
    camera->SetAimPoint(ChVector<>(0,0,0));
    vehicle->GetChassisBody()->AddAsset(camera);

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
        std::cout << "Error creating directory " << "Outputs/CSV" << std::endl;
        return;
    }
    if(!filesystem::create_directory("../Outputs/POVRAY")){
        std::cout << "Error creating directory " << "Outputs/POVRAY" << std::endl;
        return;
    }
    if(!filesystem::create_directory("../Inputs")){   
        std::cout << "Error creating directory Inputs" << std::endl;
        return;
    }

    //Set up post processing
    if(run_postprocesser){
        InitializePostProcess();
    }
    sim_initialized = true;
}

void TrackedVehicleNonVisualSimulator::DoStep(const std::vector<Parts> &parts_list) {

    char filename[100];
    std::string pov_dir("../Outputs/POVRAY");
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
    time_passed = vehicle->GetSystem()->GetChTime();
    driver->Synchronize(time_passed);
    vehicle->Synchronize(time_passed, driver_inputs, shoe_forces_left, shoe_forces_right);
    if(terrain_exists){
        terrain->Synchronize(time_passed);
    }
    
    // Advance simulation for one timestep for all modules
    driver->Advance(step_size);
    vehicle->Advance(step_size);
    if(terrain_exists){
        terrain->Advance(step_size);
    }

    //outpit render data
    if(run_postprocesser && model_initialized){
        if(frameCount % 33 == 0){
            std::cout << "Exporting POVRAY Data" <<std::endl;
            pov_exporter.ExportData();
            sprintf(filename, "%s/data_%.3d.dat", pov_dir.c_str(), renderCount);
            std::string fn(filename);
            utils::WriteShapesPovray(vehicle->GetSystem(), fn);
            ++renderCount;
        }
    }

    // Output data for STAR-CCM+
    if(makeCSV && model_initialized){
        sprintf(filename, "%s/chrono_to_star_%.3f.csv", csv_dir.c_str(), time_passed);
        std::string fn(filename);
        vehicleCreator->ExportData(parts_list, fn);
    }
    else{
        vehicleCreator->ExportData(parts_list);
    }

    //Output save data
    if(frameCount % save_interval == 0 && model_initialized){
        SaveData();
    }

    std::cout << "Sim frame:      " << frameCount << std::endl;
    std::cout << "Time:           " << time_passed << std::endl;
    std::cout << "   throttle: " << driver->GetThrottle() << "   steering: " << driver->GetSteering()
              << "   braking:  " << driver->GetBraking() << std::endl;
    std::cout << "Vehicle position: " << vehicle->GetVehiclePos() << std::endl;
    std::cout << "Vehicle rotation: " << vehicle->GetVehicleRot() << std::endl;
    std::cout << std::endl;

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

    while (time_passed < tend) {
        DoStep(vec);
    }
}

void TrackedVehicleNonVisualSimulator::RunSyncedSimulation(const std::string& driver_file, const std::vector<Parts> &vec) {
   
    char filename[100];
    int spec_id;
    ChVector<> force;
    ChVector<> moment;
    Parts part;


    if(!sim_initialized){
        InitializeSimulation(driver_file);
    }
    if(!model_initialized){
        InitializeModel();
        driver = chrono_types::make_shared<ChDataDriver>(*vehicle, vehicle::GetDataFile(driver_file));
        driver->Initialize();
    }
    
    DoStep(vec);
    while(time_passed < tend){
        
        sprintf(filename, "../Inputs/star_to_chrono_%.3f.csv", time_passed);
        std::string data_file(filename);
        CSVReader reader(data_file);
        
        while(!reader.Open(filename)){
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
            vehicleCreator->AddForce(part, spec_id, force, time_passed);
            vehicleCreator->AddTorque(part, spec_id, moment, time_passed);
        }
        DoStep(vec);
    }
}


}
}
