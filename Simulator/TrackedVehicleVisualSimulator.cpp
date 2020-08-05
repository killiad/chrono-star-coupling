#include "TrackedVehicleVisualSimulator.h"

namespace chrono{
namespace vehicle{

TrackedVehicleVisualSimulator::TrackedVehicleVisualSimulator(std::shared_ptr<TrackedVehicleCreator> userVehicle) : TrackedVehicleSimulator(userVehicle) {}

void TrackedVehicleVisualSimulator::InitializeSimulation(const std::string& driver_file) {

    // Simulation length (Povray only)
    double render_step_size = 1.0 / 50;  // FPS = 50

    if(!filesystem::create_directory("../Outputs")){
        std::cout << "Error creating directory Outputs" << std::endl;
        return;
    }
    if(!filesystem::create_directory("../Outputs/CSV")){
        std::cout << "Error creating directory Outputs/CSV" << std::endl;
        return;
    }

    app = chrono_types::make_shared<ChTrackedVehicleIrrApp>(vehicle.get(), L"Tracked Vehicle Simulation");

    app->SetSkyBox();
    app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app->SetChaseCamera(ChVector<>(0.0, 0.0, 0.0), 6.0, 0.5);

    app->SetTimestep(step_size);
    app->AssetBindAll();
    app->AssetUpdateAll();

    driver = chrono_types::make_shared<ChIrrGuiDriver>(*app);

    // Set the time response for steering and throttle keyboard inputs.
    // NOTE: this is not exact, since we do not render quite at the specified FPS.
    double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver->SetSteeringDelta(render_step_size / steering_time);
    driver->SetThrottleDelta(render_step_size / throttle_time);
    driver->SetBrakingDelta(render_step_size / braking_time);

    // Set file with driver input time series
    driver->SetInputDataFile(vehicle::GetDataFile(driver_file));

    //Custom code. Seting Inpute mode (LOCK, KEYBOARD, DATAFILE, JOYSTICK)
    driver->SetInputMode(driver->InputMode::DATAFILE);
    GetLog() << driver->GetInputModeAsString() << "\n";

    driver->Initialize();

    // Inter-module communication data
    shoe_states_left = BodyStates(vehicle->GetNumTrackShoes(LEFT));
    shoe_states_right = BodyStates(vehicle->GetNumTrackShoes(RIGHT));
    shoe_forces_left = TerrainForces(vehicle->GetNumTrackShoes(LEFT));
    shoe_forces_right = TerrainForces(vehicle->GetNumTrackShoes(RIGHT));

    sim_initialized = true;
}


void TrackedVehicleVisualSimulator::DoStep(const std::vector<Parts>& parts_list) {

    // Output directories (Povray only)
    const std::string csv_dir = "Outputs/CSV";
    char filename[100];

    // Render scene
    app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
    app->DrawAll();
    app->EndScene();

    if(!model_initialized){
        driver = chrono_types::make_shared<ChIrrGuiDriver>(*app);
        double render_step_size = 1.0 / 50;
        double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
        double throttle_time = 1.0;  // time to go from 0 to +1
        double braking_time = 0.3;   // time to go from 0 to +1
        driver->SetSteeringDelta(render_step_size / steering_time);
        driver->SetThrottleDelta(render_step_size / throttle_time);
        driver->SetBrakingDelta(render_step_size / braking_time);
        driver->SetInputDataFile(vehicle::GetDataFile("generic/driver/No_Maneuver.txt"));
        driver->SetInputMode(driver->InputMode::DATAFILE);
        driver->Initialize();

    }

    // Collect output data from modules (for inter-module communication)
    ChDriver::Inputs driver_inputs = driver->GetInputs();
    vehicle->GetTrackShoeStates(LEFT, shoe_states_left);
    vehicle->GetTrackShoeStates(RIGHT, shoe_states_right);

    // Update modules (process inputs from other modules)
    driver->Synchronize(vehicle->GetChTime());
    vehicle->Synchronize(vehicle->GetChTime(), driver_inputs, shoe_forces_left, shoe_forces_right);
    if(terrain_exists){
        terrain->Synchronize(vehicle->GetChTime());
    }
    app->Synchronize("", driver_inputs);

    // Advance simulation for one timestep for all modules
    driver->Advance(step_size);
    vehicle->Advance(step_size);
    if(terrain_exists){
        terrain->Advance(step_size);
    }
    app->Advance(step_size);
    vehicle->GetSystem()->DoStepDynamics(step_size);

    if(makeCSV && model_initialized){
        sprintf(filename, "%s/chrono_to_star_%.3f.csv", csv_dir.c_str(), vehicle->GetChTime());
        std::string fn(filename);
        vehicleCreator->ExportData(parts_list, fn);
    }
    else {
        vehicleCreator->ExportData(parts_list);
    }

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

    // Spin in place for real time to catch up
    realtime_timer.Spin(step_size);
    ++frameCount;
}

void TrackedVehicleVisualSimulator::RunSimulation(const std::string& driver_file, const std::vector<Parts> &vec){

    if(!sim_initialized){
        InitializeSimulation(driver_file);
    }
    if(!model_initialized){
        InitializeModel();
        driver = chrono_types::make_shared<ChIrrGuiDriver>(*app);
        double render_step_size = 1.0 / 50;
        double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
        double throttle_time = 1.0;  // time to go from 0 to +1
        double braking_time = 0.3;   // time to go from 0 to +1
        driver->SetSteeringDelta(render_step_size / steering_time);
        driver->SetThrottleDelta(render_step_size / throttle_time);
        driver->SetBrakingDelta(render_step_size / braking_time);
        driver->SetInputDataFile(vehicle::GetDataFile(driver_file));
        driver->SetInputMode(driver->InputMode::DATAFILE);
        driver->Initialize();
    }

    while (app->GetDevice()->run()) {
        DoStep();
        if(realtime_timer.GetTimeSeconds() >= tend){
            return;
        }
    }
}

void TrackedVehicleVisualSimulator::RunSyncedSimulation(const std::string& driver_file, const std::vector<Parts> &vec,
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
        driver = chrono_types::make_shared<ChIrrGuiDriver>(*app);
        double render_step_size = 1.0 / 50;
        double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
        double throttle_time = 1.0;  // time to go from 0 to +1
        double braking_time = 0.3;   // time to go from 0 to +1
        driver->SetSteeringDelta(render_step_size / steering_time);
        driver->SetThrottleDelta(render_step_size / throttle_time);
        driver->SetBrakingDelta(render_step_size / braking_time);
        driver->SetInputDataFile(vehicle::GetDataFile(driver_file));
        driver->SetInputMode(driver->InputMode::DATAFILE);
        driver->Initialize();
    }
    
    DoStep(vec);
    while(app->GetDevice()->run()){

        if(vehicle->GetChTime() >= tend){
            return;
        }
        if(frameCount % file_ratio == 1 || file_ratio == 1){ 
            sprintf(filename, "../Inputs/star_to_chrono_%.3f.csv", vehicle->GetChTime());
            data_file = filename;
        }
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
            vehicleCreator->AddForce(part, spec_id, force, vehicle->GetChTime());
            vehicleCreator->AddTorque(part, spec_id, moment, vehicle->GetChTime());
            reader.GetLine();
        }
        DoStep(vec);
        reader.Close();
    }
}

} //end namspace vehicle
} //end namespace chrono
