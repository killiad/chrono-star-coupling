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
    if(!filesystem::create_directory("../Outputs/POVRAY")){
        std::cout << "Error creating directory Outputs/POVRAY" << std::endl;
        return;
    }

    app = chrono_types::make_shared<ChTrackedVehicleIrrApp>(vehicle.get(), L"Tracked Vehicle Simulation");

    app->SetSkyBox();
    app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    //app->SetChaseCamera(ChVector<>(0.0, 0.0, 0.0), 6.0, 0.5);

    app->SetTimestep(step_size);
    auto camera = chrono_types::make_shared<ChCamera>();
    camera->SetPosition(ChVector<>(-3,0,3));
    camera->SetAimPoint(ChVector<>(0,0,0));
    vehicle->GetChassisBody()->AddAsset(camera);

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
    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    shoe_states_left = BodyStates(vehicle->GetNumTrackShoes(LEFT));
    shoe_states_right = BodyStates(vehicle->GetNumTrackShoes(RIGHT));
    shoe_forces_left = TerrainForces(vehicle->GetNumTrackShoes(LEFT));
    shoe_forces_right = TerrainForces(vehicle->GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    //Set up post processing
    if(run_postprocesser){
        InitializePostProcess();
    }

    sim_initialized = true;
}


void TrackedVehicleVisualSimulator::DoStep(const std::vector<Parts>& parts_list) {

    // Output directories (Povray only)
    const std::string csv_dir = "Outputs/CSV";
    const std::string pov_dir = "Outputs/POVRAY";
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
    time_passed = vehicle->GetSystem()->GetChTime();
    driver->Synchronize(time_passed);
    vehicle->Synchronize(time_passed, driver_inputs, shoe_forces_left, shoe_forces_right);
    if(terrain_exists){
        terrain->Synchronize(time_passed);
    }
    app->Synchronize("", driver_inputs);

    // Advance simulation for one timestep for all modules
    driver->Advance(step_size);
    vehicle->Advance(step_size);
    if(terrain_exists){
        terrain->Advance(step_size);
    }
    app->Advance(step_size);

    if(makeCSV && model_initialized){
        sprintf(filename, "%s/chrono_to_star_%.3f.csv", csv_dir.c_str(), time_passed);
        std::string fn(filename);
        vehicleCreator->ExportData(parts_list, fn);
    }
    else {
        vehicleCreator->ExportData(parts_list);
    }

    if(run_postprocesser && model_initialized){
        std::cout << "Exporting POVRAY Data" <<std::endl;
        //pov_exporter.ExportData();
        sprintf(filename, "%s/data_%.3f.dat", pov_dir.c_str(), time_passed);
        std::string fn(filename);
        utils::WriteShapesPovray(vehicle->GetSystem(), fn);
    }

    if(frameCount % save_interval == 0 && model_initialized){
        SaveData();
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
        if(realtime_timer.GetTimeSeconds() > tend){
            return;
        }
    }
}

void TrackedVehicleVisualSimulator::RunSyncedSimulation(const std::string& driver_file, const std::vector<Parts> &vec) {
   
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
    }
    
    DoStep(vec);
    while(app->GetDevice()->run()){

        if(time_passed >= tend){
            return;
        }
        
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

} //end namspace vehicle
} //end namespace chrono
