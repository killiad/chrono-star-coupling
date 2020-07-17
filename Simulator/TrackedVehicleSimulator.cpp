#include "TrackedVehicleSimulator.h"

namespace chrono{
namespace vehicle{


TrackedVehicleSimulator::TrackedVehicleSimulator(std::shared_ptr<TrackedVehicleCreator> userVehicle) : 
    vehicleCreator(userVehicle), vehicle(userVehicle->GetVehicle()), tend(10.0), 
    step_size(4e-3), makeCSV(false), run_postprocesser(false), pov_exporter(vehicle->GetSystem()), 
    time_passed(0.0), terrain_exists(false), renderCount(1), sim_initialized(false), model_initialized(false),
    save_interval(-1), prefix(""){}

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

void TrackedVehicleSimulator::SetSaveProperties(int interval, std::string file_prefix){
    save_interval = interval;
    prefix = file_prefix;
    if(!filesystem::create_directory("../Outputs/Saves")){
        std::cout << "Error creating directory Outputs/Saves" << std::endl;
        return;
    }
}

void TrackedVehicleSimulator::SetSolver() {

	solver = chrono_types::make_shared<ChSolverPSOR>();
    solver->SetMaxIterations(50);;
    solver->SetOmega(0.8);
    solver->SetSharpnessLambda(1.0);
    vehicle->GetSystem()->SetSolver(solver);

    vehicle->GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
    vehicle->GetSystem()->SetMinBounceSpeed(2.0);
}

void TrackedVehicleSimulator::SetTimeStep(double step) {
	step_size = step;
}

//set true to export the csv files
void TrackedVehicleSimulator::SetCSV(bool export_data) {
	makeCSV = export_data;
}

//Set post processing
void TrackedVehicleSimulator::SetPostProcess(bool process) {
	run_postprocesser = process;
}

void TrackedVehicleSimulator::InitializePostProcess() {

	pov_exporter = postprocess::ChPovRay(vehicle->GetSystem());

	// Create (if needed) the output directory
    const std::string dir = "../Outputs/POVRAY";
    if (!filesystem::create_directory(filesystem::path(dir))) {
        std::cout << "Error creating directory " << dir << std::endl;
        return;
    }

    // Sets some file names for in-out processes.
    pov_exporter.SetTemplateFile(GetChronoDataFile("_template_POV.pov"));
    pov_exporter.SetOutputScriptFile(dir + "/rendering_frames.pov");
    pov_exporter.SetOutputDataFilebase("my_state");
    pov_exporter.SetPictureFilebase("picture");

    // Even better: save the .dat files and the .bmp files in two subdirectories,
    // to avoid cluttering the current directory.
    const std::string out_dir = dir + "/output";
    const std::string anim_dir = dir + "/anim";
    filesystem::create_directory(filesystem::path(out_dir));
    filesystem::create_directory(filesystem::path(anim_dir));

    pov_exporter.SetOutputDataFilebase(out_dir + "/my_state");
    pov_exporter.SetPictureFilebase(anim_dir + "/picture");

    // --Optional: modify default light
    pov_exporter.SetLight(ChVector<>(-3, 4, 2), ChColor(0.15f, 0.15f, 0.12f), false);

    // --Optional: add further POV commands, for example in this case:
    //     create an area light for soft shadows
    //     create a Grid object; Grid() parameters: step, linewidth, linecolor, planecolor
    //   Remember to use \ at the end of each line for a multiple-line string.
    pov_exporter.SetCustomPOVcommandsScript(
        " \
    	light_source {   \
    	<2, 10, -3>  \
    	color rgb<1.2,1.2,1.2> \
        area_light <4, 0, 0>, <0, 0, 4>, 8, 8 \
        adaptive 1 \
        jitter\
        } \
        object{ Grid(1,0.02, rgb<0.7,0.8,0.8>, rgbt<1,1,1,1>) rotate <0, 0, 90>  } \
    ");

    // --Optional: attach additional custom POV commands to some of the rigid bodies,
    //   using the ChPovRayAssetCustom asset. This asset for example projects a
    //   checkered texture to the floor. This POV specific asset won't be rendered
    //   by Irrlicht or other interfaces.
    auto mPOVcustom = chrono_types::make_shared<postprocess::ChPovRayAssetCustom>();
    mPOVcustom->SetCommands((char*)"pigment { checker rgb<0.9,0.9,0.9>, rgb<0.75,0.8,0.8> }");

    // IMPORTANT! Tell to the POVray exporter that
    // he must take care of converting the shapes of
    // all items!

    pov_exporter.AddAll();
    pov_exporter.ExportScript();
}

void TrackedVehicleSimulator::InitializeModel(){
    
    bool fixed = vehicle->GetChassis()->IsFixed();
    bool temp_csv = makeCSV;
    bool temp_process = run_postprocesser;

    vehicle->GetChassis()->SetFixed(true);
    SetCSV(false);
    SetPostProcess(false);

    while(time_passed < 0.02){
        std::cout << "INITIALIZING MODEL: NOT ACTUAL SIMULATION" << std::endl;
        DoStep();
    }
    
    vehicle->GetChassis()->SetFixed(fixed);
    SetCSV(temp_csv);
    SetPostProcess(temp_process);
    vehicle->GetSystem()->SetChTime(0.0);
    time_passed = vehicle->GetChTime();
    frameCount = 0;
    model_initialized = true;
    std::cout << "INITIALIZION COMPLETE" << std::endl;
}


void TrackedVehicleSimulator::SaveData() const {

    char filename [100];
    
    sprintf(filename, "../Outputs/Saves/%s_%.3f.csv", prefix.c_str(), time_passed);
    std::string fn(filename);
    bool *dof = vehicleCreator->GetDOF();
    auto body = chrono_types::make_shared<ChBody>();
    CSVMaker csv(fn);
    csv.Clear();

    //First row contains file name, powertrain file, constact method, time, degrees of freedom, vehicle position, 
    //vehicle orientation, and vehicle speed
    csv.Add(vehicleCreator->GetMasterFile());
    csv.AddComma();
    csv.Add(vehicleCreator->GetPowertrainFile());
    csv.AddComma();
    if(vehicle->GetSystem()->GetContactMethod() == ChContactMethod::NSC){
        csv.Add("NSC");
    }
    else{
        csv.Add("SMC");
    }
    csv.AddComma();
    csv.Add(time_passed);
    csv.AddComma();
    for(int degree = 0; degree < 6; ++degree){
        csv.Add(dof[degree]);
        csv.AddComma();
    }
    csv.AddVector(vehicle->GetVehiclePos());
    csv.AddComma();
    csv.AddQuaternion(vehicle->GetVehicleRot());
    csv.AddComma();
    csv.Add(vehicle->GetChassisBody()->GetPos_dt().x());
    csv.NewLine();

    //All other rows contain general ID, specific ID, part: position, orientaion, velocity, angular velocity,
    //acceleration, angular acceleration, force, and torque (gyro torqye?)

    //Chassis
    body = vehicle->GetChassisBody();
    csv.SaveBodyData(body, vehicleCreator->Part_to_ID(Parts::CHASSIS), 0);
    csv.NewLine();
    
    //Track Shoes
    for(int spec_id = 0; spec_id < vehicleCreator->GetVehicleInfo().Left_TrackShoeNum; ++spec_id){
        body = vehicle->GetTrackShoe(LEFT, spec_id)->GetShoeBody();
        csv.SaveBodyData(body, vehicleCreator->Part_to_ID(Parts::TRACKSHOE_LEFT), spec_id);
        csv.NewLine();
    }
    
    for(int spec_id = 0; spec_id < vehicleCreator->GetVehicleInfo().Right_TrackShoeNum; ++spec_id){
        body = vehicle->GetTrackShoe(RIGHT, spec_id)->GetShoeBody();
        csv.SaveBodyData(body, vehicleCreator->Part_to_ID(Parts::TRACKSHOE_RIGHT), spec_id);
        csv.NewLine();
    }

    //Sprockets
    body = vehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody();
    csv.SaveBodyData(body, vehicleCreator->Part_to_ID(Parts::SPROCKET_LEFT), 0);
    csv.NewLine();

    body = vehicle->GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody();
    csv.SaveBodyData(body, vehicleCreator->Part_to_ID(Parts::SPROCKET_RIGHT), 0);
    csv.NewLine();
   
    //Idlers
    body = vehicle->GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody();
    csv.SaveBodyData(body, vehicleCreator->Part_to_ID(Parts::IDLER_LEFT), 0);
    csv.NewLine();

    body = vehicle->GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody();
    csv.SaveBodyData(body, vehicleCreator->Part_to_ID(Parts::IDLER_RIGHT), 0);
    csv.NewLine();

    //Rollers
    for(int spec_id = 0; spec_id < vehicleCreator->GetVehicleInfo().Left_RollerNum; ++spec_id){
        body = vehicle->GetTrackAssembly(LEFT)->GetRoller(spec_id)->GetBody();
        csv.SaveBodyData(body, vehicleCreator->Part_to_ID(Parts::ROLLER_LEFT), spec_id);
        csv.NewLine();
    }
    
    for(int spec_id = 0; spec_id < vehicleCreator->GetVehicleInfo().Right_RollerNum; ++spec_id){
        body = vehicle->GetTrackAssembly(RIGHT)->GetRoller(spec_id)->GetBody();
        csv.SaveBodyData(body, vehicleCreator->Part_to_ID(Parts::ROLLER_RIGHT), spec_id);
        csv.NewLine();
    }

    //Road Wheels
    for(int spec_id = 0; spec_id < vehicleCreator->GetVehicleInfo().Left_RoadWheelNum; ++spec_id){
        body = vehicle->GetTrackAssembly(LEFT)->GetRoadWheel(spec_id)->GetWheelBody();
        csv.SaveBodyData(body, vehicleCreator->Part_to_ID(Parts::ROADWHEEL_LEFT), spec_id);
        csv.NewLine();
    }
    
    for(int spec_id = 0; spec_id < vehicleCreator->GetVehicleInfo().Right_RoadWheelNum; ++spec_id){
        body = vehicle->GetTrackAssembly(RIGHT)->GetRoadWheel(spec_id)->GetWheelBody();
        csv.SaveBodyData(body, vehicleCreator->Part_to_ID(Parts::ROADWHEEL_RIGHT), spec_id);
        csv.NewLine();
    }

    std::cout << "Saved data to " << fn << std::endl; 
}

} //end namespace vehicle 
} //end namespace chrono

