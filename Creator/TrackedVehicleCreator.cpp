#include "TrackedVehicleCreator.h"
#include "chrono_parallel/physics/Ch3DOFContainer.h"
#include "core/ChTypes.h"
#include "physics/ChMaterialSurface.h"

namespace chrono{
namespace vehicle{

TrackedVehicleCreator::TrackedVehicleCreator(const std::string& filename, ChContactMethod method, bool parallel) : master_file(filename),
    powertrain_file(""), powertrain(false), restricted(false), is_parallel(parallel){

    //// NOTE
    //// When using SMC, a double-pin shoe type requires MKL or MUMPS.  
    //// However, there appear to still be redundant constraints in the double-pin assembly
    //// resulting in solver failures with MKL and MUMPS (rank-deficient matrix).
    ////
    //// For now, use ChContactMethod::NSC for a double-pin track model

    // Create the vehicle system
    ChSystem* system;
    switch(method){
        
        case ChContactMethod::NSC:
            if(parallel){
                system = new ChSystemParallelNSC();   
            }
            else{
                system = new ChSystemNSC();
            }
            break;
        
        case ChContactMethod::SMC:
            if(parallel){
                system = new ChSystemParallelSMC();   
            }
            else{
                system = new ChSystemSMC();
            }
            break;
    }
    vehicle = chrono_types::make_shared<TrackedVehicle>(system, vehicle::GetDataFile(filename));
    for(int i = 0; i < 6; ++i){
        DOF[i] = false;
    }

    // Control steering type (enable crossdrive capability).
    ////vehicle.GetDriveline()->SetGyrationMode(true);
    
    //populating VehicleInfo struct
    auto left_assembly = vehicle->GetTrackAssembly(LEFT);
    auto right_assembly = vehicle->GetTrackAssembly(RIGHT);
    info.Left_TrackShoeNum = left_assembly->GetNumTrackShoes();
    info.Right_TrackShoeNum = right_assembly->GetNumTrackShoes();
    info.Left_RollerNum = left_assembly->GetNumRollers();
    info.Right_RollerNum = right_assembly->GetNumRollers();
    info.Left_RoadWheelNum = left_assembly->GetNumRoadWheelAssemblies();
    info.Right_RoadWheelNum = right_assembly->GetNumRoadWheelAssemblies();
    info.Mass = vehicle->GetVehicleMass();
}

/*TrackedVehicleCreator::TrackedVehicleCreator(std::string save_file){
    CSVReader csv(save_file);
    saveFile = save_file;
        
    //First row data
    master_file = csv.GetString();
    powertrain_file = csv.GetString();
    ChContactMethod method;
    if(csv.GetString() == "NSC"){
       method = ChContactMethod::NSC;
    }
    else{
        method = ChContactMethod::SMC;
    }
    double sim_time = csv.GetNumber();
    DOF[0] = csv.GetNumber();
    DOF[1] = csv.GetNumber();
    DOF[2] = csv.GetNumber();
    DOF[3] = csv.GetNumber();
    DOF[4] = csv.GetNumber();
    DOF[5] = csv.GetNumber();
    init_pos = ChVector<>(csv.GetNumber(), csv.GetNumber(), csv.GetNumber());
    init_rot = ChQuaternion<> (csv.GetNumber(), csv.GetNumber(), csv.GetNumber(), csv.GetNumber());
    init_Chassis_FwdVel = csv.GetNumber();

    vehicle = chrono_types::make_shared<TrackedVehicle>(vehicle::GetDataFile(master_file), method);
    vehicle->GetSystem()->SetChTime(sim_time);

    auto left_assembly = vehicle->GetTrackAssembly(LEFT);
    auto right_assembly = vehicle->GetTrackAssembly(RIGHT);
    info.Left_TrackShoeNum = left_assembly->GetNumTrackShoes();
    info.Right_TrackShoeNum = right_assembly->GetNumTrackShoes();
    info.Left_RollerNum = left_assembly->GetNumRollers();
    info.Right_RollerNum = right_assembly->GetNumRollers();
    info.Left_RoadWheelNum = left_assembly->GetNumRoadWheelAssemblies();
    info.Right_RoadWheelNum = right_assembly->GetNumRoadWheelAssemblies();
    info.Mass = vehicle->GetVehicleMass();
}*/

void TrackedVehicleCreator::SetSolver(int threads) {

    auto method = vehicle->GetSystem()->GetContactMethod();
    int max_iteration_bilateral = 1000;  // 1000;
    int max_iteration_normal = 0;
    int max_iteration_sliding = 200;  // 2000;
    int max_iteration_spinning = 0;
    float contact_recovery_speed = -1;
    double tolerance = 0.01;
    double r_g = 0.02;

    // Perform dynamic tuning of number of threads?
    bool thread_tuning = false;
    
    if(is_parallel){
        ChSystemParallel *casted_system = dynamic_cast<ChSystemParallel*>(vehicle->GetSystem());
        // Set number of threads
        int max_threads = CHOMPfunctions::GetNumProcs();
        if (threads > max_threads)
            threads = max_threads;
        //CHOMPfunctions::SetNumThreads(threads);
        casted_system->SetNumThreads(threads);
        std::cout << "Using " << threads << " threads" << std::endl;

        // Set solver parameters
        casted_system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
        casted_system->GetSettings()->solver.use_full_inertia_tensor = false;
        casted_system->GetSettings()->solver.tolerance = tolerance;

        if(method == ChContactMethod::NSC){
            casted_system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
            casted_system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
            casted_system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
            casted_system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
            casted_system->GetSettings()->solver.alpha = 0;
            casted_system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
            dynamic_cast<ChSystemParallelNSC*>(casted_system)->ChangeSolverType(SolverType::APGD);
            casted_system->GetSettings()->collision.collision_envelope = 0.1 * r_g;
        }
        else{
            casted_system->GetSettings()->solver.contact_force_model = ChSystemSMC::PlainCoulomb;
        }
        casted_system->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
    }
    else {
        vehicle->GetSystem()->SetSolverMaxIterations(50);
    }

    vehicle->GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
    vehicle->GetSystem()->SetMinBounceSpeed(2.0);
}


void TrackedVehicleCreator::Initialize(const ChCoordsys<>& chassisPos, const double chassisFwdVel){

    if(initialized){
        return;
    }

    vehicle->Initialize(chassisPos, chassisFwdVel);
    
    // Set visualization type for vehicle components
    vehicle->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    initialized = true;
}

void TrackedVehicleCreator::Initialize(const ChVector<> position, const ChQuaternion<> orientation, const double chassisFwdVel){
    ChCoordsys<> coords(position, orientation);
    Initialize(coords, chassisFwdVel);
}


//must run after GenerateVehicle()
void TrackedVehicleCreator::SetPowertrain(const std::string& filename){
    if(!powertrain){
        auto pwrtrain = chrono_types::make_shared<SimplePowertrain>(vehicle::GetDataFile(filename));
        vehicle->InitializePowertrain(pwrtrain);
        powertrain = true;
        powertrain_file = filename;
    }
}


void TrackedVehicleCreator::RestrictDOF(bool x, bool y, bool z, bool rot_x, bool rot_y, bool rot_z) {

    DOF[0] = x;
    DOF[1] = y;
    DOF[2] = z;
    DOF[3] = rot_x;
    DOF[4] = rot_y;
    DOF[5] = rot_z;
    

    //If all the vehicle DOF are restricted, just fix the chassis
    if(x && y && z && rot_x && rot_y && rot_z){
        vehicle->GetChassis()->SetFixed(true);
        return;
    }
    else{
        vehicle->GetChassis()->SetFixed(false);
    }
    
    //Creates a contactless, fixed ball and orients it with the chassis
    if(restricted){
        vehicle->GetSystem()->RemoveBody(ball); 
    }
    ball = chrono_types::make_shared<ChBodyEasySphere>(1,1,false, false);
    ball->SetPos(vehicle->GetChassisBody()->GetPos());
    ball->SetRot(vehicle->GetChassisBody()->GetRot());
    ball->SetBodyFixed(true);
    vehicle->GetSystem()->AddBody(ball);
    
    //Links the chassis to the ball, restricting only the degrees of freedom that were passed in as true
    if(restricted){
        vehicle->GetSystem()->RemoveLink(restricter_link);
    }
    restricter_link = chrono_types::make_shared<ChLinkMateFix>();
    restricter_link->Initialize(vehicle->GetChassisBody(), ball);    
    restricter_link->SetConstrainedCoords(x, y, z, rot_x, rot_y, rot_z);
    vehicle->GetSystem()->AddLink(restricter_link);

    restricted = true;
}

void TrackedVehicleCreator::ExportComponentList(const std::string filename) const {
    vehicle->ExportComponentList(filename);
}

Parts TrackedVehicleCreator::ID_To_Part(double id) const {
    int integer_id = (int) id;
    auto part = static_cast<Parts>(integer_id);
    return part;
}

std::shared_ptr<ChBody> TrackedVehicleCreator::Part_To_Body(Parts part, int id) const {
    switch(part){
        case Parts::CHASSIS:
            return vehicle->GetChassisBody();
        case Parts::TRACKSHOE_LEFT:
            return vehicle->GetTrackAssembly(LEFT)->GetTrackShoe(id)->GetShoeBody();
        case Parts::TRACKSHOE_RIGHT:
            return vehicle->GetTrackAssembly(RIGHT)->GetTrackShoe(id)->GetShoeBody();
        case Parts::SPROCKET_LEFT:
            return vehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody();
        case Parts::SPROCKET_RIGHT:
            return vehicle->GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody();
        case Parts::IDLER_LEFT:
            return vehicle->GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody();
        case Parts::IDLER_RIGHT:
            return vehicle->GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody();
        case Parts::ROLLER_LEFT:
            return vehicle->GetTrackAssembly(LEFT)->GetRoller(id)->GetBody();
        case Parts::ROLLER_RIGHT:
            return vehicle->GetTrackAssembly(RIGHT)->GetRoller(id)->GetBody();
        case Parts::ROADWHEEL_LEFT:
            return vehicle->GetTrackAssembly(LEFT)->GetRoadWheelAssembly(id)->GetWheelBody();
        case Parts::ROADWHEEL_RIGHT:
            return vehicle->GetTrackAssembly(RIGHT)->GetRoadWheelAssembly(id)->GetWheelBody();
        default:
            std::cout <<"Not a part!" << std::endl;
            return chrono_types::make_shared<ChBody>();
    }
}

int TrackedVehicleCreator::Part_To_ID(Parts part) const {
    switch(part){
        case Parts::CHASSIS:
            return 0;
        case Parts::TRACKSHOE_LEFT:
            return 1;
        case Parts::TRACKSHOE_RIGHT:
            return 2;
        case Parts::SPROCKET_LEFT:
            return 3;
        case Parts::SPROCKET_RIGHT:
            return 4;
        case Parts::IDLER_LEFT:
            return 5;
        case Parts::IDLER_RIGHT:
            return 6;
        case Parts::ROLLER_LEFT:
            return 7;
        case Parts::ROLLER_RIGHT:
            return 8;
        case Parts::ROADWHEEL_LEFT:
            return 9;
        case Parts::ROADWHEEL_RIGHT:
            return 10;
        default:
            std::cout << "Not a part!" << std::endl;
            return -1;
    }
}

} //end namespace vehicle
} //end namespace chrono
