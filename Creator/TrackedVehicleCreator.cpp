#include "TrackedVehicleCreator.h"

namespace chrono{
namespace vehicle{

TrackedVehicleCreator::TrackedVehicleCreator(const std::string& filename, ChContactMethod method) : master_file(filename),
    powertrain_file(""), powertrain(false), restricted(false){

    //// NOTE
    //// When using SMC, a double-pin shoe type requires MKL or MUMPS.  
    //// However, there appear to still be redundant constraints in the double-pin assembly
    //// resulting in solver failures with MKL and MUMPS (rank-deficient matrix).
    ////
    //// For now, use ChContactMethod::NSC for a double-pin track model

    // Create the vehicle system
    //TrackedVehicle veh(vehicle::GetDataFile(filename), contact_method);
    vehicle = chrono_types::make_shared<TrackedVehicle>(vehicle::GetDataFile(filename), method);
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

TrackedVehicleCreator::TrackedVehicleCreator(std::string save_file){
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
}

void TrackedVehicleCreator::SaveData(std::string prefix, double time_passed) const{

    char filename [100];
    
    sprintf(filename, "../Outputs/Saves/%s_%.3f.csv", prefix.c_str(), time_passed);
    std::string fn(filename);
    auto body = chrono_types::make_shared<ChBody>();
    CSVMaker csv(fn);
    csv.Clear();

    //First row contains file name, powertrain file, constact method, time, degrees of freedom, vehicle position, 
    //vehicle orientation, and vehicle speed
    csv.Add(master_file);
    csv.AddComma();
    csv.Add(powertrain_file);
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
        csv.Add(DOF[degree]);
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
    csv.SaveBodyData(body, Part_to_ID(Parts::CHASSIS), 0);
    csv.NewLine();
    
    //Track Shoes
    for(int spec_id = 0; spec_id < info.Left_TrackShoeNum; ++spec_id){
        body = vehicle->GetTrackShoe(LEFT, spec_id)->GetShoeBody();
        csv.SaveBodyData(body, Part_to_ID(Parts::TRACKSHOE_LEFT), spec_id);
        csv.NewLine();
    }
    
    for(int spec_id = 0; spec_id < info.Right_TrackShoeNum; ++spec_id){
        body = vehicle->GetTrackShoe(RIGHT, spec_id)->GetShoeBody();
        csv.SaveBodyData(body, Part_to_ID(Parts::TRACKSHOE_RIGHT), spec_id);
        csv.NewLine();
    }

    //Sprockets
    body = vehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody();
    csv.SaveBodyData(body, Part_to_ID(Parts::SPROCKET_LEFT), 0);
    csv.NewLine();

    body = vehicle->GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody();
    csv.SaveBodyData(body, Part_to_ID(Parts::SPROCKET_RIGHT), 0);
    csv.NewLine();
   
    //Idlers
    body = vehicle->GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody();
    csv.SaveBodyData(body, Part_to_ID(Parts::IDLER_LEFT), 0);
    csv.NewLine();

    body = vehicle->GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody();
    csv.SaveBodyData(body, Part_to_ID(Parts::IDLER_RIGHT), 0);
    csv.NewLine();

    //Rollers
    for(int spec_id = 0; spec_id < info.Left_RollerNum; ++spec_id){
        body = vehicle->GetTrackAssembly(LEFT)->GetRoller(spec_id)->GetBody();
        csv.SaveBodyData(body, Part_to_ID(Parts::ROLLER_LEFT), spec_id);
        csv.NewLine();
    }
    
    for(int spec_id = 0; spec_id < info.Right_RollerNum; ++spec_id){
        body = vehicle->GetTrackAssembly(RIGHT)->GetRoller(spec_id)->GetBody();
        csv.SaveBodyData(body, Part_to_ID(Parts::ROLLER_RIGHT), spec_id);
        csv.NewLine();
    }

    //Road Wheels
    for(int spec_id = 0; spec_id < info.Left_RoadWheelNum; ++spec_id){
        body = vehicle->GetTrackAssembly(LEFT)->GetRoadWheel(spec_id)->GetWheelBody();
        csv.SaveBodyData(body, Part_to_ID(Parts::ROADWHEEL_LEFT), spec_id);
        csv.NewLine();
    }
    
    for(int spec_id = 0; spec_id < info.Right_RoadWheelNum; ++spec_id){
        body = vehicle->GetTrackAssembly(RIGHT)->GetRoadWheel(spec_id)->GetWheelBody();
        csv.SaveBodyData(body, Part_to_ID(Parts::ROADWHEEL_RIGHT), spec_id);
        csv.NewLine();
    }
    
    csv.Add("EOF");
    csv.Close();
    std::cout << "Saved data to " << fn << std::endl; 
}

void TrackedVehicleCreator::LoadData(){
    Initialize(init_pos, init_rot, init_Chassis_FwdVel);
    SetPowertrain(powertrain_file);
    RestrictDOF(DOF[0],DOF[1],DOF[2],DOF[3],DOF[4],DOF[5]);

    double gen_ID;
    double spec_ID;
    ChVector<> pos(VNULL);
    ChQuaternion<> rot(QUNIT);
    ChVector<> pos_dt(VNULL);
    ChQuaternion<> rot_dt(QUNIT);
    ChVector<> pos_dtdt(VNULL);
    ChQuaternion<> rot_dtdt(QUNIT);
    ChVector<> force(VNULL);
    ChVector<> torque(VNULL);

    CSVReader csv(saveFile);
    csv.GetLine();
    while(csv.IsValidRow()){
        
        gen_ID = csv.GetNumber();
        spec_ID = csv.GetNumber();
        pos = csv.GetVector();
        rot = csv.GetQuaternion();
        pos_dt = csv.GetVector();
        rot_dt = csv.GetQuaternion();
        pos_dtdt = csv.GetVector();
        rot_dtdt = csv.GetQuaternion();
        force = csv.GetVector();
        torque = csv.GetVector();

    }
    csv.Close();
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

Parts TrackedVehicleCreator::ID_To_Part(double id){
    int integer_id = (int) id;
    auto part = static_cast<Parts>(integer_id);
    return part;
}

int TrackedVehicleCreator::Part_to_ID(Parts part) const {
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
