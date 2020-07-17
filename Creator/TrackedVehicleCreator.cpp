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

void TrackedVehicleCreator::Initialize(const ChCoordsys<>& chassisPos, const double chassisFwdVel){
    
    vehicle->Initialize(chassisPos, chassisFwdVel);
    
    // Set visualization type for vehicle components
    vehicle->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);


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
    
    //Creates a contactless, fixed ball and orients it with the chassis
    auto ball = chrono_types::make_shared<ChBodyEasySphere>(1,1,false, false);
    ball->SetPos(vehicle->GetChassisBody()->GetPos());
    ball->SetRot(vehicle->GetChassisBody()->GetRot());
    ball->SetBodyFixed(true);
    vehicle->GetSystem()->AddBody(ball);

    //Links the chassis to the ball, restricting only the degrees of freedom that were passed in as true
    auto restricter = chrono_types::make_shared<ChLinkMateFix>();
    if(restricted){
        vehicle->GetSystem()->RemoveLink(restricter_link);
    }
    restricter_link = restricter;
    restricter_link->Initialize(vehicle->GetChassisBody(), ball);    
    restricter->SetConstrainedCoords(x, y, z, rot_x, rot_y, rot_z);
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
