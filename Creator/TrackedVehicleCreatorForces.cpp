#include "TrackedVehicleCreator.h"

namespace chrono{
namespace vehicle{

void TrackedVehicleCreator::AddForce(Parts part, int id, ChVector<double> force, double time){

    auto body = Part_To_Body(part, id);
    body->UpdateForces(time);
    body->Accumulate_force(force, body->GetPos(), false);
    body->UpdateForces(time);
}

void TrackedVehicleCreator::AddTorque(Parts part, int id, ChVector<double> force, double time){

    auto body = Part_To_Body(part, id);
    body->UpdateForces(time);
    body->Accumulate_torque(force, true);
    body->UpdateForces(time);
}

//Default value for id is -1. If id == -1, function will
//clear all added forces of all parts othe type tht was passed in
void TrackedVehicleCreator::ClearAddedForces(Parts part, int id){

	//For the part passed in, clear the added forces
    switch(part){

        case Parts::CHASSIS:
            vehicle->GetChassisBody()->Empty_forces_accumulators();
            break;

		case Parts::TRACKSHOE_LEFT:
		{
			int tracke_shoe_num = vehicle->GetTrackAssembly(LEFT)->GetNumTrackShoes();
			if(id == -1){
				for(int i = 0; i < tracke_shoe_num; ++i){
					vehicle->GetTrackAssembly(LEFT)->GetTrackShoe(i)->GetShoeBody()->Empty_forces_accumulators();
				}
			}
			else{
				vehicle->GetTrackAssembly(LEFT)->GetTrackShoe(id)->GetShoeBody()->Empty_forces_accumulators();
			}
			break;
		}

		case Parts::TRACKSHOE_RIGHT:
		{
			int tracke_shoe_num = vehicle->GetTrackAssembly(RIGHT)->GetNumTrackShoes();
			if(id == -1){
				for(int i = 0; i < tracke_shoe_num; ++i){
					vehicle->GetTrackAssembly(RIGHT)->GetTrackShoe(i)->GetShoeBody()->Empty_forces_accumulators();
				}
			}
			else{
				vehicle->GetTrackAssembly(RIGHT)->GetTrackShoe(id)->GetShoeBody()->Empty_forces_accumulators();
			}
			break;
		}

		case Parts::SPROCKET_LEFT:
			vehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody()->Empty_forces_accumulators();
			break;

		case Parts::SPROCKET_RIGHT:
			vehicle->GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody()->Empty_forces_accumulators();
			break;

		case Parts::IDLER_LEFT:
			vehicle->GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody()->Empty_forces_accumulators();
			break;

		case Parts::IDLER_RIGHT:
			vehicle->GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody()->Empty_forces_accumulators();
			break;

		case Parts::ROLLER_LEFT:
		{
			int roller_num = vehicle->GetTrackAssembly(LEFT)->GetNumRollers();
			if(id == -1){
				for(int i = 0; i < roller_num; ++i){
					vehicle->GetTrackAssembly(LEFT)->GetRoller(id)->GetBody()->Empty_forces_accumulators();
				}
			}
			else{
				vehicle->GetTrackAssembly(LEFT)->GetRoller(id)->GetBody()->Empty_forces_accumulators();
			}
			break;
		}

		case Parts::ROLLER_RIGHT:
		{
			int roller_num = vehicle->GetTrackAssembly(RIGHT)->GetNumRollers();
			if(id == -1){
				for(int i = 0; i < roller_num; ++i){
					vehicle->GetTrackAssembly(RIGHT)->GetRoller(id)->GetBody()->Empty_forces_accumulators();
				}
			}
			else{
				vehicle->GetTrackAssembly(RIGHT)->GetRoller(id)->GetBody()->Empty_forces_accumulators();
			}
			break;
		}

		case Parts::ROADWHEEL_LEFT:
		{
			if(id == -1){
				ChRoadWheelAssemblyList assemblies  = vehicle->GetTrackAssembly(LEFT)->GetRoadWheelAssemblies();
				int i = 0;
				for(auto assembly : assemblies){
					assembly->GetWheelBody()->Empty_forces_accumulators();
				}
			}
			else{
				vehicle->GetTrackAssembly(LEFT)->GetRoadWheelAssembly(id)->GetWheelBody()->Empty_forces_accumulators();
			}
			break;
		}

		case Parts::ROADWHEEL_RIGHT:
		{
			if(id == -1){
				ChRoadWheelAssemblyList assemblies  = vehicle->GetTrackAssembly(RIGHT)->GetRoadWheelAssemblies();
				int i = 0;
				for(auto assembly : assemblies){
					assembly->GetWheelBody()->Empty_forces_accumulators();
				}
			}
			else{
				vehicle->GetTrackAssembly(RIGHT)->GetRoadWheelAssembly(id)->GetWheelBody()->Empty_forces_accumulators();
			}
			break;
		}

		default:
			std::cout << "Not a part!" << std::endl;
			return;
	}
}


} //end namespace vehicle
} //end namespace chrono
