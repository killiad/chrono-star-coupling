#include "Simulator/TrackedVehicleNonvisualSimulator.h"
#include "Simulator/TrackedVehicleVisualSimulator.h"

using namespace chrono;
using namespace chrono::vehicle;

int main(int argc, char* argv[]) {

	// JSON file for vehicle model
	std::string vehicle_file("M113/vehicle/M113_Vehicle_SinglePin.json");

	// JSON file for powertrain
	std::string simplepowertrain_file("M113/powertrain/M113_SimplePowertrain.json");
	// JSON files for terrain (rigid plane)
	std::string rigidterrain_file("terrain/RigidPlane.json");
	// Driver input file (if not using Irrlicht)
	std::string driver_file("generic/driver/No_Maneuver.txt");
    
	//Vector that stores what type of data will be outputed
	std::vector<Parts> data;
    data.push_back(Parts::CHASSIS);
	data.push_back(Parts::TRACKSHOE_LEFT);
    data.push_back(Parts::TRACKSHOE_RIGHT);
    data.push_back(Parts::SPROCKET_LEFT);
    data.push_back(Parts::SPROCKET_RIGHT);
    data.push_back(Parts::IDLER_LEFT);
    data.push_back(Parts::IDLER_RIGHT);
    data.push_back(Parts::ROLLER_LEFT);
    data.push_back(Parts::ROLLER_RIGHT);
    data.push_back(Parts::ROADWHEEL_LEFT);
    data.push_back(Parts::ROADWHEEL_RIGHT);

    //Initializing Vehicle Creator and Simulator
	auto runningGear = chrono_types::make_shared<TrackedVehicleCreator>(vehicle_file, ChContactMethod::NSC, true);
    ChVector<> chassisPos(0,0,1.2);
    ChQuaternion<> chassisOrientation = QUNIT; 
    runningGear->Initialize(chassisPos, chassisOrientation, 0.0);
    std::shared_ptr<TrackedVehicleSimulator> simulator = chrono_types::make_shared<TrackedVehicleNonVisualSimulator>(runningGear);
    
	runningGear->SetPowertrain(simplepowertrain_file);
    runningGear->SetSolver(2);
	runningGear->RestrictDOF(true, true, true, true, false, true);
    std::cout << "Mass: " << runningGear->GetVehicleInfo().Mass << std::endl;	
    //simulator->SetTerrain(rigidterrain_file, Terrain::RIGID);
	simulator->SetSimulationLength(2.0);
	simulator->SetTimeStep(4e-3);
	simulator->SetCSV(true);
	simulator->RunSyncedSimulation(driver_file, data);

	return 0;
}
