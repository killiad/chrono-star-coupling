#include "Creator/TrackedVehicleCreator.h"
#include "Terrain/TerrainCreator_Rigid.h"
#include "Terrain/TerrainCreator_SCMDeformable.h"
#include "Terrain/TerrainCreator_Flat.h"
#include "Terrain/TerrainCreator_Granular.h"
#include "Terrain/TerrainCreator_FEADeformable.h"
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
	std::string terrain_file("terrain/templates/Granular.csv");
	// Driver input file
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
	runningGear->SetPowertrain(simplepowertrain_file);
    auto simulator = chrono_types::make_shared<TrackedVehicleNonVisualSimulator>(runningGear);
    
    runningGear->SetSolver(2);
	runningGear->RestrictDOF(true, true, true, true, true, true);
    auto terrain = chrono_types::make_shared<TerrainCreator_Granular>(terrain_file, runningGear->GetVehicle());
	simulator->SetSimulationLength(2.0);
	simulator->SetTimeStep(1e-3);
	simulator->SetCSV(true);
    simulator->SetLogInfo(true, true);
    simulator->SetTerrain(terrain->GetTerrain());
	simulator->RunSimulation(driver_file, data);

	return 0;
}
