#ifndef TRACKED_VEHICLE_SIMULATOR_H
#define TRACKED_VEHICLE_SIMULATOR_H

#include "chrono_postprocess/ChPovRay.h"
#include "chrono_postprocess/ChPovRayAssetCustom.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono/core/ChRealtimeStep.h"

#include "../Creator/TrackedVehicleCreator.h"
#include "../CSV/CSVReader.h"
#include "../CSV/CSVMaker.h"

#include <filesystem>
#include "cstdio"

// If Irrlicht support is available...
#ifdef CHRONO_IRRLICHT
// ...include additional headers
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"

#endif

namespace chrono{
namespace vehicle{

enum class Terrain { RIGID, SCM_DEFORMABLE };

class TrackedVehicleSimulator {

	public:
		
		//INPUT: Shared pointer of a TrackedVehicleCreator that contains the vehicle that will be simulated
		//Constructor or class. Note that if userVehicle gets modified, then this classes VehicleCreator gets
		//modified since userVehicle is a shared pointer
		TrackedVehicleSimulator(std::shared_ptr<TrackedVehicleCreator> userVehicle);

		//Initialize the solver. Runs PSOR by default
		void SetSolver();

		//INPUT: time step of the simulation
		//set the simulation time step
		void SetTimeStep(double step);

		//INPUT: If you want the simulation to export csv files
		//set true to export the csv files
		void SetCSV(bool export_data);

		//INPUT: if you want the simulation to export post processing data
		//Set post processing
		void SetPostProcess(bool process);

		//If postprocessor is true, this function will initialize and set up post procesing
		void InitializePostProcess();

		//INPUT: file to generate terrain (JSON if Rigid, mesh if SCM_Deformable), terrain type based off enum above
		//Sets a terrain
		void SetTerrain(const std::string& filename, Terrain type);

		//INPUT: Time, in seconds, on how long the simulation will last
		//Sets how long the simulation will run, in seconds
		void SetSimulationLength(double seconds);

        //INPUT: After how many frames the user wants to save the simulation and the prefix for the save file names
        //This will set how often the simulation will save
        void SetSaveProperties(int interval, std::string file_prefix = "saved_data");

        //This function will save data in CSV format to be read by the TrackedVehicleCreator's constructor
        //if one wishes to stop the simulation then start again later.
        void SaveData() const;
		
        //This function will run a couple time steps with a fixed vehicle to ensure everything is properly initialized
        //before the actual simulation is ran.
        void InitializeModel();

		//Returns the terrain. This is a shared pointer, so edits made to the terrain will carry
		//over in this simulation class.
		inline std::shared_ptr<ChTerrain> GetTerrain() { return terrain; }
	
		//Returns the solver. This is a shared pointer, so edits made to the solver will carry
		//over in this simulation class.
		inline std::shared_ptr<ChSolverPSOR> GetSolver() { return solver; }

		//Returns if simulation will post process
		inline bool GetPostProcess() const { return run_postprocesser; }

		//Returns length of the simulation
		inline double GetSimulationLength() const { return tend; }

		//Returns time step length
		inline double GetTimeStep() const { return step_size; }

		//Returns if simulation will export CSV files
		inline bool getExportCSV() const { return makeCSV; }

		//Returns how much time has passed in the simulaiton
		inline double GetTime() const { return time_passed; }

		//Returns how many simulation frames has passed
		inline int getFrameCount() const { return frameCount; }

		//Returns how many frames have been rendered for post processing
		inline int getRenderCount() const {return renderCount; }

        //INPUT: file that contains information on steering, throttle, and breaking, and parts whose data
        //will be exported
		//Run the simulation, printing info to the terminal or to a CSV file
		virtual void RunSimulation(const std::string& driver_file, const std::vector<Parts> &parts_list = std::vector<Parts>()) = 0;

        //INPUT: file that contains information on steering, throttle, and breaking, and parts whose data
        //will be exported
        //Run the simulation, exporting info into a CSV file automatically. Note that this function has no possibility for
        //not printing info into CSV files, so SetExportCSV() is irrelevent. This function will run with syncronosouly 
        //with STAR-CCM+, and use CSV files for communication.
        virtual void RunSyncedSimulation(const std::string& driver_file, const std::vector<Parts> &parts_list = std::vector<Parts>()) = 0;

		//Must be called before DoTimeStep. This will set everything up for the simulation.
		//INPUT: file that will tell vehicle when to steer, throttle, and break
		virtual void InitializeSimulation(const std::string& driver_file) = 0;

		//PRECONDITIONS: Assumes IntiliazeSimulation has been called
		//INPUT: Parts that will be outputed either to the terminal or to a csv file
        //Runs the time step
		virtual void DoStep(const std::vector<Parts>& parts_list = std::vector<Parts>()) = 0;

	protected:

		std::shared_ptr<TrackedVehicleCreator> vehicleCreator;

		std::shared_ptr<TrackedVehicle> vehicle;

		std::shared_ptr<ChTerrain> terrain;

		std::shared_ptr<ChSolverPSOR> solver;

        std::string prefix;

		//length of simulation
		double tend;

		double step_size;

		double time_passed;
		
        bool run_postprocesser;

		bool makeCSV;

        bool sim_initialized;

        bool model_initialized;

		bool terrain_exists;

		int frameCount;

		int renderCount;

        int save_interval;

		postprocess::ChPovRay pov_exporter;

		BodyStates shoe_states_left;

		BodyStates shoe_states_right;

		TerrainForces shoe_forces_left;

		TerrainForces shoe_forces_right;
};

}
}

#endif
