#ifndef TRACKED_VEHICLE_CREATOR_H
#define TRACKED_VEHICLE_CREATOR_H

#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "../CSV/CSVMaker.h"
#include "../CSV/CSVReader.h"


namespace chrono{
namespace vehicle{

//Use this enum to pass around references to different parts of the vehicle
enum class Parts { CHASSIS,
                   TRACKSHOE_LEFT, TRACKSHOE_RIGHT,
				   SPROCKET_LEFT, SPROCKET_RIGHT, 
				   IDLER_LEFT, IDLER_RIGHT,
				   ROLLER_LEFT, ROLLER_RIGHT,
				   ROADWHEEL_LEFT, ROADWHEEL_RIGHT };

//Struct that is used to store data about the vehicle
struct VehicleInfo {
    int Left_TrackShoeNum;
    int Right_TrackShoeNum;
    int Left_RollerNum;
    int Right_RollerNum;
    int Left_RoadWheelNum;
    int Right_RoadWheelNum;
    double Mass;
};

//Class to aid in constructing the vehicle and dealing with relevant data about the vehicle
class TrackedVehicleCreator {

	public:

		//Constructor. Takes in a JSON file, a contact method, and a boolean on whether or not the system is
        //parallel and sets the respective vehicle. 
		TrackedVehicleCreator(const std::string& filename, ChContactMethod method = ChContactMethod::NSC, bool parallel = false); 
		
        //Constructor for loading simulation data from a previously saved simulation. Expects a CSV
        //TrackedVehicleCreator(std::string save_file);

		//Initialize the vehicle. Must be called if client wants default initialization overriden, but by default is called inthe constructor
		void Initialize(const ChCoordsys<>& chassisPos = ChCoordsys<>(ChVector<>(0,0,1.2), QUNIT), const double chassisFwdVel = 0.0);

        //Same as above, but uses individual position and orientation data instead of just a coordinate system
        void Initialize(const ChVector<> position, const ChQuaternion<> orientation, const double chassisFwdVel = 0.0);

        /*//This function will save data in CSV format to be read by the TrackedVehicleCreator's constructor
        //if one wishes to stop the simulation then start again later.
        void SaveData(std::string prefix, double time_passed) const;

        //Called after calling the constructor that loads saved data ONLY
        void LoadData();*/

		//Set the powertrain. InputL JSON file
		void SetPowertrain(const std::string& filename);

		//Initialize the solver
		void SetSolver(int threads = 1);

    	//Called during simulation, but can be called outside simulation if client wished. Prints info to the terminal about
		//the parts passed in via the vector
		void ExportData(const std::vector<Parts> &parts_list) const;

		//Outputs desired data about parts that are passed in via the vector into a csv file. Pass in the parts to export
        //along with the file name to export the data to. CSV file will be of the form:
        //General ID, Specific ID, Position vector (3 columns), rotation matrix (9 columns)
		void ExportData(const std::vector<Parts> &parts_list, std::string &filename) const;

		//Exports json list of all component parts of the vehicle
		//INPUT: file name for JSON file
		void ExportComponentList(const std::string filename) const;

		//Pass in true for which degrees of freedom you wish to restrict. Pass in false for the DOF
        //you wish to remain free. For the DOF that are restricted, the vehicle will not move in that way.
        //For example, restricting z will resist gravity.
        void RestrictDOF(bool x, bool y, bool z, bool rot_x, bool rot_y, bool rot_z);

		//Add a force to a part of the vehicle. For the time parameter, pass in the current
        //time of the simulation.
		void AddForce(Parts part, int id, ChVector<double> force, double time);

		//Add a force to a part of the vehicle. For the time parameter, pass in the current
        //time of the simulation
		void AddTorque(Parts part, int id, ChVector<double> torque, double time);

		//Removes added forces and torques. It is the responsibility of the user to make sure forces are cleared before they add new ones
		//The id is set by default to -1 since if nothing is passed in, it will clear the forces for all the bodies of the specified part.
		//For example, for the track shoe, this is the difference between clearing a specific track shoe or all track shoes.
		void ClearAddedForces(Parts part, int id = -1);

        //This function takes in a whole number double. It will return the corresponding part in enum form, performing
        //the necessary casts to do so. If id is not a valid part id, then function is undefined
        Parts ID_To_Part(double id);

        //Takes in a part and returns the ID. This function is necessary because Parts is a strongly typed enum, which does
        //not implicitely cast to an int. However, it will prevent naming clashes that may happen in the future otherwise.
        int Part_to_ID(Parts part) const;

		inline std::shared_ptr<TrackedVehicle> GetVehicle() { return vehicle; }

        inline VehicleInfo GetVehicleInfo() { return info; }

        inline std::string GetMasterFile() { return master_file; }
        
        inline std::string GetPowertrainFile() { return powertrain_file; }
        
        inline bool* GetDOF() { return DOF; }

        inline bool IsParallel() { return is_parallel; }

	private:
        
        //the actual vehicle system
		std::shared_ptr<TrackedVehicle> vehicle;

        std::string master_file;

        std::string powertrain_file;
        
        std::string saveFile;

        ChVector<> init_pos;

        ChQuaternion<> init_rot;

        double init_Chassis_FwdVel;

        bool DOF[6];

        bool initialized;

        bool powertrain;

        bool restricted;

        bool is_parallel;

        VehicleInfo info;

        std::shared_ptr<ChLinkMateFix> restricter_link;

        std::shared_ptr<ChBodyEasySphere> ball;
};

}//end of vehicle
}//end of chrono

#endif
