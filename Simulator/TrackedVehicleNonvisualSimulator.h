#ifndef TRACKED_VEHICLE_NONVISUAL_SIMULATOR_H
#define TRACKED_VEHICLE_NONVISUAL_SIMULATOR_H

#include "TrackedVehicleSimulator.h"

namespace chrono{
namespace vehicle{

class TrackedVehicleNonVisualSimulator : public TrackedVehicleSimulator {

	public:

		TrackedVehicleNonVisualSimulator(std::shared_ptr<TrackedVehicleCreator> userVehicle);

		virtual void InitializeSimulation(const std::string& driver_file) override;

		virtual void DoStep(const std::vector<Parts>& parts_list = std::vector<Parts>()) override;

		virtual void RunSimulation(const std::string& driver_file, const std::vector<Parts> &parts_list = std::vector<Parts>()) override;
		
        virtual void RunSyncedSimulation(const std::string& driver_file, const std::vector<Parts> &parts_list = std::vector<Parts>(),
                const int file_ratio = 1) override;

	private:

		std::shared_ptr<ChDataDriver> driver;
};

}
}

#endif
