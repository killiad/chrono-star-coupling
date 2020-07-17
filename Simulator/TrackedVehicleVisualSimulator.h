#ifndef TRACKED_VEHICLE_VISUAL_SIMULATOR_H
#define TRACKED_VEHICLE_VISUAL_SIMULATOR_H

#include "TrackedVehicleSimulator.h"

namespace chrono{
namespace vehicle{

class TrackedVehicleVisualSimulator : public TrackedVehicleSimulator {

	public:

		TrackedVehicleVisualSimulator(std::shared_ptr<TrackedVehicleCreator> userVehicle);

		virtual void InitializeSimulation(const std::string& driver_file) override;

		virtual void DoStep(const std::vector<Parts>& parts_list = std::vector<Parts>()) override;

		virtual void RunSimulation(const std::string& driver_file, const std::vector<Parts> &parts_list = std::vector<Parts>()) override;

		virtual void RunSyncedSimulation(const std::string& driver_file, const std::vector<Parts> &parts_list = std::vector<Parts>()) override;

	private:

		std::shared_ptr<ChTrackedVehicleIrrApp> app;

		std::shared_ptr<ChIrrGuiDriver> driver;

		ChRealtimeStepTimer realtime_timer;

};

}
}

#endif
