#include "TrackedVehicleCreator.h"

namespace chrono{
namespace vehicle{

void TrackedVehicleCreator::ExportData(const std::vector<Parts>& part_list) const {

    //For every part in the vector, print its relevent data
    for(auto part : part_list) {

        switch(part){

            case Parts::TRACKSHOE_LEFT:
            {
                size_t track_shoe_num = vehicle->GetTrackAssembly(LEFT)->GetNumTrackShoes();
                std::cout << "LEFT TRACKSHOE" << std::endl;
                std::cout << "Number of trackshoes: " << track_shoe_num << std::endl;
                std::vector<BodyState> states(track_shoe_num);
                vehicle->GetTrackShoeStates(LEFT, states);

                for(size_t i = 0; i < track_shoe_num; ++i) {
                    std::cout << "Trackshoe ID: " << i << " Position: " << states[i].pos
                              << " Linear Veloctiy: " << states[i].lin_vel 
                              << " Rotation: " << states[i].rot << " Angular Velocity: " 
                              << states[i].ang_vel << std::endl;
                }

                std::cout << std::endl;
                break;
            }


            case Parts::TRACKSHOE_RIGHT:
            {
                size_t track_shoe_num = vehicle->GetTrackAssembly(RIGHT)->GetNumTrackShoes();
                std::cout << "RIGHT TRACKSHOE" << std::endl;
                std::cout << "Number of trackshoes: " << track_shoe_num << std::endl;
                std::vector<BodyState> states(track_shoe_num);
                vehicle->GetTrackShoeStates(RIGHT, states);

                for(int i = 0; i < track_shoe_num; ++i) {
                    std::cout << "Trackshoe ID: " << i << " Position: " << states[i].pos
                              << " Linear Veloctiy: " << states[i].lin_vel 
                              << " Rotation: " << states[i].rot << " Angular Velocity: " 
                              << states[i].ang_vel << std::endl;
                }

                std::cout << std::endl;
                break;
            }

            case Parts::SPROCKET_LEFT:
                std::cout << "LEFT SPROCKET" << std::endl;
                std::cout << "Number of Teeth: " << vehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetNumTeeth() << std::endl;
                std::cout << "Position: " << vehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody()->GetPos() << std::endl;
                std::cout << std::endl;
                break;

            case Parts::SPROCKET_RIGHT:
                std::cout << "RIGHT SPROCKET" << std::endl;
                std::cout << "Number of Teeth: " << vehicle->GetTrackAssembly(RIGHT)->GetSprocket()->GetNumTeeth() << std::endl;
                std::cout << "Position: " << vehicle->GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody()->GetPos() << std::endl;
                std::cout << std::endl;
                break;

            case Parts::IDLER_LEFT:
                std::cout << "LEFT IDLER" <<std::endl;
                std::cout << "Position: " << vehicle->GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody()->GetPos() << std::endl;
                std::cout << std::endl;
                break;

            case Parts::IDLER_RIGHT:
                std::cout << "RIGHT IDLER" <<std::endl;
                std::cout << "Position: " << vehicle->GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody()->GetPos() << std::endl;
                std::cout << std::endl;
                break;

            case Parts::ROLLER_LEFT:
            {
                size_t roller_num = vehicle->GetTrackAssembly(LEFT)->GetNumRollers();
                std::cout << "LEFT ROLLER" << std::endl;
                std::cout << "Number of rollers: " << roller_num << std::endl;
                
                for(int i = 0; i < roller_num; ++i) {
                    std::cout << "Mass: " << vehicle->GetTrackAssembly(LEFT)->GetRoller(i)->GetMass() << std::endl;
                    std::cout << "Intertia: " << vehicle->GetTrackAssembly(LEFT)->GetRoller(i)->GetInertia() << std::endl;
                    std::cout << "Position: " << vehicle->GetTrackAssembly(LEFT)->GetRoller(i)->GetBody()->GetPos() << std::endl;
                }
                
                std::cout << std::endl;
                break;
            }

            case Parts::ROLLER_RIGHT:
            {
                size_t roller_num = vehicle->GetTrackAssembly(RIGHT)->GetNumRollers();
                std::cout << "RIGHT ROLLER" << std::endl;
                std::cout << "Number of rollers: " << roller_num << std::endl;

                for(int i = 0; i < roller_num; ++i) {
                    std::cout << "Mass: " << vehicle->GetTrackAssembly(RIGHT)->GetRoller(i)->GetMass() << std::endl;
                    std::cout << "Intertia: " << vehicle->GetTrackAssembly(RIGHT)->GetRoller(i)->GetInertia() << std::endl;
                    std::cout << "Position: " << vehicle->GetTrackAssembly(RIGHT)->GetRoller(i)->GetBody()->GetPos() << std::endl;
                }

                std::cout << std::endl;
                break;
            }

            case Parts::ROADWHEEL_LEFT:
            {
                std::cout << "LEFT ROADWHEEL" << std::endl;
                ChRoadWheelAssemblyList assemblies  = vehicle->GetTrackAssembly(LEFT)->GetRoadWheelAssemblies();
                std::cout << "Number of RoadWheels on Left Side: " << assemblies.size() << std::endl;
                int i = 0;
                
                for(auto assembly : assemblies) {
                    std::cout << "Wheel " << i << " Position: " << assembly->GetWheelBody()->GetPos() << std::endl;
                    ++i;
                }

                std::cout << std::endl;
                break;
            }

            case Parts::ROADWHEEL_RIGHT:
            {
                std::cout << "RIGHT ROADWHEEL" << std::endl;
                ChRoadWheelAssemblyList assemblies  = vehicle->GetTrackAssembly(RIGHT)->GetRoadWheelAssemblies();
                std::cout << "Number of RoadWheels on Right Side: " << assemblies.size() << std::endl;
                int i = 0;
                
                for(auto assembly : assemblies) {
                    std::cout << "Wheel " << i << " Position: " << assembly->GetWheelBody()->GetPos() << std::endl;
                    ++i;
                }

                std::cout << std::endl;
                break;
            }

            case Parts::CHASSIS:
            {
              auto chassis = vehicle->GetChassisBody();
              std::cout << "CHASSIS" << std::endl;
              std::cout << "Position: " << chassis->GetPos() << std::endl;
              std::cout << "Orientation: " << chassis->GetRot() << std::endl;
              std::cout<< std::endl;
              break;
            }

            default:
                std::cout << "Not a part" << std::endl << std::endl;
                break;

        }
    }
}

void TrackedVehicleCreator::ExportData(const std::vector<Parts> &part_list, std::string &filename) const {
    
    //Labeling columns in first row of CSV file
    CSVMaker csv(filename);
    int gen_ID = 0;
    int spec_id = 0;
    std::shared_ptr<ChBody> body;
    std::cout << "Creating file: " << filename << std::endl;
    csv.Clear();
    csv.Add("General_ID,");
    csv.Add("Specific_ID,");
    csv.Add("Position_X,");
    csv.Add("Position_Y,");
    csv.Add("Position_Z,");
    csv.Add("Rotation_00,");
    csv.Add("Rotation_01,");
    csv.Add("Rotation_02,");
    csv.Add("Rotation_10,");
    csv.Add("Rotation_11,");
    csv.Add("Rotation_12,");
    csv.Add("Rotation_20,");
    csv.Add("Rotation_21,");
    csv.Add("Rotation_22");
    csv.NewLine();
   
    //For every part in the vector, export its position and orientation to the csv file
    for(auto part : part_list) {

        switch(part){

            case Parts::CHASSIS:
                body = vehicle->GetChassisBody();
                csv.BodyToCSV(body, Part_to_ID(part), 0);
                csv.NewLine();
                break;

            case Parts::TRACKSHOE_LEFT:
                for(int spec_id = 0; spec_id < info.Left_TrackShoeNum; ++spec_id){
                    body = vehicle->GetTrackAssembly(LEFT)->GetTrackShoe(spec_id)->GetShoeBody();
                    csv.BodyToCSV(body, Part_to_ID(part), spec_id);
                    csv.NewLine();
                }
                break;

            case Parts::TRACKSHOE_RIGHT:
                for(int spec_id = 0; spec_id < info.Right_TrackShoeNum; ++spec_id){
                    body = vehicle->GetTrackAssembly(RIGHT)->GetTrackShoe(spec_id)->GetShoeBody();
                    csv.BodyToCSV(body, Part_to_ID(part), spec_id);
                    csv.NewLine();
                }
                break;

            case Parts::SPROCKET_LEFT:
                body=vehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody();
                csv.BodyToCSV(body, Part_to_ID(part), 0);
                csv.NewLine();
                break;

            case Parts::SPROCKET_RIGHT:
                body=vehicle->GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody();
                csv.BodyToCSV(body, Part_to_ID(part), 0);
                csv.NewLine();
                break;

            case Parts::IDLER_LEFT:
                body = vehicle->GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody();
                csv.BodyToCSV(body, Part_to_ID(part), 0);
                csv.NewLine();
                break;

            case Parts::IDLER_RIGHT:
                body = vehicle->GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody();
                csv.BodyToCSV(body, Part_to_ID(part), 0);
                csv.NewLine();
                break;

            case Parts::ROLLER_LEFT:
                for(int spec_id = 0; spec_id < info.Left_RollerNum; ++spec_id) {
                    body = vehicle->GetTrackAssembly(LEFT)->GetRoller(spec_id)->GetBody();
                    csv.BodyToCSV(body, Part_to_ID(part), spec_id);
                    csv.NewLine();
                }
                break;

            case Parts::ROLLER_RIGHT:
                for(int spec_id = 0; spec_id < info.Right_RollerNum; ++spec_id) {
                    body = vehicle->GetTrackAssembly(RIGHT)->GetRoller(spec_id)->GetBody();
                    csv.BodyToCSV(body, Part_to_ID(part), spec_id);
                    csv.NewLine();
                }
                break;

            case Parts::ROADWHEEL_LEFT:
                for(int spec_id = 0; spec_id < info.Left_RoadWheelNum; ++spec_id) {
                    body = vehicle->GetTrackAssembly(LEFT)->GetRoadWheel(spec_id)->GetWheelBody();
                    csv.BodyToCSV(body, Part_to_ID(part), spec_id);
                    csv.NewLine();
                }
                break;

            case Parts::ROADWHEEL_RIGHT:
                for(int spec_id = 0; spec_id < info.Right_RoadWheelNum; ++spec_id) {
                    body = vehicle->GetTrackAssembly(RIGHT)->GetRoadWheel(spec_id)->GetWheelBody();
                    csv.BodyToCSV(body, Part_to_ID(part), spec_id);
                    csv.NewLine();
                }
                break;

            default:
                std::cout << "Not a part" << std::endl << std::endl;
                break;
        }
    }
    csv.Close();
}

} //end namespace vehicle
} //end namespace chrono
