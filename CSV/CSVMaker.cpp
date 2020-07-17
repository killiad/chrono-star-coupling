#include "CSVMaker.h"

namespace chrono{

CSVMaker::CSVMaker(std::string fileName){
	
	file_name = fileName;
    //sets output to append mode
	output.open(file_name, std::ios::app);
}

CSVMaker::~CSVMaker(){
    
    if(output.is_open()){
        output.close();
    }
}

int CSVMaker::Clear(){
  int out = remove(file_name.c_str());
  output.close();
  output.open(file_name);
  return out;
}

void CSVMaker::BodyToCSV(std::shared_ptr<ChBody> body, int gen_ID, int spec_ID) {
    Add(gen_ID);
    AddComma();
    Add(spec_ID);
    AddComma();
    AddVector(body->GetPos());
    AddComma();
    AddMatrix(ChMatrix33<>(body->GetRot()));
  }

void CSVMaker::SaveBodyData(std::shared_ptr<ChBody> body, int gen_ID, int spec_ID) {
    Add(gen_ID);
    AddComma();
    Add(spec_ID);
    AddComma();
    AddVector(body->GetPos());
    AddComma();
    AddQuaternion(body->GetRot());
    AddComma();
    AddVector(body->GetPos_dt());
    AddComma();
    AddQuaternion(body->GetRot_dt());
    AddComma();
    AddVector(body->GetPos_dtdt());
    AddComma();
    AddQuaternion(body->GetRot_dtdt());
    AddComma();
    AddVector(body->Get_accumulated_force());
    AddComma();
    AddVector(body->Get_accumulated_torque());
}

} //end namespace chrono
