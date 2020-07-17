#include "CSVReader.h"

namespace chrono{

CSVReader::CSVReader(const std::string filename) {
    Open(filename);
    processed = new size_t(0);
    std::getline(input, row);
}

CSVReader::~CSVReader(){
    delete processed;
    if(input.is_open()){
        Close();
    }
}

void CSVReader::GetLine(){
    std::getline(input, row);
    cursor = 0;
}

bool CSVReader::IsValidRow(){
    return isdigit(row[0]);
}

double CSVReader::GetNumber(){
    double data;
    
    if(cursor == row.size() + 1){
        GetLine();
    }

    data = std::stod(&row[cursor], processed);
    cursor += *processed + 1;

    return data;
}

bool CSVReader::Open(const std::string filename){
    
    if(input.is_open()){
        Close();
    }
    
    input.open(filename, std::ios::in);
    if(input.is_open()){
        GetLine();
        return true;
    }
    
    return false;
}

void CSVReader::Close(){
    if(input.is_open()){
        input.close();
        cursor = 0;
        *processed = 0;    
    }
}

ChVector<> CSVReader::GetVector(){
    double x = GetNumber();
    double y = GetNumber();
    double z = GetNumber();
    return ChVector<>(x,y,z);
}

}
