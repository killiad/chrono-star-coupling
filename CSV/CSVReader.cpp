#include "CSVReader.h"

namespace chrono{

CSVReader::CSVReader(const std::string filename) {
    Open(filename);
    processed = new size_t(0);
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
    return row != "EOF" && row != "";
}

double CSVReader::GetNumber(){
    double data;
    if(cursor == row.size() + 1){
        GetLine();
    }
    if(!IsValidRow()){ //get rid of?
        return -1;
    }
    
    data = std::stod(&row[cursor], processed);
    cursor += *processed + 1;

    return data;
}

std::string CSVReader::GetString(){
    std::string cell = "";
    if(cursor == row.size() + 1){
        GetLine();
    }

    char letter = row.at(cursor);
    ++cursor;
    while(letter != ',' && letter != '\n'){
        cell.push_back(letter);
        letter = row.at(cursor);
        ++cursor;
    }

    return cell;
}

bool CSVReader::Open(const std::string filename){
    
    if(input.is_open()){
        input.close();
    }
    input.open(filename, std::ios::in);
    if(input.is_open()){
        GetLine();
        return true;
    }
    
    return false;
}

void CSVReader::Close(){
    input.close();
    cursor = 0;
    *processed = 0;    
}

ChVector<> CSVReader::GetVector(){
    double x = GetNumber();
    double y = GetNumber();
    double z = GetNumber();
    return ChVector<>(x,y,z);
}

ChQuaternion<> CSVReader::GetQuaternion(){
    double a = GetNumber();
    double b = GetNumber();
    double c = GetNumber();
    double d = GetNumber();
    return ChQuaternion<>(a,b,c,d);
}

}
