#ifndef CSVREADER_H
#define CSVREADER_H

#include "chrono/core/ChVector.h"
#include "chrono/core/ChStream.h"

#include <cstdio>
#include <fstream>
#include <string>

namespace chrono{

//Specialized class to aid in parzing CSV files with only numerical data.
class CSVReader{

    public:

        //Constructor. Input CSV you wish to read. The constructor will open the file automatically
        //and load the first line
        CSVReader(const std::string filename);

        //Destructor
        ~CSVReader();

        //Loads a line into the buffer. If GetLine is used twice, a line of data
        //is effectively skipped over, which could be desirable if one wants to skip nonumerical
        //data. If one uses GetLine right after instantiation, then the first line of data is skipped.
        //Generally, this function should only be called externally when the user wants to skip a line.
        void GetLine();

        //Checks to see if the first character of the line is a number. If so, it will return true. Note
        //that we asssume that if the first character is a number, than all characters will be numbers or
        //decimal points
        bool IsValidRow();

        //Returns the number at the current cell, then moves the cursor to the next cell of data
        //Assumes ValidInput() = true
        double GetNumber();

        //Gets the next three numbers, then returns them as a vector. Note that this fuction
        //assumes that the next three cells are numerical
        ChVector<> GetVector();

        //opens the passed in file
        bool Open(const std::string filename);

        //Closes the current file
        void Close();

        //Get the current row of the file, as a string
        inline std::string GetRow() { return row; }

    private:

        std::ifstream input;

        std::string row;

        size_t* processed;

        int cursor;

};

}//end namespace chrono
#endif
