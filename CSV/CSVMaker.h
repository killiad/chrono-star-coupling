#ifndef CSVMAKER_H
#define CSVMAKER_H

#include "chrono/physics/ChBody.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"

#include <cstdio>
#include <fstream>


namespace chrono{

class CSVMaker
{
  private:
	 
   std::string file_name;
	 
   std::ofstream output;

  public:

    //fileName is the output file. The CSV maker will automatically append data to the end of the file.  
  	CSVMaker(std::string fileName="output.csv");

    //Destructor
    ~CSVMaker();

    //Creates a new line
  	inline void NewLine()
  	{
  		output << "\n";
  	}

    //Returns the name of the file
  	inline std::string GetName()
  	{
  		return file_name;
  	}

    //Adds a comma. Used to seperate values in the same row
  	inline void AddComma(){
  		output << ",";
  	}

    //Closes the file
  	inline void Close()
  	{
  		output.close();
  	}

    //Adds a value to the cell. Does not automatically add a comma.
    //If one wishes to add a comma, one can just pass a comma in on
    //the end of the parameter if it is a string or one can use AddComma()
  	template <class T>
  	inline void Add(T word)
  	{
  		output << word;
  	}
 	
    //A vector of form <x,y,x> becomes x,y,z in the CSV file. Notice there is
    //no comma after z.
 	  template <class T>
  	void AddVector(ChVector<T> vec);

    //A quaternion of form (a,b,c,d) becomes a,b,c,d in the CSV file. Notice there is
    //no comma after d.
    template <class T>
    void AddQuaternion(ChQuaternion<T> quat);

    //A matrix of the form  [a,b,c] will become a,b,c,d,e,f,h,i,j in the CSV file. Notice
    //                      [d,e,f] there is no comma after j.
    //                      [h,i,j]
    template <class T>
    void AddMatrix(ChMatrix33<T> matrix);

    //Clears the current CSV file of all data by deleting the file than making a fresh, blank
    //file.
    int Clear();

    //Adds relavant data for coupling with STAR-CCM+
    void BodyToCSV(std::shared_ptr<ChBody> body, int gen_ID, int spec_ID);

    //Saves data of part that is passed in. This is used so save the current state of the simulation.
    void SaveBodyData(std::shared_ptr<ChBody> body, int gen_ID, int spec_ID);
};

} //end namespace chrono


namespace chrono{

template <class T>
void CSVMaker::AddVector(ChVector<T> vec){

	output << vec.x() << "," << vec.y() << "," << vec.z();
}

template <class T>
void CSVMaker::AddQuaternion(ChQuaternion<T> quat){
  output << quat.e0() << "," << quat.e1() << "," << quat.e2() << "," << quat.e3();
}

template <class T>
void CSVMaker::AddMatrix(ChMatrix33<T> matrix){
  for(int row = 0; row < 3; ++row) {
    for(int col = 0; col < 3; ++col) {
      output << matrix(row, col);
      if(row != 2 || col != 2){
        AddComma();
      }
    }
  }
}

} //end namespace chrono

#endif 
