// example of setw
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip> // setw

int main () 
{
  std::ofstream OutFile("example.txt");

  OutFile << std::setw(10) << ' ';
  for( int j=0; j<=10; ++j )
    OutFile << std::setw(5) << j;
  OutFile << std::endl;
     
  for( int i=0; i<=10; ++i )
  {
    OutFile << std::setw(10) << i;
    for( int j=0; j<=10; ++j )
    {
      OutFile << std::setw(5) << i*j;
    }
    OutFile << std::endl;    
  }
  
  OutFile.close();
}
