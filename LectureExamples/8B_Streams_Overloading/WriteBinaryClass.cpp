// demonstrate a not-very-nice way to output entire objects to 
// binary files
#include <fstream>
#include <iostream>

class Sensor
{
public:
  Sensor() : Hits(32), Value(42.37) {}
  void Report() { std::cout << Hits << " " << Value << std::endl; }
  
private:
  int    Hits;
  double Value;
};


int main()
{
  Sensor x;

  std::fstream myFile( "data.bin", std::ios::out | std::ios::binary );
  myFile.write( (char*)&x, sizeof( Sensor ) );
  myFile.close();
  
  Sensor y;
  std::fstream inFile( "data.bin", std::ios::in | std::ios::binary );
  inFile.read( (char*)&y, sizeof( Sensor ) );
  inFile.close();
  y.Report();
}
