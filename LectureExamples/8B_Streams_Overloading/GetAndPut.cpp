// example showing reading and writing a single char at a time
#include <fstream>
#include <iostream>
#include <string>

int main () 
{
  std::ifstream InFile("GetAndPut.cpp");
  std::ofstream OutFile("Copy.cpp");

  while( OutFile.good() && InFile.good() && !InFile.eof() )
  {
    char NextChar =  InFile.get();
    OutFile.put( NextChar );
  }
  
  InFile.close();
  OutFile.close();

  std::cout.put('h').put('i').put('\n');
}
