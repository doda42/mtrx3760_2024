// example showing getline for dealing with files line by line
#include <fstream>
#include <iostream>
#include <iomanip>

int main () 
{
  char LineBuff[256];
  int LineCount = 0;
  
  std::ifstream InFile("FileGetLine.cpp");
  while( InFile.good() )
  {
    InFile.getline( LineBuff, 256 );
    if( !InFile.good() )
      break;
    std::cout << std::setw(5) << LineCount << '\t' << LineBuff << std::endl;
    ++LineCount;
  }

}
