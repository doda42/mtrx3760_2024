// example showing reading and writing text files using streams
#include <fstream>
#include <iostream>
#include <string>

int main () 
{
  std::ofstream OutFile("example.txt");
  if( OutFile.is_open() ) {
    int a = 3;
    std::string b("Hello");
    double c = 42.37;
    
    OutFile << a << " " << b << " " << c << std::endl;
    OutFile.close();
  } else {
    std::cout << "Unable to open for writing" << std::endl;
  }
  
  std::ifstream InFile("example.txt");
  if( InFile.is_open() ) {
    int a;
    std::string b;
    double c;
    InFile >> a >> b >> c;
    std::cout << a << " " << b << " " << c << std::endl;
    InFile.close();
  } else {
    std::cout << "Unable to open for reading" << std::endl;
  }

}
