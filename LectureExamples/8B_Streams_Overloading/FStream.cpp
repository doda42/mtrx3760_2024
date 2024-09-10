// example showing reading and writing a single file, 
// and random access with seekg
#include <fstream>
#include <iostream>
#include <string>

int main () 
{
  std::fstream File("example.txt", std::ios::out | std::ios::in);
  int a = 3;
  std::string b("Hello");
  double c = 42.37;

  File << a << " " << b << " " << c << std::endl;
  File >> a >> b >> c;
  std::cout << a << " " << b << " " << c << std::endl;
  
  File.clear(); // clear the eof bit... we've just read to the end
  File.seekg(3, std::ios::beg); // seek to the 4th char from beginning
  File >> b;
  std::cout << b << std::endl;
  File.close();
}
