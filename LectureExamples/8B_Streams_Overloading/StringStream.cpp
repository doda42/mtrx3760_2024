// example showing string streams
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream

int main () {

  std::stringstream MySensor;

  MySensor << 100 << ' ' << 200;

  int foo, bar;
  MySensor >> foo >> bar;

  std::cout << "foo: " << foo << std::endl;
  std::cout << "bar: " << bar << std::endl;

  std::cout << MySensor.str() << std::endl;
}
