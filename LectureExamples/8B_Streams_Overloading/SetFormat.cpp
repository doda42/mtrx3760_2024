#include <iostream>
#include <sstream>

int main()
{
  std::cout 
  << "fixed:      " << std::fixed << 0.01 << '\n'
  << "scientific: " << std::scientific << 0.01 << '\n'
  << "hexfloat:   " << std::hexfloat << 0.01 << '\n'
  << "default:    " << std::defaultfloat << 0.01 << '\n';
}
