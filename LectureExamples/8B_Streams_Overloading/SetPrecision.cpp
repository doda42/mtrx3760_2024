#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

int main()
{
  const long double pi = M_PI;
  std::cout << std::setfill('*')
    << std::setw(15) << "default (6): " << pi << '\n'
    << std::setw(15) << "10: " << std::setprecision(10) << pi << '\n'
    << std::setw(15) << "max: "
    << std::setprecision(std::numeric_limits<long double>::digits10 + 1)
    << pi << '\n';
}
