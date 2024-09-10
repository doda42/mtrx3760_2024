// istream::ignore example
#include <iostream>     // std::cin, std::cout

int main () {
  char first, last;

  std::cout << "Please, enter your first name followed by your surname: ";

  first = std::cin.get();     // get one character
  std::cin.ignore(256,' ');   // ignore until space

  last = std::cin.get();      // get one character

  std::cout << "Your initials are " << first << last << '\n';

  return 0;
}
