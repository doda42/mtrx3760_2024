#include <iostream>
#include <string>
#include <sstream>


class CEatStrings
{
  public:
    CEatStrings( conststd::string& aInput ) {
      std::cout << aInput << std::endl;
    }
};

int main()
{
  std::stringstream myStream;
  
  myStream << "Hello, World!" << std::endl;
  
  for( int i=0; i<10; ++i )
    myStream << i << ' ';
  myStream << std::endl;

  CEatStrings myStringEater( myStream.str() ); // .str() returns a string

  std::stringstream anotherStream;
  anotherStream << 1 << " " << 32;
  
  int foo,bar;
  anotherStream >> foo >> bar; // stream out as well as in

  std::cout << "foo: " << foo << '\n';
  std::cout << "bar: " << bar << '\n';
}
