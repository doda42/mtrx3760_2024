// Demonstrates some common pitfalls
//   std::thread::joinable()
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>

void tA()
{
  std::cout << "Doing work A" << std::endl;
}

void tB( int Arg )
{
  std::cout << "Doing work B " << Arg << std::endl;
}


int main()
{
  std::thread ThreadA( tA );
  std::thread ThreadB( tB, 3 );
  
  std::cout << "Main" << std::endl;
   
  ThreadA.join();
  //ThreadA.join(); // crash
  
  if( ThreadA.joinable() ) // a safer alternative
    ThreadA.join();
  
  ThreadB.detach();
  //ThreadB.detach(); // crash
  if( ThreadB.joinable() ) // a safer alternative
    ThreadB.detach();
}
