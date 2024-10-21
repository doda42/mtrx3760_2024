// Demonstrates pass-by-reference into a threaded function
//   std::move to pass ownership of a thread
//   std::this_thread::sleep_for for thread-friendly sleep
//   std::ref to pass a reference through the thread constructor
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>
#include <chrono>

void IncInt( int& n )
{
  std::cout << "IncInt executing" << std::endl;
  for (int i = 0; i < 5; ++i) {
    ++n;
    std::this_thread::sleep_for( std::chrono::milliseconds(10) ); // safe delay
  }
}

int main()
{
  int n = 0;
  // std::thread tA( IncInt, n ); // pass by value: compiler error, IncInt wants a ref
  std::thread tA( IncInt, std::ref(n) ); // std::ref tells the compiler to pass a reference
  
  // use std::move to move ownership of the IncInt()
  std::thread tB( std::move(tA) ); // tB is now running IncInt(), tA is no longer a thread
  tB.join(); // wait for thread to finish
  
  std::cout << "Final value of n is " << n << '\n';
}
