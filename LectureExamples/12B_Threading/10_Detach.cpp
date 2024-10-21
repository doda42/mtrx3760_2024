// Demonstrates std::detach
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>
#include <chrono>

void Worker()
{
  for( int i=0; i<10; ++i ) {
    std::cout << "Worker iteration " << i << std::endl;
    std::this_thread::sleep_for( std::chrono::milliseconds(1) ); // safe delay
  }
  std::cout << "---Thread done---" << std::endl;
}

void ThreadLauncher()
{
  std::thread tA( Worker ); 
  std::this_thread::sleep_for( std::chrono::milliseconds(3) ); // safe delay

  // if we don't detach then tA's destructor terminates the program
  tA.detach(); // instead, the thread keeps running...
}

int main()
{
  std::cout << "---Start---" << std::endl;
  ThreadLauncher();    
  std::cout << "---Launched, fixed sleep---" << std::endl;
  std::this_thread::sleep_for( std::chrono::milliseconds(100) ); // safe delay
  std::cout << "---Program Done---" << std::endl;
}
