// Demonstrate simple threading
// std::thread, std::join
// You may notice a race condition
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>

void ThreadWorkerA()
{
  std::cout << "Doing work A" << std::endl;
}

void ThreadWorkerB( int Arg )
{
  std::cout << "Doing work B " << Arg << std::endl;
}


int main()
{
  std::thread ThreadA( ThreadWorkerA ); // create thread by passing pointer to function
  std::thread ThreadB( ThreadWorkerB, 3 ); // same with an argument to the function
  
  std::cout << "Main" << std::endl;
   
  ThreadA.join(); // waits for ThreadA to exit
  ThreadB.join(); // waits for ThreadB to exit
}
