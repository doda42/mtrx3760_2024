// Demonstrates mutex to solve race condition; see RaceCondition.cpp
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>
#include <mutex>

std::mutex MyMutex;

void ThreadWorkerA()
{
  MyMutex.lock();
  std::cout << "Doing work A" << std::endl;
  std::cout << "Thread ID: " << std::this_thread::get_id() << std::endl;
  MyMutex.unlock();
}

void ThreadWorkerB( int Arg )
{
  MyMutex.lock();
  std::cout << "Doing work B : " << Arg << std::endl;
  std::cout << "Thread ID: " << std::this_thread::get_id() << std::endl;
  MyMutex.unlock();  
}


int main()
{
  std::thread ThreadA( ThreadWorkerA ); // create thread by passing pointer to function
  std::thread ThreadB( ThreadWorkerB, 3 ); // same with an argument to the function
  
  MyMutex.lock();
  std::cout << "Main" << std::endl;
  std::cout << "Thread ID: " << std::this_thread::get_id() << std::endl; 
  MyMutex.unlock();  
   
  ThreadA.join(); // waits for ThreadA to exit
  ThreadB.join(); // waits for ThreadB to exit
}
