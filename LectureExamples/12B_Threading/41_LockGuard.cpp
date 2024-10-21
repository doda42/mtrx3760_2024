// Demonstrates std::lock_guard to solve unreleased mutex issue; see UnreleasedLocks.cpp
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>
#include <mutex>

std::mutex MyMutex;

void ThreadWorkerA()
{
  std::lock_guard<std::mutex> MyLockGuard( MyMutex );
  // lock_guard constructor locks the mutex
  
  std::cout << "Doing work A" << std::endl;
  std::cout << "Thread ID: " << std::this_thread::get_id() << std::endl;
  // what if your code forgets to call unlock?
  // Or an exception causes the thread to terminate before unlock gets called?
  // no longer a problem, as the lock_guard destructor releases the mutex
   
  // lock_guard destructor unlocks the mutex
}

void ThreadWorkerB( int Arg )
{
  std::lock_guard<std::mutex> MyLockGuard( MyMutex );

  std::cout << "Doing work B : " << Arg << std::endl;
  std::cout << "Thread ID: " << std::this_thread::get_id() << std::endl;
}


int main()
{
  std::thread ThreadA( ThreadWorkerA ); // create thread by passing pointer to function
  std::thread ThreadB( ThreadWorkerB, 3 ); // same with an argument to the function
  
  {
    std::lock_guard<std::mutex> MyLockGuard( MyMutex );
    std::cout << "Main" << std::endl;
    std::cout << "Thread ID: " << std::this_thread::get_id() << std::endl; 
  }
 
  std::cout << "Waiting for B to complete..." << std::endl;
  ThreadB.join(); // waits for ThreadB to exit
 
  std::cout << "Waiting for A to complete..." << std::endl;
  ThreadA.join(); // waits for ThreadA to exit
  
  std::cout << "Done" << std::endl;
}
