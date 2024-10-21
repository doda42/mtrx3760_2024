// Demonstrates the concept behind how a mutex works
// Warning: doesn't work, there are race conditions in this mutex implementation
// ... this is just to explain the core functionality
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>

class CMutex
{
  public:
    CMutex() : Active(0) {}
    void lock() { 
      while( Active > 0 )
        ; // wait
      ++Active; 
    }
    void unlock() {
      --Active;
    }
  private:
    int Active;
};

CMutex MyMutex;

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
