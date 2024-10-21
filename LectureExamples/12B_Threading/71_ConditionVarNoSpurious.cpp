// Demonstrate condition_variable
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

std::mutex g_Mutex;
std::condition_variable g_Cond;
int g_go = 0;

void PrintID( int id ) {
  std::unique_lock<std::mutex> myLock( g_Mutex ); // locks g_Mutex
  // use an additional global flag to avoid "spurious" wakeups
  while( !g_go )
    g_Cond.wait( myLock ); // releases g_Mutex and waits

  // a shorter version of the same:
  // g_Cond.wait( myLock, [](){return g_go; });

  std::cout << "Thread " << id << std::endl;
  std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
}

void go() {
  std::unique_lock<std::mutex> myLock( g_Mutex ); // locks g_Mutex
  g_go = 1;
  g_Cond.notify_all(); // notifies everyone waiting on g_Cond to go
}

int main ()
{
  std::thread threads[4];
  
  for (int i=0; i<4; ++i){
    // this delay is to avoid racing of the unsafe cout in PrintID
    std::this_thread::sleep_for( std::chrono::milliseconds(50) ); 
    threads[i] = std::thread( PrintID, i );
  }

  std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
  std::cout << "threads ready to race...\n";
  go();

  for( auto& th : threads ) 
    th.join();
}
