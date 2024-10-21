// Demonstrate condition_variable
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

std::mutex g_Mutex;
std::condition_variable g_Cond;

void PrintID( int id ) {
  // an unsafe cout for tutorial purposes only
  std::cout << "print waiting for ownership of g_Mutex" << std::endl;
  
  // unique_lock is a lock manager like lock_guard, it works with condition variables
  std::unique_lock<std::mutex> myLock( g_Mutex ); // locks g_Mutex
  
  std::cout << "print waiting for g_Cond" << std::endl;
  g_Cond.wait( myLock ); // releases g_Mutex and waits

  // g_Mutex locked again when the wait function returns
  // so there will be only one thread running at a time
  std::cout << "Thread " << id << std::endl;
  
  //myLock.unlock(); // try manually unlocking the thread here
  std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
  
  // g_Mutex gets unlocked by myLock's destructor
}

void go() {
  // make sure no one's holding g_Mutex locked, i.e. all threads are ready
  std::unique_lock<std::mutex> myLock( g_Mutex ); // locks g_Mutex
  
  g_Cond.notify_all(); // notifies everyone waiting on g_Cond to go
  // try notify_one() instead of notify_all()
}

int main ()
{
  std::thread threads[4];
  
  for (int i=0; i<4; ++i) {
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
