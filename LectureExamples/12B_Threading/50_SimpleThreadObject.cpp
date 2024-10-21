// Demonstrate threading with objects: calls a member function as a set of threads
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

class Odo
{
public:
  Odo() : mCounts(0) {}
  int getCounts() { return mCounts; }
  void addCounts( int counts ) {
    std::lock_guard<std::mutex> lockGuard(mMutex); // try this example without the mutex
    for( int i = 0; i < counts; ++i )
      ++mCounts;
  }

private:
  int mCounts;
  std::mutex mMutex;
};

int main()
{
  Odo odoObject;
  std::vector<std::thread> threads;
  
  for(int i = 0; i < 5; ++i) {
    // note thread constructor syntax: std::thread(pFunc, pObj, <args>)
    threads.push_back( std::thread( &Odo::addCounts, &odoObject, 1000 ) );
  }

  // wait for all threads to complete
  for( int i = 0; i < threads.size(); ++i)
    threads[i].join();

  std::cout << odoObject.getCounts() << std::endl;
}

