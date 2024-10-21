// A simple race condition example, motivating std::atomic
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>
#include <vector>
#include <atomic>

class Counter {
public:
  Counter() : value(0) {}
  int value; 
  
  // replace the above line with the following for a thread-safe version
  // std::atomic<int> value; 
  
  void increment(){
    ++value;
  }
};

void Worker( Counter& aCount )
{
  for(int i = 0; i < 100; ++i)
  {
    aCount.increment();
  }
}

int main(){
  // unfortunately std::atomic<Counter> doesn't work
  // atomic only works on primitive types
  Counter counter;

  std::vector<std::thread> threads;
  for(int i = 0; i < 5; ++i)
  {
    threads.push_back( std::thread( Worker, std::ref(counter) ) );
  }

  for( auto& thread : threads ){
    thread.join();
  }

  std::cout << counter.value << std::endl;
}
