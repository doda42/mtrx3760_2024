#include <atomic> 
#include <iostream>

namespace Normal
{
  class Counter
  {
    public:
      Counter() :_count(0) { }
      void Increment() { ++_count; }
      int _count;
  };
}

namespace ThreadSafe
{
  class Counter
  {
    public:
      Counter() :_count(0) { }
      void Increment() { ++_count; }
      std::atomic<int> _count;
  };
}


// put everything in the "Normal namespace" into global namespace
using namespace Normal;
//using namespace ThreadSafe;  

// try all four combinations of commenting the above two lines

int main()
{  
  Counter MyCounter;
  MyCounter.Increment();
  std::cout << MyCounter._count << std::endl;
}
