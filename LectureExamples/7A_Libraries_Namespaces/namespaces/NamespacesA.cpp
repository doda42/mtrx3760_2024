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

int main()
{
  Normal::Counter MyCounter;
  MyCounter.Increment();
  std::cout << MyCounter._count << std::endl;
  
  ThreadSafe::Counter MyCounter2;
  MyCounter2.Increment();
  std::cout << MyCounter2._count << std::endl;
}
