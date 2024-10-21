// Demonstrates another pitfall: accessing out-of-scope variables
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>

void newThreadCallback(int* p)
{
  std::chrono::milliseconds dura( 100 );
  std::this_thread::sleep_for( dura );
  *p = 19; // unsafe... does *p still exist?
}

void startNewThread()
{
  int i = 10;
  std::thread t( newThreadCallback, &i ); //pointer to stack variable i
  t.detach();
}

int main()
{
  startNewThread();
  std::chrono::milliseconds dura( 200 );
  std::this_thread::sleep_for( dura );
}
