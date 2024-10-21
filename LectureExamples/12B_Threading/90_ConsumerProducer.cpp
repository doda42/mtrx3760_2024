// A more complex example building up a consumer/producer using a circular buffer
// The circular buffer is cyclic and of fixed size; it uses condition variables to
// mediate fetching and depositing of new entries.
// Consumer and producer threads simulate the case of having sensor data produced
// and processed by multiple independent threads.  Thread safety is ensured entirely
// by the thread-safe CircularBuffer class.
//
// Compile with the pthread library, i.e.:
// g++ <fname> -lpthread

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

//--The circular buffer data structure--
class CircularBuffer {
public:
  CircularBuffer( int capacity );
  ~CircularBuffer();

  void deposit( int data );
  int fetch();

private:
  int* buffer;
  int capacity;

  int front;
  int rear;
  int count;

  std::mutex lock;
  std::condition_variable not_full;
  std::condition_variable not_empty;
};

//--A producer thread that adds contents to the buffer--
void producer( int id, CircularBuffer& buffer ){
  for( int i = 0; i < 45; ++i )
  {
    buffer.deposit( id + i );
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
}

//--A consumer thread that takes content off the buffer--
void consumer(int id, CircularBuffer& buffer){
  for( int i = 0; i < 30; ++i )
  {
    int value = buffer.fetch();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}

//---
int main(){
  CircularBuffer buffer( 20 );

  // make three concurrent consumer threads
  std::thread c1(consumer, 0, std::ref(buffer));
  std::thread c2(consumer, 1, std::ref(buffer));
  std::thread c3(consumer, 2, std::ref(buffer));
  
  // make two concurrent producer threads
  std::thread p1(producer, 1000, std::ref(buffer));
  std::thread p2(producer, 2000, std::ref(buffer));

  p1.join();
  p2.join();
  c1.join();
  c2.join();
  c3.join();
}

//--CircularBuffer implementation---
CircularBuffer::CircularBuffer( int capacity ) 
  : capacity(capacity), front(0), rear(0), count(0) 
{
  buffer = new int[capacity];
}

CircularBuffer::~CircularBuffer()
{
  delete[] buffer;
}

//----
void CircularBuffer::deposit( int data )
{
  std::unique_lock<std::mutex> l(lock);
  not_full.wait(l, [this](){return count != capacity; });
  
  buffer[rear] = data;
  rear = (rear + 1) % capacity;
  ++count;

  std::cout << "Deposited\t" << data << " count\t" << count << std::endl;
    
  not_empty.notify_one();
}

//----
int CircularBuffer::fetch()
{
  std::unique_lock<std::mutex> l(lock);
  not_empty.wait(l, [this](){return count != 0; });

  int result = buffer[front];
  front = (front + 1) % capacity;
  --count;

  std::cout << "Fetched  \t" << result << " count\t" << count << std::endl;

  not_full.notify_one();

  return result;
}
