// Simple intro to asynchronous programming
// Demonstrates use of std::async, std::shard_future to launch multiple consumers
//
// Compile with g++ AsyncExampleShardFuture.cpp -lpthread
#include <iostream>
#include <future>
#include <chrono>

// A function that simulates a time-consuming task
int slow_add(int a, int b) {
  std::this_thread::sleep_for(std::chrono::seconds(2));  // Simulate a delay
  return a + b;
}

// A function to consume the result of the shared future
void consume_result(std::shared_future<int> shared_fut, const std::string& consumer_name) {
  std::cout << consumer_name << " waiting for result...\n";
  int result = shared_fut.get();  // Will block until the result is ready
  std::cout << consumer_name << " got result: " << result << std::endl;
}

int main() {
  // Launch slow_add asynchronously
  std::cout << "Starting async task...\n";
  std::future<int> fut = std::async(std::launch::async, slow_add, 5, 7);

  // Convert std::future to std::shared_future
  // Note there's no way to create it as shared, it must be created as a future then converted
  std::shared_future<int> shared_fut = fut.share();

  // First consumer using std::async
  auto consumer1 = std::async(std::launch::async, consume_result, shared_fut, "Consumer 1");

  // Second consumer, in a more realistic scenario this could be a different function
  auto consumer2 = std::async(std::launch::async, consume_result, shared_fut, "Consumer 2");

  // Wait for both consumers to finish
  consumer1.get();
  consumer2.get();

  return 0;
}
