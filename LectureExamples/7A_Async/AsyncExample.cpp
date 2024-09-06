// Simple intro to asynchronous programming
// Demonstrates use of std::async and std::future to launch a single task
//
// Compile with g++ AsyncExample.cpp -lpthread

#include <iostream>
#include <future>  // For std::async and std::future
#include <thread>  // For std::this_thread::sleep_for
#include <chrono>  // For std::chrono::seconds

// A simple function that simulates a time-consuming task
int slow_add(int a, int b) {
  std::this_thread::sleep_for(std::chrono::seconds(2));  // Simulate a delay
  return a + b;
}

int main() {
  // Launch slow_add asynchronously
  std::cout << "Starting async task...\n";
  std::future<int> result = std::async(std::launch::async, slow_add, 5, 10);

  // You can do other work here while the async task runs in the background
  std::cout << "Doing some other work while waiting for the result...\n";

  // Wait for the result from the async task (it blocks if not ready)
  int sum = result.get();  // get() will block until slow_add finishes

  // Output the result
  std::cout << "Result from async task: " << sum << std::endl;

  return 0;
}


