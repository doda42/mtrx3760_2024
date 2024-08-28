// DebugBuilds_Simple.cpp
// Simplified example of asserts and logging in debug / release build targets
// Try building and running with :
//   g++ DebugBuilds_Simple.cpp
//   g++ -DNDEBUG DebugBuilds_Simple.cpp

#include <iostream>
#include <string>
#include <cassert>  // the built-in assert is ignored if we #define NDEBUG

class MyLogger
{
  public:
    static void LogMessage( const std::string& rMessage ) {
    #ifndef NDEBUG
    std::cout << rMessage << std::endl;
    #endif
  }
};

void Increment( int* pInt )
{
  assert( pInt != NULL );
  ++(*pInt);
  MyLogger::LogMessage( "Value: " + std::to_string( *pInt ) );
}

int main()
{
  MyLogger::LogMessage( "Starting" );
  int A = 0;
  Increment( &A );
  Increment( &A );
  Increment( &A );
  int* pA = NULL;
  Increment( pA );  
  MyLogger::LogMessage( "Done" );  
}

