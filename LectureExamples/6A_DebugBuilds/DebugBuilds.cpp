// DebugBuilds.cpp
// Example of asserts and logging in debug / release build targets
// The logger includes a timestamp and the calling function information in log messages
// Try building and running with :
//   g++ -DRELEASE DebugBuilds.cpp
//   g++ DebugBuilds.cpp
//   g++ -DDEBUG DebugBuilds.cpp

#ifdef RELEASE
#define NDEBUG  // needs to appear before #include <cassert>
#endif

#include <iostream>
#include <string>
#include <cassert>  // the built-in assert is ignored if we #define NDEBUG

#ifdef DEBUG
#define MyLog( msg ) {MyLogger::LogMessage( __FILE__, __PRETTY_FUNCTION__, __LINE__, (msg) );}
#else
#define MyLog( msg ) // do nothing
#endif

class MyLogger
{
  public:
    static void LogMessage( const char* pFile, const char* pFunc, int Line, const std::string& rMessage ) {
    clock_t Time = clock();
    std::cout << "[ " << Time << " " << pFile << ":" << Line << ": "
              << pFunc << " ]: " << rMessage << std::endl;
  }
};

void Increment( int* pInt )
{
  assert( pInt != NULL );
  ++(*pInt);
  MyLog( "Value: " + std::to_string( *pInt ) );
}

int main()
{
  MyLog( "Starting" );
  int A = 0;
  Increment( &A );
  Increment( &A );
  Increment( &A );
  int* pA = NULL;
  Increment( pA );  
  MyLog( "Done" );  
}

