// IncrementVariants.cpp
// A few examples of approaches to debug checks in a very simple function
// Note that the trivial solution here is to use a reference
// But this will not always be possible, so it's important to understand
// the options in dealing with pointers safely.
// Try compiling with and without NDEBUG defined

#include <iostream>
#include <string>
#include <cassert>  // the built-in assert is ignored if we #define NDEBUG

//---Types---
enum eResult { RESULT_OK=0, RESULT_ERROR };

//---Functions---

// no pointers, no problems
void IncrementCorrect( int& rInt )
{
  ++rInt;
}

// quiet failure, not good
void IncrementSilentFail( int* pInt )
{
  if( pInt != NULL )
  {
    ++(*pInt);
  }
}

// runtime check, needs to be called correctly
eResult IncrementReturnResult( int* pInt )
{
  eResult Result = RESULT_ERROR;
  if( pInt != NULL )
  {
    ++(*pInt);
    Result = RESULT_OK;
  }
  return Result;
}

// assert, bulletproof in debug, 
// beware release-only bugs!
void IncrementAsserts( int* pInt )
{
  assert( pInt != NULL );
  ++(*pInt);
}

// Mix: has checks in both debug and release
eResult IncrementAssertAndReturn( int* pInt )
{
  eResult Result = RESULT_ERROR;
  assert( pInt != NULL );
  
  if( pInt != NULL )
  {
    ++(*pInt);
    Result = RESULT_OK;
  }
  return Result;
}


int main()
{
  int A = 0;
  IncrementCorrect( A ); // avoid the problem
  
  IncrementSilentFail( NULL ); // worst: silently fail
  
  eResult Status = IncrementReturnResult( NULL );
  if( Status != RESULT_OK )
    std::cout << "Hit a problem" << std::endl;

  IncrementAsserts( &A );
  
  eResult Status2 = IncrementAssertAndReturn( NULL );
  if( Status2 != RESULT_OK )
    std::cout << "Hit a problem" << std::endl;

}

