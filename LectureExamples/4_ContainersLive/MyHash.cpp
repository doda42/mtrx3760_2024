// In today's lecture we live-coded a simple hash table to understand how these work.
// We explored how inserting and retrieving items from the hash table is very fast, and
// independent of how many items are in the table.
//
// We then explored two other data structures that could be used for associating names and numbers:
//
// The first of these is what most people think of first, a vector 
// When using a vector it's best to use a struct to tie together the keys and values, e.g. names and numbers
// The vector approach has advantages and disadvantages; it is easy to understand but slow to insert and retrieve
//
// The third option we explored was a sorted vector. Like an alphabetically sorted directory, you can
// perform a fast kind of search called a binary search to find keys more quickly than for an unsorted vector.
// These do however require a bit more time to either build (sorting as you go), or sort once (once it's built).
//
// Right now the hash table implementation is complete but the other two are not.
// It would be a good exercise to complete the other two, and measure how much time each one takes to
// a) populate a directory, and b) look up a name
//
// Code by Don Dansereau and the 2024 MTRX3760 class

#include <iostream>
#include <cstring>

//---Consts---
static const int MaxNameLen = 16;

//---Structs---
struct EntryStruct
{
  char Name[MaxNameLen];
  int Number;
};


//---Base class for different kinds of directory---
class CDirectoryBase
{
  public:
    virtual void AddPerson( const char* pName, int Number ) = 0;
    virtual int GetNumber( const char* pName ) = 0;
};


//---Sorted vector-based directory---
class CDirectorySortedVec: public CDirectoryBase
{
  public:
    CDirectorySortedVec() : mNumEntries(0) {}
    void AddPerson( const char* pName, int Number )
    {
      EntryStruct NewEntry;
      strcpy( NewEntry.Name, pName );
      NewEntry.Number = Number;
      
      // find where in the list to add the person  ---> a bit slower
  
      Entries[mNumEntries] = NewEntry;
      ++mNumEntries;
    }
    int GetNumber( const char* pName )
    {
      // fast, binary search
      return 0;
    }
 
  private:
    static const int MaxEntries = 16;
    EntryStruct Entries[MaxEntries]; 
    int mNumEntries;
};


//---Vector-based directory---
class CDirectoryVec: public CDirectoryBase
{
  public:
    CDirectoryVec() : mNumEntries(0) {}
    void AddPerson( const char* pName, int Number )
    {
      EntryStruct NewEntry;
      strcpy( NewEntry.Name, pName );
      NewEntry.Number = Number;
      
      Entries[mNumEntries] = NewEntry;
      ++mNumEntries;
    }
    int GetNumber( const char* pName )
    {
      for( int i=0; i<mNumEntries; ++i )   // searching through on average 1/2 phone book; worst-case: whole book
        if( !strcmp( Entries[i].Name, pName ) )
          // found it!;
          ;  
      return 0;
    }
 
  private:
    static const int MaxEntries = 16;
    EntryStruct Entries[MaxEntries]; 
    int mNumEntries;
};
    
    
// associates names with numbers
class CDirectoryHash: public CDirectoryBase
{
  public:
    CDirectoryHash();
    void AddPerson( const char* pName, int Number );
    int GetNumber( const char* pName );
    
  private:
    //---Consts---
    static const int NumBuckets = 32; // how many buckets in our hash table
    
    //---Helper functions---
    int HashName( const char* pName );

    //---Data---
    int mBuckets[NumBuckets];

};

//---
int main()
{
  CDirectoryHash MyDirectory;
  
  MyDirectory.AddPerson( "Bob", 123456 );
  MyDirectory.AddPerson( "Mary", 88877 );
  MyDirectory.AddPerson( "Lary", 88877 );    
  MyDirectory.AddPerson( "Barbraanne", 22223 );    
  MyDirectory.AddPerson( "Bob1", 33 );
  MyDirectory.AddPerson( "Bob2", 33 );
  MyDirectory.AddPerson( "Bob3", 33 );      
  MyDirectory.AddPerson( "Bob4", 33 );
  MyDirectory.AddPerson( "asdfasdf", 33 );                
  MyDirectory.AddPerson( "asdfasdasdff", 33 );       

  MyDirectory.GetNumber( "Bob" );
  MyDirectory.GetNumber( "Mary" );  
  MyDirectory.GetNumber( "Barbraanne" );
  
  MyDirectory.GetNumber( "Ollie" );      
   

  return 0;
}

//---
CDirectoryHash::CDirectoryHash()
{
  for( int i=0; i<NumBuckets; ++i )
    mBuckets[i] = -1;
}

//---
void CDirectoryHash::AddPerson( const char* pName, int Number )
{
  // Hash the name -> index into buckets
  int BucketIndex = HashName( pName );
  std::cout << "Adding " << pName << " to bucket " << BucketIndex << std::endl;
  
  // Store the number at that index
  mBuckets[ BucketIndex ] = Number;
}

//---
int CDirectoryHash::GetNumber( const char* pName )
{
  int Result = 0;
  // Hash the name -> index into buckets
  int BucketIndex = HashName( pName );
  std::cout << "Retrieving " << pName << " at bucket " << BucketIndex << std::endl;
  
  Result = mBuckets[ BucketIndex ];
  
  // Retrieve and return the number at that index
  std::cout << "Found number " << Result << " for name " << pName << std::endl;
  return Result;
}


//---
int CDirectoryHash::HashName( const char* pName )
{
  int Result = 0;
  
  for( int iIdx = 0; pName[iIdx] != 0; ++iIdx )
  {
    Result = Result ^ pName[iIdx]; // xor bitwise
  }
  Result = Result % NumBuckets; // modulo to bring into range
  
  return Result;
}

