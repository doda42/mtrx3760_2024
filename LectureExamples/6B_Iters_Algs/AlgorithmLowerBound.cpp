// Demonstrate std::lower_bound, a binary search that returns an iterator
// pointing to the object if it's found
#include <iostream>
#include <algorithm>
#include <list>
 
int main()
{
  std::list<int> haystack {1, 3, 4, 5, 9}; // binary search requires sorted input!
  std::list<int> needles {1, 2, 3};

  for( auto needle : needles ) {
    std::cout << "Searching for " << needle << std::endl;

    std::list<int>::iterator result =
       std::lower_bound( haystack.begin(), haystack.end(), needle);
    if( result != haystack.end() && *result == needle ) {
      std::cout << "Found " << *result << std::endl;
      ++(*result);
    }
    else
    {
      std::cout << "no dice!\n";
    }
  }
  
  for( int val : haystack )
    std::cout << val << ' ';
  std::cout << std::endl;
}
