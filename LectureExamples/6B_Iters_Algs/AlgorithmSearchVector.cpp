// Demonstrate std::search
#include <iostream>
#include <algorithm>
#include <vector>
 
int main()
{
  std::vector<int> haystack {1, 3, 4, 5, 9}; // binary search requires sorted input!
  std::vector<int> needles {1, 2, 3};

  for( auto needle : needles ) {
    std::cout << "Searching for " << needle << std::endl;
    
    if( std::binary_search( haystack.begin(), haystack.end(), needle) ) {
      std::cout << "Found " << needle << std::endl;
    }
    else
    {
      std::cout << "no dice!\n";
    }
    
  }
}
