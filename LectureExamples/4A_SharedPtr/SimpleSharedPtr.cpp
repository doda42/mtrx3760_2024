#include <iostream>
#include <memory> // shared_ptr


int main()
{
  std::shared_ptr<int> pInt1 = std::make_shared<int>( 42 );  // replaces "new"
  
  std::cout << "*pInt1: " << *pInt1 << std::endl;
  std::cout << "pInt1.use_count(): " << pInt1.use_count() << std::endl;
  
  {
    std::shared_ptr<int> pInt2 = pInt1;  // adds another "user" of the shared ptr
    std::cout << "*pInt2: " << *pInt2 << std::endl;

    std::cout << "pInt1.use_count(): " << pInt1.use_count() << std::endl;
    std::cout << "pInt2.use_count(): " << pInt2.use_count() << std::endl;

    (*pInt1)++;

    std::cout << "*pInt1: " << *pInt1 << std::endl;
    std::cout << "*pInt2: " << *pInt2 << std::endl;
  }

  // pInt2 got destroyed, so use_count is now 1  
  std::cout << "pInt1.use_count(): " << pInt1.use_count() << std::endl;
}

// when use_count hits 0, object is
// destroyed, replaces "delete"

