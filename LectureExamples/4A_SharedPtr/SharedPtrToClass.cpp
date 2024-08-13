#include <iostream>
#include <memory> // shared_ptr

class CWheel
{
  public:
    CWheel() {}
    CWheel( int aTurns ) : mTurns(aTurns) { std::cout << "CTor" << std::endl; }
    ~CWheel() { std::cout << "DTor" << std::endl; }
    int mTurns;
};

int main()
{
  std::shared_ptr<CWheel> pWheel1 = std::make_shared<CWheel>( 42 );  // replaces "new"
  
  std::cout << "pWheel1->mTurns " << pWheel1->mTurns << std::endl;
  std::cout << "pWheel1.use_count(): " << pWheel1.use_count() << std::endl;
  
  {
    std::shared_ptr<CWheel> pWheel2 = pWheel1;  // adds another "user" of the shared ptr
    std::cout << "pWheel2->mTurns: " << pWheel2->mTurns << std::endl;

    std::cout << "pWheel1.use_count(): " << pWheel1.use_count() << std::endl;
    std::cout << "pWheel2.use_count(): " << pWheel2.use_count() << std::endl;
  }

  // pWheel2 got destroyed, so use_count is now 1  
  std::cout << "pWheel1.use_count(): " << pWheel1.use_count() << std::endl;
}

// when use_count hits 0, object is
// destroyed, replaces "delete"

