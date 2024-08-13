#include <iostream>

void Callback1( int aParam )
{
  std::cout << "Callback1 got called, param: " << aParam << std::endl;
}

class CCamera
{
  public:
    CCamera( void (*apCallback)(int) ) : mpNotify( apCallback ) {}
    
    void NewFrame() {
      // ...
      mpNotify( 3 );
    }

  private:
    void (*mpNotify)(int);
};

int main()
{
  // demonstrate pointer-to-function
  void (*p)(int) = Callback1;
  p(42);
  
  // Register a callback with the camera class
  CCamera myCamera( Callback1 );

  // simulate arrival of a new frame
  myCamera.NewFrame();
}
