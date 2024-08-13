#include <iostream>


typedef void (*t_callback)( int );

void Callback1( int aParam )
{
  std::cout << "Callback1 got called, param: " << aParam << std::endl;
}

class CCamera
{
  public:
    CCamera( t_callback apCallback ) : mpNotify( apCallback ) {}
    
    void NewFrame() {
      // ...
      mpNotify( 3 );
    }

  private:
    t_callback mpNotify;
};

int main()
{
	// demonstrate pointer-to-function
	t_callback p = Callback1;
	p(42);

	// Register a callback with the camera class
  CCamera myCamera( Callback1 );

  // simulate arrival of a new frame
	myCamera.NewFrame();
}
