#include <iostream>

class ICallable
{
  public:
    virtual void Callback( int aParam ) = 0;
};

class CCameraUser: public ICallable
{
  public:
    void Callback( int aParam) {  
      std::cout << "Callback got called, param: " << aParam << std::endl;
    }
};

class CCamera
{
  public:
    CCamera( ICallable* apCallback ) : mpNotify( apCallback ) {}
    
    void NewFrame() {
      // ...
      mpNotify->Callback( 3 );
    }

  private:
    ICallable* mpNotify;
};

int main()
{
  // the class that receives notifications
  CCameraUser myCameraUser;

  // Register a callback with the camera class
  CCamera myCamera( &myCameraUser );
  
  // simulate arrival of a new frame
  myCamera.NewFrame();
}




