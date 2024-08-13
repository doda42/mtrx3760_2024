#include <iostream>
#include <vector>

class ICallable
{
  public:
    virtual void Callback( int aParam ) = 0;
};

class CCameraUser: public ICallable
{
  public:
    void Callback( int aParam) {  
      std::cout << "CameraUser CB, param: " << aParam << std::endl;
    }
};

class CLogger: public ICallable
{
  public:
    void Callback( int aParam) {  
      std::cout << "CLogger CB, param: " << aParam << std::endl;
    }  
};

class CCamera
{
  public:
    void RegisterCallback( ICallable* pCallback )
    {
      mNotify.push_back( pCallback );
    }
    void ClearCallbacks();
    void NewFrame() {
      // ...
      for( ICallable* pCallable : mNotify )
        pCallable->Callback( 3 );
    }

  private:
    std::vector<ICallable*> mNotify;
};

int main()
{
  // the class that receives notifications
  CCameraUser myCameraUser;
  CLogger myLogger;

  // Register a callback with the camera class
  CCamera myCamera;
  
  myCamera.RegisterCallback( &myCameraUser );
  myCamera.RegisterCallback( &myLogger );
  
  // simulate arrival of a new frame
  myCamera.NewFrame();
}




