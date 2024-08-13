#include <iostream>

class CRobot
{
  public:
    //---Structs---
    struct PositionVec {
      double x,y,z;
    };
    struct RotationVec {
      double wx,wy,wz;
    };   
    struct PoseStruct {
      PositionVec mPosition;
      RotationVec mRotation;
    };
    
    //---Accessors---
    const PoseStruct& GetPose();
  
  private:
    //---Member variables---
    PoseStruct mPose;
};

//---
int main()
{
  CRobot Rosalind;
  CRobot::PoseStruct myPose = Rosalind.GetPose();
  std::cout << myPose.mPosition.x << std::endl;
}

//---
const CRobot::PoseStruct& CRobot::GetPose()
{
  return mPose;
}
