#include <iostream>

class CRobot
{
  public:
    //---Structs/classes---
    struct PositionVec {
      double x,y,z;
    };
    struct RotationVec {
      double wx,wy,wz;
    };   
    class CPose {
      public:
        CPose(): mPosition{0,0,0}, mRotation{0,0,0} {} 
        void MoveAlong();
        PositionVec mPosition;
        RotationVec mRotation;
    };
    
    //---Functions---
    void Simulate() { mPose.MoveAlong(); }
    //---Accessors---
    const CPose& GetPose();
  
  private:
    //---Member variables---
    CPose mPose;
};

//---
int main()
{
  CRobot Rosalind;
  std::cout << Rosalind.GetPose().mPosition.x << std::endl;
  Rosalind.Simulate();
  std::cout << Rosalind.GetPose().mPosition.x << std::endl;
}

//---
const CRobot::CPose& CRobot::GetPose()
{
  return mPose;
}

void CRobot::CPose::MoveAlong()
{
  mPosition.x++;
}
