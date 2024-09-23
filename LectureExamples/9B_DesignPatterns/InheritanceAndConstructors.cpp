// This demonstrates how constructors and destructors get called
// with inheritance

#include <iostream>

class CSensor // base class
{
  public:
    CSensor();
    CSensor( int ID );
    virtual ~CSensor();
  protected:
    int mID;
};

class CSonar: public CSensor // derived class
{
  public:
    CSonar( int ID, double Freq );
    CSonar( double Freq );
    ~CSonar();
  private:
    double mFreq;
};


int main()
{
  CSonar Sonar1(1, 43.7);
  CSonar Sonar2(33.3);
  
  std::cout << "Done" << std::endl;
}



//---------------------------------------
CSensor::CSensor()
  : mID(-1) 
{ 
  std::cout << "Default Sensor CTor ID " << mID << std::endl; 
}

//---------------------------------------    
CSensor::CSensor( int ID ) 
  : mID(ID) 
{ 
  std::cout << "Sensor CTor ID " << mID << std::endl; 
}

//---------------------------------------    
CSensor::~CSensor() 
{ 
  std::cout << "Sensor DTor ID " << mID << std::endl; 
}

//---------------------------------------
CSonar::CSonar( int ID, double Freq ) 
  : CSensor( ID ), // explicit call to base class ctor
    mFreq(Freq) 
{
  std::cout << "Sonar CTor Freq " << mFreq << " ID " << mID << std::endl; 
}

//---------------------------------------
CSonar::CSonar( double Freq ) 
  : mFreq(Freq) // implicit call base class ctor
{
  std::cout << "Sonar CTor Freq " << mFreq << " ID " << mID << std::endl; 
}

//---------------------------------------
CSonar::~CSonar() 
{ 
  std::cout << "Sonar DTor Freq " << mFreq << " ID " << mID << std::endl; 
}

