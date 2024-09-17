// An example exploring different forms of association and composition
// Note that syntax alone doesn't tell you the answer
//
// A pointer can point to an object that is only associated with the class
// A pointer can also point to an object that is owned by the class
// The difference is structural, not syntactic
//
// The example illustrates this by example of passengers that can enter and leave a car,
// while with wheels and an engine it's safe to assume they are always with, i.e. "owned" by the car.
// This is simplified as it excludes the possibility of removing wheels and engines, e.g. for repair.
// We assume a car always has the same wheels and engine.

#include <iostream>

class CEngine
{
  public:
    CEngine() { std::cout << "CEngine Ctor" << std::endl; }
    ~CEngine() { std::cout << "CEngine Dtor" << std::endl; }
};

class CWheel
{
  public:
    CWheel() { std::cout << "CWheel Ctor" << std::endl; }
    ~CWheel() { std::cout << "CWheel Dtor" << std::endl; }
};

class CPerson
{
  public:
    CPerson() { std::cout << "CPerson Ctor" << std::endl; }
    ~CPerson() { std::cout << "CPerson Dtor" << std::endl; }
};

// Examples of composition using direct instantiation
class CCar_Direct
{
  public:
    CCar_Direct() { std::cout << "CCar_Direct Ctor" << std::endl; }
    ~CCar_Direct() { std::cout << "CCar_Direct Dtor" << std::endl; }
    
    void AddPassenger( CPerson* pPassenger ) { 
      mpPassenger = pPassenger; 
      std::cout << "Passenger onboard" << std::endl; 
    }    
    void RemovePassenger( ) { 
      mpPassenger = NULL;               // it would make no sense to delete mpPassenger, why?
      std::cout << "Passenger disembarked" << std::endl;  
    }
    
    void DriveToDestination() { std::cout << "Driving to destination" << std::endl; }

  private:
    CWheel mWheels[4];      // Composition: CCar_Direct owns wheels
    CEngine mEngine;        // Composition: CCar_Direct owns engine
    CPerson* mpPassenger;   // Association: CCar_Direct temporarily takes on a passenger, who can leave
};

// Examples of composition using pointers
class CCar_Ptrs
{
  public:
    CCar_Ptrs() {
        std::cout << "CCar_Ptrs Ctor" << std::endl;
        
        for( int i=0; i<4; ++i )
            mpWheels[i] = new CWheel;   // wheels are created in the Ctor, the car owns the wheels
            
        mpEngine = new CEngine;         // engine is created in the Ctor, the car owns the engine
    }
    ~CCar_Ptrs() {
        std::cout << "CCar_Ptrs Dtor" << std::endl;
            
        for( int i=0; i<4; ++i )
            delete mpWheels[i];         // wheels only ever deleted in Dtor, car always has wheels
            
        delete mpEngine;                // engine only ever deleted in Dtor, car always has an engine
    }

    void AddPassenger( CPerson* pPassenger ) { 
      mpPassenger = pPassenger; 
      std::cout << "Passenger onboard" << std::endl; 
    }    
    void RemovePassenger( ) { 
      mpPassenger = NULL;               // it would make no sense to delete mpPassenger, why?
      std::cout << "Passenger disembarked" << std::endl;  
    }
    
    void DriveToDestination() { std::cout << "Driving to destination" << std::endl; }

  private:
    CWheel* mpWheels[4];    // Composition: CCar_Ptrs owns wheels
    CEngine* mpEngine;      // Composition: CCar_Ptrs owns engine
    CPerson* mpPassenger;   // Association: CCar_Ptrs temporarily takes on a passenger, who can leave
                            // It would make no sense to use direct instantiation (no pointer) here, why?
};



int main()
{
    CCar_Direct myDirectCar;
    CCar_Ptrs myPtrCar;
    std::cout << "Two cars constructed, neither has passengers" << std::endl << std::endl;

    CPerson Bob;
    CPerson Mary;
    std::cout << "Two people constructed, both are looking for a ride" << std::endl << std::endl;
    
    myDirectCar.AddPassenger( &Bob );
    myDirectCar.DriveToDestination( );
    myDirectCar.RemovePassenger( );
    std::cout << "Direct car gave Bob a ride" << std::endl << std::endl;
     
    myDirectCar.AddPassenger( &Mary );
    myPtrCar.AddPassenger( &Bob );

    myDirectCar.DriveToDestination( );
    myPtrCar.DriveToDestination( );

    myDirectCar.RemovePassenger( );
    myPtrCar.RemovePassenger( );
    
    std::cout << "Direct car and Ptr car gave rides to Mary and Bob, respectively" << std::endl << std::endl;

    return 0;
}

