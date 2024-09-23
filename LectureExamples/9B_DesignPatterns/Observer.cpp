// Demonstrate the Observer design pattern

#include <iostream>
#include <vector>

class Observer
{
  public:
    virtual void Notify() = 0;
};
class Subject
{
  public:
    void RegisterObserver( Observer* pObserver ) {
      mObservers.push_back( pObserver );
    }
    void NotifyObservers() {
      for( Observer* o : mObservers )
        o->Notify();
    }
  private:
    std::vector<Observer*> mObservers;
};

class FireSensor: public Subject
{
  public:
    void DetectedFire() { NotifyObservers(); }
};

class NavRoutine : public Observer
{
  public:
    virtual void Notify() { 
      std::cout << "NavRoutine notified" << std::endl; 
    }
};
class EStop : public Observer
{
  public:
    virtual void Notify() { 
      std::cout << "EStop notified" << std::endl; 
    }
};

int main()
{
  FireSensor MySensor;
  NavRoutine Nav;
  EStop Stop;
 
  MySensor.RegisterObserver( &Nav );
  MySensor.RegisterObserver( &Stop );  
  MySensor.DetectedFire();
}
