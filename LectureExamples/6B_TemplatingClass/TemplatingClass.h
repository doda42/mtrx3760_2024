#ifndef _TEMPLATINGCLASS_H
#define _TEMPLATINGCLASS_H

#include <iostream>

template <class T>
class Pair
{
  public:
    Pair( T First, T Second );
    void Report() const;

  private:
    T mFirst;
    T mSecond;
};

// Note that member function implementations need to stay in the 
// header file for templated classes
template<class T>
Pair<T>::Pair( T First, T Second )
  : mFirst( First ), mSecond( Second )
{ 
}

template<class T>
void Pair<T>::Report() const
{ 
  std::cout << mFirst << ", " << mSecond << std::endl; 
}

#endif
