// CtorDtorEtor.cpp
// Messy reality-check only, to make sure the question makes sense
// Students seem to want to use a pointer-to-pointer, here's how that would look

#include <iostream>
#include <string>

//----------------------------------
class Country
{
  public:
    Country( std::string Name, bool OnEquator )
      : _name( Name ), _onEquator( OnEquator )
    { 
      std::cout << "Country CTor: " << Name << std::endl;
    };
    ~Country()
    {
      std::cout << "Country DTor: " << _name << std::endl;
    }

    std::string GetName() { return _name; }
    bool IsOnEquator()
    {
      return _onEquator;
    }
    
  private:
    std::string _name;
    bool _onEquator;
};


//----------------------------------
class Planet
{
  public:
    Planet( std::string Name )
      : _name( Name )
    { 
      std::cout << "Planet CTor: " << Name << std::endl; 
      
      _countries = new Country*[5]; // make the array of pointers
      
      _countries[0] = new Country( "Bobnavia", true );   // initialise each pointer with a new country
      _countries[1] = new Country( "Narnia", false );
      _countries[2] = new Country( "TronWorld", true );
    };
    ~Planet()
    {
      std::cout << "Planet DTor: " << _name << std::endl;
      for( int i=0; i<_numCountries; ++i )
      {
        delete _countries[i];   // delete each country
      }
      delete _countries;      // delete the array of pointers
    }
    void ReportEquatorials()
    {
      std::cout << "Listing equatorial countries" << std::endl;
      for( int i=0; i<_numCountries; ++i )
      {
        if( _countries[i]->IsOnEquator() )
        {
          std::cout << _countries[i]->GetName() << " is on equator" << std::endl;
        }
      }
    }
    
  private:
    std::string _name;
    static const int _numCountries = 3;
    Country** _countries;
};


//----------------------------------
int main()
{
  Planet* pBob = new Planet("PlanetBob");
  pBob->ReportEquatorials();
  delete pBob;
}



