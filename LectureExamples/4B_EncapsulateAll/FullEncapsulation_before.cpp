#include <vector>


typedef long int CarDistType;

struct WheelStruct;

const int NumWheels = 4;

class CCar
{
	WheelStruct* pWheels[ NumWheels ];
	CarDistType HowFar;
};

void SortCars( std::vector<CCar*> pCar );



int main()
{
	CCar myCar;
}


class CCar
{
	typedef long int CarDistType;

	struct WheelStruct;

	static const int NumWheels = 4;

	WheelStruct* pWheels[ CCar::NumWheels ];
	CarDistType HowFar;

	static void SortCars( std::vector<CCar*> pCar );

};



int main()
{
	CCar myCar;
}
