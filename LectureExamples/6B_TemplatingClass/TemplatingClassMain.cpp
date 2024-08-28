#include <iostream>
#include "TemplatingClass.h"

int main()
{
  Pair<int> PairA( 3, 5 );
  PairA.Report();
  
  Pair<double> PairB( 3.14, 2.72 );
  PairB.Report();

  return 0;
}
