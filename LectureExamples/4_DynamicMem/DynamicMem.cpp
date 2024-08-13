// An example of dynamic memory allocation using new/delete, 
// and polymorphism, using std::vector to store pointers

#include <iostream>
#include <vector>

//-----------------------
class CMathOp
{
    public:
        virtual void Go() = 0; // abstract base class
        virtual ~CMathOp() { std::cout << "Dtor" << std::endl; }
};
//-----------------------
class CMathAdd: public CMathOp
{
    void Go() { std::cout << "add "; }
};
//-----------------------
class CMathSub: public CMathOp
{
    void Go() { std::cout << "sub "; }
};

//-----------------------
int main()
{
    std::vector<CMathOp*> pMathOps;
    
    int WhichOp;
    do
    {
        std::cin >> WhichOp;
        CMathOp* pNewOp = NULL;
        switch( WhichOp )
        {
            case 1:
                pNewOp = new CMathAdd;
            break;
            case 2:
                pNewOp = new CMathSub;
            break;            
        }
        
        // if the user requested a valid math op, enter it into the array
        if( pNewOp != NULL )
            pMathOps.push_back( pNewOp );
            
    } while( WhichOp != 0 );
    
    std::cout << "Registered " << pMathOps.size() << " operations" << std::endl;

    //---Exercise the array---
    for( CMathOp* n : pMathOps )
    {
        n->Go();
    }
    std::cout << std::endl;
    
    //---Cleanup---
    for( CMathOp* n : pMathOps )
    {
        delete n;
    }
}
