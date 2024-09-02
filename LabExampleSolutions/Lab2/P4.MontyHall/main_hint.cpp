// Technical guidance only
// not an example of good coding style
#include <iostream>
#include <string>


static const int NUM_DOORS = 3;

//--Forward Declarations--
class CDoors;

//--
class CPerson
{
    public:
        void Init( CDoors* pDoors ){ mpDoors = pDoors; }
    protected:
        std::string mName;
        CDoors* mpDoors;
};

class CHost: public CPerson
{
    public:
        void OpenEmpty();
};

class CPlayer: public CPerson
{
    public:
        int ChooseFirstDoor();
        bool ChooseToChange();
};

class CDoors
{
    public:
        CDoors( int NumDoors = NUM_DOORS );
        ~CDoors();
        void Init( int NumDoors );
        
        void AddRandomCar();
        bool OpenDoor( int Idx );
        bool SelectDoor( int Idx );
        void ChangeSelection();

        bool DoorHasCar( int Idx );
        bool DoorIsOpen( int Idx );
        bool DoorIsSelected( int Idx );
        int GetSelectedIdx();

    private:
        bool* mpHasCar;
        bool* mpOpen;
        bool* mpSelected;
        int mNumDoors;
};

class CGame
{
    public:
        CGame( CPlayer* pPlayer );
        ~CGame();

        bool Run();

    private:
        CDoors* mpDoors;
        CPlayer* mpPlayer;
        CHost* mpHost;
};

int main()
{      
    CPlayer Bob;
    CGame MyGame( &Bob );

    int WinCount = 0;
    int RunCount = 0;
    for( int i=0; i<10000; ++i )
    {    
        bool Result = MyGame.Run();
        if( Result )
            ++WinCount;
        ++RunCount;            
    }
    std::cout << "Win rate: " << double(WinCount) / double(RunCount) << std::endl;
}



