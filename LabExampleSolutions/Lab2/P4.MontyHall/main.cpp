// Technical guidance only
// not an example of good coding style
#include <iostream>
#include <string>


const int NUM_DOORS = 52;

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
        CDoors( int NumDoors = NUM_DOORS )
        : mNumDoors( NumDoors ), mpHasCar( 0 ), mpOpen( 0 ), mpSelected( 0 )
        { 
            mpHasCar = new bool[NumDoors];
            mpOpen = new bool[NumDoors];
            mpSelected = new bool[NumDoors];
        }
        ~CDoors()
        {
            delete [] mpHasCar;
            delete [] mpOpen;
            delete [] mpSelected;
        }
        void Init( int NumDoors )
        { 
            for( int i=0; i<NumDoors; ++i )
            {
                mpHasCar[i] = 0;
                mpSelected[i] = 0;
                mpOpen[i] = 0;
            }
            AddRandomCar();
        }
        void AddRandomCar()
        {
            int Idx = rand() % mNumDoors;
            mpHasCar[Idx] = true;
            std::cout << "Adding car behind door " << Idx << std::endl;            
        }
        bool DoorHasCar( int Idx )
        {
            return mpHasCar[ Idx ];
        }
        bool DoorIsOpen( int Idx )
        {
            return mpOpen[ Idx ];
        }
        void OpenDoor( int Idx )
        {
            std::cout << "Opening door " << Idx << std::endl;
            mpOpen[ Idx ] = true;
        }
        void SelectDoor( int Idx )
        {
            std::cout << "Selecting door " << Idx << std::endl;
            mpSelected[ Idx ] = true;
        }
        bool DoorIsSelected( int Idx )
        {
            return mpSelected[ Idx ];
        }
        void ChangeSelection()
        {
            // at this point every door should either be open or selected except one
            int NewSelection;
            int CurrentlySelected = 0;
            for( int i=0; i<NUM_DOORS; ++i )
            {
                if( mpSelected[i] )
                    CurrentlySelected = i;
                else if( !mpOpen[i] )
                    NewSelection = i;
            }
            mpSelected[CurrentlySelected] = false;
            mpSelected[NewSelection] = true;
        }
        int GetSelectedIdx()
        {
            int Result;
            for( Result = 0; Result < NUM_DOORS; ++Result )
            {
                if( mpSelected[ Result ] )
                    break;
            }
            return Result;
        }

    private:
        int mNumDoors;
        bool* mpHasCar;
        bool* mpOpen;
        bool* mpSelected;
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


CGame::CGame( CPlayer* pPLayer )
: mpPlayer( pPLayer )
{
    mpDoors = new CDoors;
    mpHost = new CHost;
    mpHost->Init( mpDoors );
    mpPlayer->Init( mpDoors );
}


CGame::~CGame()
{
    delete mpDoors;
    delete mpHost;
}

bool CGame::Run()
{
    bool Result = false;

    mpDoors->Init( NUM_DOORS );

    mpPlayer->ChooseFirstDoor();
    mpHost->OpenEmpty();

    bool ChangeSelection = mpPlayer->ChooseToChange();
    if( ChangeSelection) 
        std::cout << "Player chooses to change doors" << std::endl;
    else
        std::cout << "Player chooses not to change doors" << std::endl;    

    if( ChangeSelection )
        mpDoors->ChangeSelection();

    int FinalSelection = mpDoors->GetSelectedIdx();
    std::cout << "Player's final selection: " << FinalSelection << std::endl;

    Result = mpDoors->DoorHasCar( FinalSelection );
    if( Result )
        std::cout << "Player wins!" << std::endl;
    else
        std::cout << "Player loses" << std::endl;

    return Result;
}

void CHost::OpenEmpty()
{
    int DoorsOpened = 0;
    for( int i=0; i<NUM_DOORS; ++i )
    {
        if( !mpDoors->DoorIsSelected(i)  &&  !mpDoors->DoorHasCar(i) )
        {
            mpDoors->OpenDoor( i );
            ++DoorsOpened;
        }
        if( DoorsOpened >= NUM_DOORS-2 )
            break;
    }
    if( DoorsOpened != NUM_DOORS-2)
    {
        std::cout << "ISSUE!" << std::endl;
    }
}

int CPlayer::ChooseFirstDoor()
{
    int WhichToChoose = rand() % NUM_DOORS;
    mpDoors->SelectDoor( WhichToChoose );
    return WhichToChoose;
}
bool CPlayer::ChooseToChange()
{
    return true;
}
