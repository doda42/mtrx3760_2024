// A combinatorial logic circuit simulator
//
// The simulated circuit has logic gates and wires to connect them. Test 
// signals are generated to test the circuit's output, which is printed
// to screen.
//
// This approach uses polymorphism to define a CComponent class that is
// derived into specific gates. 
//
// A CCircuit class has components and wires. CCircuit also inherits from 
// CComponent, allowing circuits to be used as sub-circuits in more complex
// circuits.
//
// This version of the program follows the "encapsulate everything" approach
// to OOD in which everything, including constants and types, is embedded 
// in a class.
//
// This version accepts input from an external file through piping, and uses
// stl containers, so that no magic numbers are required anywhere.
//
// A more elegant solution is available by making a wire a component and
// generalising input and output pins, so there are no special cases for 
// connecting wires / components.
//
// This example solution has some poor coding practices, most importantly
// a lack of commenting. It could also use references rather than pointers
// in places and it is not const-correct. It should be broken across multiple
// files.
//
// Copyright (c) Donald Dansereau, 2023


//--Includes-------------------------------------------------------------------
#include <iostream>
#include <unordered_map>
#include <vector>
#include <assert.h>


//---Forward Declarations------------------------------------------------------
class CWire;  // forward declaration

//---Component-----------------------------------------------------------------
// Base class for components.
// Logic gates like and, or, xor should derive from this and override ComputeOutput
// calling the base-class ComputeOutput to drive the output wires.
class CComponent
{
  public:
  
    //--Structs, typedefs and enums--------------------------------------------
    enum eLogicLevel                              // enum defining the possible states of a logic line
    {
      LOGIC_UNDEFINED = -1,
      LOGIC_LOW,
      LOGIC_HIGH
    };
    struct SWireToPinConnection
    {
      CComponent* pComponent;
      int PinNumber;
    };
    struct SPinToWireConnection
    {
      CWire* pWire;
      int OutputPinNumber;
    };

    //-------------------------------------------------------------------------
    CComponent();
    void ConnectOutput( CWire* apOutputConnection, int aOutputIndex );
    void DriveInput( int aInputIndex, eLogicLevel aNewLevel );
    eLogicLevel GetOutputState( int aOutputIndex );
    
  protected:
    virtual void ComputeOutput();

    std::vector<eLogicLevel> mInputs;
    std::vector<eLogicLevel> mOutputValues;
    std::vector<SPinToWireConnection> mOutputWires;
};

//---CNotGate-----------------------------------------------------------------
class CNotGate: public CComponent
{
  private:
    void ComputeOutput();
};
//---CNandGate------------------------------------------------------------------
class CNandGate: public CComponent
{
  private:
    void ComputeOutput();
};
//---CAndGate------------------------------------------------------------------
class CAndGate: public CComponent
{
  private:
    void ComputeOutput();
};
//---COrGate-------------------------------------------------------------------
class COrGate: public CComponent
{
  private:
    void ComputeOutput();
};
//---CXorGate------------------------------------------------------------------
class CXorGate: public CComponent
{
  private:
    void ComputeOutput();
};

//---CWire Interface-----------------------------------------------------------
// CWire is used to connect devices in this simulation
class CWire
{
  public:
    CWire();
    
    // AddOutputConnection adds to the list of outputs that this wire drives
    void AddOutputConnection( CComponent* apComponentToDrive, int aComponentInputToDrive );
    
    // DriveLevel drives the wire's value, so that each of its connected outputs
    // get set to the corresponding level
    void DriveLevel( CComponent::eLogicLevel aNewLevel );
    
  private:
    std::vector<CComponent::SWireToPinConnection> mPinsToDrive;
};



//---CTester-------------------------------------------------------------------
class CTester
{
  public:
    CTester();
    void AddInputToDrive( CWire* apInputToDrive );
    void AddOutputToObserve( CComponent* apComponentToObserve, int aOutputIndex );
    void RunTest();

  private:
    void PrintStateBinary();
    void DriveOutputs();
    CComponent::eLogicLevel StateToLogicLevel( int WhichBit );
    
    int mCurState;
    std::vector<CWire*> mInputsToDrive;
    std::vector<CComponent::SWireToPinConnection> mComponentsToObserve;
};

//---CCircuit------------------------------------------------------------------
class CCircuit: public CComponent
{
  public:
    virtual ~CCircuit();
    
    void BuildFromCinStream();
    void RunTest();

    void AddComponent( CComponent* pComponent, std::string ComponentName );
    void AddWire( std::string GateName, int aComponentInputToDrive, std::string WireName );

  protected:
    CTester mTester;
    
    std::unordered_map<std::string, CComponent*> mComponentMap;    
    std::unordered_map<std::string, CWire*> mWireMap;
};


//---main----------------------------------------------------------------------
int main()
{
  CCircuit MyCircuit;
  MyCircuit.BuildFromCinStream();
  MyCircuit.RunTest();
  
  return 0;
}

//---
CCircuit::~CCircuit()
{
  for( auto& it: mComponentMap ) 
  {
    if( it.second != NULL )
      delete it.second;
  }
  
  for( auto& it: mWireMap ) 
  {
    if( it.second != NULL )
      delete it.second;
  }
}
//---    
void CCircuit::AddComponent( CComponent* apComponent, std::string aComponentName )
{
  mComponentMap[ aComponentName ] = apComponent;
}
//---
void CCircuit::AddWire( std::string GateName, int aComponentInputToDrive, std::string aWireName )
{
  CComponent* pDestComponent = mComponentMap[ GateName ];
  assert( pDestComponent != NULL );

  CWire* pNewWire = new CWire;
  pNewWire->AddOutputConnection( pDestComponent, aComponentInputToDrive );
  mWireMap[ aWireName ] = pNewWire;
}

//---CTester Implementation----------------------------------------------------
CTester::CTester()
:
  mCurState( 0 )
{
}
//---
void CTester::AddInputToDrive( CWire* apInputToDrive )
{
  mInputsToDrive.push_back( apInputToDrive );
}
//---
void CTester::AddOutputToObserve( CComponent* apComponentToObserve, int aOutputIndex )
{
  CComponent::SWireToPinConnection NewConnection;
  NewConnection.pComponent = apComponentToObserve;
  NewConnection.PinNumber = aOutputIndex;
  
  mComponentsToObserve.push_back( NewConnection );
}
//---
void CTester::RunTest()
{
  for( int i=0; i < (1<<mInputsToDrive.size()); ++i )
  {
    std::cout << "Testing input ";
    PrintStateBinary();
    DriveOutputs();
    std::cout << " result: ";
    for( int iOut=0; iOut<mComponentsToObserve.size(); ++iOut )
      std::cout << mComponentsToObserve[iOut].pComponent->GetOutputState( mComponentsToObserve[iOut].PinNumber ) << " ";
    std::cout << std::endl;

    ++mCurState;
  }
}
//---
void CTester::PrintStateBinary()
{
  for( int i=mInputsToDrive.size()-1; i>=0; --i )
    std::cout << StateToLogicLevel(i) << " ";
}
//---
void CTester::DriveOutputs()
{
  for( int i=0; i<mInputsToDrive.size(); ++i )
  {
    if( mInputsToDrive[i] != NULL )
      mInputsToDrive[i]->DriveLevel( StateToLogicLevel(i) );
  }
}
//---
CComponent::eLogicLevel CTester::StateToLogicLevel( int WhichBit )
{
  CComponent::eLogicLevel Result = CComponent::LOGIC_LOW;
  int MaskedVal = mCurState & (1<<WhichBit);
  if( MaskedVal )
    Result = CComponent::LOGIC_HIGH;
  return Result;
}


//---CWire Implementation------------------------------------------------------
CWire::CWire()
{
}
//---
void CWire::AddOutputConnection( CComponent* apComponentToDrive, int aComponentInputToDrive )
{
  CComponent::SWireToPinConnection NewConnection;
  NewConnection.pComponent = apComponentToDrive;
  NewConnection.PinNumber = aComponentInputToDrive;
  mPinsToDrive.push_back( NewConnection );
}
//---
void CWire::DriveLevel( CComponent::eLogicLevel aNewLevel )
{
  for( int i=0; i<mPinsToDrive.size(); ++i )
  {
    if( mPinsToDrive[i].pComponent != NULL )
      mPinsToDrive[i].pComponent->DriveInput( mPinsToDrive[i].PinNumber, aNewLevel );
  }
}

//---CComponent Implementation-------------------------------------------------
CComponent::CComponent() 
: mInputs( 2, LOGIC_UNDEFINED ),
  mOutputValues( 1, LOGIC_UNDEFINED )
{
}
//---
void CComponent::ConnectOutput( CWire* apOutputConnection, int aOutputIndex )
{
  SPinToWireConnection NewConnection;
  NewConnection.pWire = apOutputConnection;
  NewConnection.OutputPinNumber = aOutputIndex;
  mOutputWires.push_back( NewConnection );
}
//---
void CComponent::DriveInput( int aInputIndex, eLogicLevel aNewLevel )
{
  mInputs[aInputIndex] = aNewLevel;
  ComputeOutput();
}
//---
CComponent::eLogicLevel CComponent::GetOutputState( int aOutputIndex ) 
{ 
  return mOutputValues[aOutputIndex];
}
//---
void CComponent::ComputeOutput()
{
  for( int i=0; i<mOutputWires.size(); ++i )
  {
    if( mOutputWires[i].pWire != NULL )
      mOutputWires[i].pWire->DriveLevel( mOutputValues[ mOutputWires[i].OutputPinNumber ] );
  }
}

//---Implementation for each gate----------------------------------------------
void CNandGate::ComputeOutput()
{
  eLogicLevel NewVal = LOGIC_HIGH;
  if( mInputs[0] == LOGIC_UNDEFINED || mInputs[1] == LOGIC_UNDEFINED )
    NewVal = LOGIC_UNDEFINED;
  else if( mInputs[0] == LOGIC_HIGH && mInputs[1] == LOGIC_HIGH )
    NewVal = LOGIC_LOW;

  mOutputValues[0] = NewVal;
  
  CComponent::ComputeOutput();
}
//---
void CNotGate::ComputeOutput()
{
  eLogicLevel NewVal = LOGIC_UNDEFINED;
  if( mInputs[0] == LOGIC_LOW )
    NewVal = LOGIC_HIGH;
  else if( mInputs[0] == LOGIC_HIGH )
    NewVal = LOGIC_LOW;

  mOutputValues[0] = NewVal;

  CComponent::ComputeOutput();
}
//---
void CAndGate::ComputeOutput()
{
  eLogicLevel NewVal = LOGIC_LOW;
  if( mInputs[0] == LOGIC_UNDEFINED || mInputs[1] == LOGIC_UNDEFINED )
    NewVal = LOGIC_UNDEFINED;
  else if( mInputs[0] == LOGIC_HIGH && mInputs[1] == LOGIC_HIGH )
    NewVal = LOGIC_HIGH;

  mOutputValues[0] = NewVal;
  
  CComponent::ComputeOutput();
}
//---
void COrGate::ComputeOutput()
{
  eLogicLevel NewVal = LOGIC_LOW;
  if( mInputs[0] == LOGIC_UNDEFINED || mInputs[1] == LOGIC_UNDEFINED )
    NewVal = LOGIC_UNDEFINED;
  else if( mInputs[0] == LOGIC_HIGH || mInputs[1] == LOGIC_HIGH )
    NewVal = LOGIC_HIGH;

  mOutputValues[0] = NewVal;

  CComponent::ComputeOutput();
}
//---
void CXorGate::ComputeOutput()
{
  eLogicLevel NewVal = LOGIC_LOW;
  if( mInputs[0] == LOGIC_UNDEFINED || mInputs[1] == LOGIC_UNDEFINED )
    NewVal = LOGIC_UNDEFINED;
  else if( (mInputs[0] == LOGIC_HIGH) ^ (mInputs[1] == LOGIC_HIGH) )
    NewVal = LOGIC_HIGH;

  mOutputValues[0] = NewVal;
 
  CComponent::ComputeOutput();
}

//---
void CCircuit::BuildFromCinStream()
{
  while( true )
  {
    std::string Request;
    std::cin >> Request;  // get the next word from the input stream
    
    if( Request[0] == '#' )
    {
      // a comment line
      // get the rest of the line and ignore it
      std::string DummyVar;
      getline( std::cin, DummyVar );
    }
    else if( Request.compare( "component" ) == 0 )
    {
      std::string GateType;
      std::string GateName;
      std::cin >> GateType;
      std::cin >> GateName;
      std::cout << "Adding gate of type " << GateType << " named " << GateName << std::endl;

      // create the gate      
      CComponent* pNewComponent = NULL;   
      if( GateType.compare( "xor" ) == 0 )
      {
        pNewComponent = new CXorGate;
      }
      else if( GateType.compare( "and" ) == 0 )
      {
        pNewComponent = new CXorGate;
      }
      else if( GateType.compare( "or" ) == 0 )
      {
        pNewComponent = new CXorGate;
      }
      else if( GateType.compare( "not" ) == 0 )
      {
        pNewComponent = new CNotGate;
      }
      
      AddComponent( pNewComponent, GateName );

    }
    else if( Request.compare( "wire" ) == 0 )
    {
      std::string WireName;
      std::string GateName;
      int GateInputIdx;
      
      std::cin >> GateName;      
      std::cin >> GateInputIdx;
      std::cin >> WireName;
      
      std::cout << "Adding wire named " << WireName << " driving " << GateName << " input pin " << GateInputIdx << std::endl;

      AddWire( GateName, GateInputIdx, WireName );
    }
    else if( Request.compare( "connect" ) == 0 )
    {

      std::string SourceName;
      std::string DestName;
      int PinIdx;
      std::cin >> SourceName;      
      std::cin >> DestName;
      std::cin >> PinIdx;

      auto FindComponent = mComponentMap.find( SourceName );
      if( FindComponent != mComponentMap.end() )
      {
        std::cout << "Adding connection from component " << SourceName << " pin " << PinIdx << " to wire " << DestName << std::endl;
        CComponent* pSrcComponent = FindComponent->second;
        CWire* pDestWire = mWireMap[ DestName ];
        assert( pDestWire != NULL );
        pSrcComponent->ConnectOutput( pDestWire, PinIdx );
      }
      else
      {
        CWire* pSrcWire = mWireMap[ SourceName ];
        CComponent* pDestComponent = mComponentMap[ DestName ];
        assert( pSrcWire != NULL );
        assert( pDestComponent != NULL );
        std::cout << "Adding connection from wire " << SourceName << " to component " << DestName << " pin idx " << PinIdx << std::endl;
        pSrcWire->AddOutputConnection( pDestComponent, PinIdx );
      }
    } 
    else if( Request.compare( "testerOutput" ) == 0 )
    {
      std::string GateName;
      int GateOutputIdx;
      
      std::cin >> GateName;      
      std::cin >> GateOutputIdx;
      
      CComponent* pComponent = mComponentMap[ GateName ];
      assert( pComponent != NULL );
      mTester.AddOutputToObserve( pComponent, GateOutputIdx );
      
    }
    else if( Request.compare( "testerInput" ) == 0 )
    {
      std::string WireName;
      std::cin >> WireName;
      
      CWire* pWire = mWireMap[ WireName ];
      assert( pWire != NULL );
    
      mTester.AddInputToDrive( pWire );
    }
    else if( Request.compare( "end" ) == 0 )
    {
      break; // end of file
    }
    else
    {
      std::cout << "Unrecognised command " << Request << std::endl;
      std::cout << "Continuing to next line" << std::endl;
      // get the rest of the line and ignore it
      std::string DummyVar;
      getline( std::cin, DummyVar );
    }
  }
}

//---
void CCircuit::RunTest()
{ 
  mTester.RunTest(); 
}

