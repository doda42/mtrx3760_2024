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
// This example solution has some poor coding practices, most importantly
// a lack of commenting. It also constructs all its circuits in a hard-coded
// manner, using magic numbers and hard-coded component and wire instantiation.
//
// Copyright (c) Donald Dansereau, 2023


//--Includes-------------------------------------------------------------------
#include <iostream>

//--Consts and enums-----------------------------------------------------------
const int MaxInputsPerComponent = 10;         
const int MaxOutputsPerComponent = 10;         
const int MaxFanout = 10;                     // maximum fanout: max inputs that one output can drive

const int MaxComponentsPerCircuit = 10;       // 
const int MaxWiresPerCircuit = 10;            // 

const int MaxTesterInputBits = 10;            // max bits the tester can drive
const int MaxTesterOutputBits = 10;           // max bits the tester can observe


enum eLogicLevel                              // enum defining the possible states of a logic line
{
  LOGIC_UNDEFINED = -1,
  LOGIC_LOW,
  LOGIC_HIGH
};

//---Forward Declarations------------------------------------------------------
class CComponent;  // forward declaration

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
    void DriveLevel( eLogicLevel aNewLevel );
    
  private:
    int mNumOutputConnections;                  // how many outputs are connected
    CComponent* mpComponentsToDrive[MaxFanout]; // list of connected components
    int mComponentInputIndices[MaxFanout];      // list of input to drive in each components
};

//---Component-----------------------------------------------------------------
// Base class for components.
// Logic gates like and, or, xor should derive from this and override ComputeOutput
// calling the base-class ComputeOutput to drive the output wires.
class CComponent
{
  public:
    CComponent();
    void ConnectOutput( CWire* apOutputConnection, int aOutputIndex );
    void DriveInput( int aInputIndex, eLogicLevel aNewLevel );
    eLogicLevel GetOutputState( int aOutputIndex );
    
  protected:
    virtual void ComputeOutput();

    int mNumInputs;             
    eLogicLevel mInputs[MaxInputsPerComponent];
    eLogicLevel mOutputValues[MaxOutputsPerComponent];
    CWire* mpOutputWires[MaxOutputsPerComponent];
};

//---CNandGate-----------------------------------------------------------------
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
    eLogicLevel StateToLogicLevel( int WhichBit );
    
    int mCurState;
    int mNumInputsToDrive;
    CWire* mpInputsToDrive[MaxTesterInputBits];
    int mNumOutputValuesToObserve;
    CComponent* mpComponentsToObserve[MaxTesterOutputBits];
    int mComponentOutputIdx[MaxTesterOutputBits];
};

//---CCircuit------------------------------------------------------------------
// A CCircuit is a collection of components connected by wires, i.e. a circuit
// has components and a circuit has wires.
// In this simulation a CCircuit also inherits from CComponent, i.e. a circuit
// is a component. This allows circuits to be used as sub-circuits in larger
// circuits.
// Just as logic gates derive from CComponent, subcircuits like half adders
// and full adders should derive from CCircuit and override ComputeOutput from
// the CComponent abstract base class. ConnectTester is pure abstract and must
// be defined by each derived subcircuit class. The constructor of each 
// subcircuit is a reasonable place to hard-code initialisation.
class CCircuit: public CComponent
{
  public:
    CCircuit();    
    virtual ~CCircuit();
  
    void AddComponent( CComponent* apComponent );
    void AddWire( CComponent* apComponentToDrive, int aComponentInputToDrive );
    
    virtual void ConnectTester( CTester* apTester ) = 0;
    
  protected:
    int mNumComponents;
    CComponent* mpComponents[MaxComponentsPerCircuit];
    
    int mNumWires;
    CWire mWires[MaxWiresPerCircuit];
};
//---
class CHalfAdder: public CCircuit
{
  public:
    CHalfAdder();
    void ConnectTester( CTester* apTester );
  private:
    void ComputeOutput();
};
//---
class CFullAdder: public CCircuit
{
  public:
    CFullAdder();
    void ConnectTester( CTester* apTester );
  private:
    void ComputeOutput();
};
//---
class C3BitAdder: public CCircuit
{
  public:
    C3BitAdder();
    void ConnectTester( CTester* apTester );
  private:
    void ComputeOutput();
};

//---main----------------------------------------------------------------------
int main()
{
  // This main contains much more than was requested, as a means of demonstrating
  // each level of complexity in the hierarchy. The lab requested only the final test
  // showing the 3-bit adder in action.
  
  {
    CXorGate TestGate;
    TestGate.DriveInput( 0, LOGIC_HIGH );
    TestGate.DriveInput( 1, LOGIC_LOW );
    std::cout << "Test gate output: " << TestGate.GetOutputState( 0 ) << std::endl; 
  }

  {
    CHalfAdder MyHalfAdder;
    CTester MyTester;
    MyHalfAdder.ConnectTester( &MyTester );
    MyTester.RunTest();
  }  

  {
    CFullAdder MyFullAdder;
    CTester MyTester;
    MyFullAdder.ConnectTester( &MyTester );
    MyTester.RunTest();
  }  
  
  {
    C3BitAdder MyNBitAdder;
    CTester MyTester;
    MyNBitAdder.ConnectTester( &MyTester );
    MyTester.RunTest();
  }  
  
  return 0;
}

//---CCircuit Implementation---------------------------------------------------
CCircuit::CCircuit()
  : mNumComponents(0), 
    mNumWires(0)
{
  for( int i=0; i<MaxComponentsPerCircuit; ++i )
    mpComponents[i] = NULL;
}
//---
CCircuit::~CCircuit()
{
  for( int i=0; i<mNumComponents; ++i )
    if( mpComponents[i] != NULL )
      delete mpComponents[i];
}
//---    
void CCircuit::AddComponent( CComponent* apComponent )
{
  mpComponents[ mNumComponents ] = apComponent;
  ++mNumComponents;
}
//---
void CCircuit::AddWire( CComponent* apComponentToDrive, int aComponentInputToDrive )
{
  mWires[mNumWires].AddOutputConnection( apComponentToDrive, aComponentInputToDrive );
  ++mNumWires;
}

//---CTester Implementation----------------------------------------------------
CTester::CTester()
:
  mCurState( 0 ),
  mNumInputsToDrive( 0 ),
  mNumOutputValuesToObserve( 0 )
{
  for( int i=0; i<MaxTesterInputBits; ++i )
    mpInputsToDrive[i] = NULL;

  for( int i=0; i<MaxTesterOutputBits; ++i )
  {
    mpComponentsToObserve[i] = NULL;
    mComponentOutputIdx[i] = -1;
  }
}
//---
void CTester::AddInputToDrive( CWire* apInputToDrive )
{
  mpInputsToDrive[mNumInputsToDrive] = apInputToDrive;
  ++mNumInputsToDrive;
}
//---
void CTester::AddOutputToObserve( CComponent* apComponentToObserve, int aOutputIndex )
{
  mpComponentsToObserve[mNumOutputValuesToObserve] = apComponentToObserve;
  mComponentOutputIdx[mNumOutputValuesToObserve] = aOutputIndex;
  ++mNumOutputValuesToObserve;
}
//---
void CTester::RunTest()
{
  for( int i=0; i < (1<<mNumInputsToDrive); ++i )
  {
    std::cout << "Testing input ";
    PrintStateBinary();
    DriveOutputs();
    std::cout << " result: ";
    for( int iOut=0; iOut<mNumOutputValuesToObserve; ++iOut )
      std::cout << mpComponentsToObserve[iOut]->GetOutputState( mComponentOutputIdx[iOut] ) << " ";
    std::cout << std::endl;

    ++mCurState;
  }
}
//---
void CTester::PrintStateBinary()
{
  for( int i=mNumInputsToDrive-1; i>=0; --i )
    std::cout << StateToLogicLevel(i) << " ";
}
//---
void CTester::DriveOutputs()
{
  for( int i=0; i<mNumInputsToDrive; ++i )
  {
    if( mpInputsToDrive[i] != NULL )
      mpInputsToDrive[i]->DriveLevel( StateToLogicLevel(i) );
  }
}
//---
eLogicLevel CTester::StateToLogicLevel( int WhichBit )
{
  eLogicLevel Result = LOGIC_LOW;
  int MaskedVal = mCurState & (1<<WhichBit);
  if( MaskedVal )
    Result = LOGIC_HIGH;
  return Result;
}


//---CWire Implementation------------------------------------------------------
CWire::CWire()
: mNumOutputConnections( 0 )
{
  for( int i=0; i<MaxFanout; ++i )
  {
    mpComponentsToDrive[i] = NULL;
    mComponentInputIndices[i] = 0;
  }
}
//---
void CWire::AddOutputConnection( CComponent* apComponentToDrive, int aComponentInputToDrive )
{
  mpComponentsToDrive[mNumOutputConnections] = apComponentToDrive;
  mComponentInputIndices[mNumOutputConnections] = aComponentInputToDrive;
  ++mNumOutputConnections;
}
//---
void CWire::DriveLevel( eLogicLevel aNewLevel )
{
  for( int i=0; i<mNumOutputConnections; ++i )
  {
    if( mpComponentsToDrive[i] != NULL )
      mpComponentsToDrive[i]->DriveInput( mComponentInputIndices[i], aNewLevel );
  }
}

//---CComponent Implementation-------------------------------------------------
CComponent::CComponent() 
 :  mNumInputs( 2 )      // default to two inputs per component
{
  for( int i=0; i<MaxInputsPerComponent; ++i )
    mInputs[i] = LOGIC_UNDEFINED;
    
  for( int i=0; i<MaxOutputsPerComponent; ++i )
  {
    mOutputValues[i] = LOGIC_UNDEFINED;
    mpOutputWires[i] = NULL; // we are using NULL to signal when no wire is connected
  }
}
//---
void CComponent::ConnectOutput( CWire* apOutputConnection, int aOutputIndex )
{
  mpOutputWires[aOutputIndex] = apOutputConnection;
}
//---
void CComponent::DriveInput( int aInputIndex, eLogicLevel aNewLevel )
{
  mInputs[aInputIndex] = aNewLevel;
  ComputeOutput();
}
//---
eLogicLevel CComponent::GetOutputState( int aOutputIndex ) 
{ 
  return mOutputValues[aOutputIndex]; 
}
//---
void CComponent::ComputeOutput()
{
  for( int i=0; i<MaxOutputsPerComponent; ++i )
  {
    if( mpOutputWires[i] != NULL )
      mpOutputWires[i]->DriveLevel( mOutputValues[i] );
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


//---Half Adder implementation-------------------------------------------------
CHalfAdder::CHalfAdder()
{
  AddComponent( new CXorGate );
  AddWire( mpComponents[0], 0 );
  AddWire( mpComponents[0], 1 );

  AddComponent( new CAndGate );
  mWires[0].AddOutputConnection( mpComponents[1], 0 );
  mWires[1].AddOutputConnection( mpComponents[1], 1 );
}
//---
void CHalfAdder::ConnectTester( CTester* apTester )
{
  apTester->AddOutputToObserve( mpComponents[0], 0 );
  apTester->AddOutputToObserve( mpComponents[1], 0 );
  apTester->AddInputToDrive( &mWires[0] );
  apTester->AddInputToDrive( &mWires[1] );
}
//---
void CHalfAdder::ComputeOutput()
{ 
  mWires[0].DriveLevel( mInputs[0] );
  mWires[1].DriveLevel( mInputs[1] );
  mOutputValues[0] = mpComponents[0]->GetOutputState( 0 );
  mOutputValues[1] = mpComponents[1]->GetOutputState( 0 );      

  CComponent::ComputeOutput();
}


//---Full Adder implementation-------------------------------------------------
CFullAdder::CFullAdder()
{
  AddComponent( new CXorGate );  // component 0
  AddComponent( new CXorGate );  // component 1
  AddComponent( new CAndGate );  // component 2
  AddComponent( new CAndGate );  // component 3
  AddComponent( new COrGate );   // component 4

  AddWire( mpComponents[0], 0 ); // wire 0: input A
  AddWire( mpComponents[0], 1 ); // wire 1: input B
  AddWire( mpComponents[1], 1 ); // wire 2: input C
  AddWire( mpComponents[1], 0 ); // wire 3: between xors
  AddWire( mpComponents[4], 0 ); // wire 4: and (2) to output or
  AddWire( mpComponents[4], 1 ); // wire 5: and (3) to output or  

  mpComponents[0]->ConnectOutput( &mWires[3], 0 );
  mpComponents[2]->ConnectOutput( &mWires[4], 0 );
  mpComponents[3]->ConnectOutput( &mWires[5], 0 );  

  mWires[0].AddOutputConnection( mpComponents[3], 1 );
  mWires[1].AddOutputConnection( mpComponents[3], 0 );
  mWires[2].AddOutputConnection( mpComponents[2], 0 );
  mWires[3].AddOutputConnection( mpComponents[2], 1 );
}
//---
void CFullAdder::ConnectTester( CTester* apTester )
{
  apTester->AddOutputToObserve( mpComponents[1], 0 );
  apTester->AddOutputToObserve( mpComponents[4], 0 );  
  apTester->AddInputToDrive( &mWires[0] );
  apTester->AddInputToDrive( &mWires[1] );
  apTester->AddInputToDrive( &mWires[2] );
}
//---
void CFullAdder::ComputeOutput()
{ 
  for( int i=0; i<3; ++i )
    mWires[i].DriveLevel( mInputs[i] );
    
  mOutputValues[0] = mpComponents[1]->GetOutputState( 0 );
  mOutputValues[1] = mpComponents[4]->GetOutputState( 0 );
  
  CComponent::ComputeOutput();
}

//---3-bit adder implementation------------------------------------------------
C3BitAdder::C3BitAdder()
{
  AddComponent( new CHalfAdder ); // component 0
  AddComponent( new CFullAdder ); // component 1
  AddComponent( new CFullAdder ); // component 2

  AddWire( mpComponents[0], 0 ); // wire 0: Half-adder input A
  AddWire( mpComponents[1], 0 ); // wire 1: Full-adder input A
  AddWire( mpComponents[2], 0 ); // wire 2: Full-adder input A    
  AddWire( mpComponents[0], 1 ); // wire 3: Half-adder input B
  AddWire( mpComponents[1], 1 ); // wire 4: Full-adder input B
  AddWire( mpComponents[2], 1 ); // wire 5: Full-adder input B
  
  AddWire( mpComponents[1], 2 ); // wire 6: Half adder to full adder
  AddWire( mpComponents[2], 2 ); // wire 7: Full adder to full adder

  mpComponents[0]->ConnectOutput( &mWires[6], 1 );
  mpComponents[1]->ConnectOutput( &mWires[7], 1 );
}
//---
void C3BitAdder::ConnectTester( CTester* apTester )
{
  apTester->AddOutputToObserve( mpComponents[0], 0 );
  apTester->AddOutputToObserve( mpComponents[1], 0 );
  apTester->AddOutputToObserve( mpComponents[2], 0 );
  apTester->AddOutputToObserve( mpComponents[2], 1 );

  apTester->AddInputToDrive( &mWires[0] );
  apTester->AddInputToDrive( &mWires[3] );
  apTester->AddInputToDrive( &mWires[1] );
  apTester->AddInputToDrive( &mWires[4] );
  apTester->AddInputToDrive( &mWires[2] );
  apTester->AddInputToDrive( &mWires[5] );
}
//---
void C3BitAdder::ComputeOutput()
{ 
  // warning: untested
  for( int i=0; i<5; ++i )
    mWires[i].DriveLevel( mInputs[i] );

  mOutputValues[0] = mpComponents[1]->GetOutputState( 0 );
  mOutputValues[1] = mpComponents[2]->GetOutputState( 0 );
  mOutputValues[2] = mpComponents[3]->GetOutputState( 0 );
  mOutputValues[3] = mpComponents[3]->GetOutputState( 1 );
  
  CComponent::ComputeOutput();
}

