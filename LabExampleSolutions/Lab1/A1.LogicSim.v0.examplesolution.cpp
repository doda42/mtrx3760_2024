// A combinatorial logic circuit simulator
//
// The simulated circuit has logic gates and wires to connect them. Test 
// signals are generated to test the circuit's output, which is printed
// to screen.
//
// Only NAND gates are supported, and the circuit topology is hard-coded.
//
// This code is functionally complete: it is capable of simulating any non-
// recurrent combinatorial logic function. The provided example demonstrates 
// a 2-input, 2-gate, 1-output logic function.
//
// This example solution has been redesigned to encapsulate the content of
// main, however it still has some poor coding practices, most importantly
// a lack of commenting. The CWire class interface is a good example of 
// appropriate commenting. 
//
// Copyright (c) Donald Dansereau, 2023


//--Includes-------------------------------------------------------------------
#include <iostream>

//--Consts and enums-----------------------------------------------------------
const int InputsPerGate = 2;                 // number of inputs per nand gate
const int MaxFanout = 2;                     // maximum fanout: max gate inputs that one gate output can drive

const int NumNandGates = 2;                  // number of nand gates
const int NumWires = 3;                      // number of wires

const int MaxTesterBits = 3;                 // max bits the tester can drive
const int MaxOutputBits = 2;                 // max bits the tester can observe


enum eLogicLevel                             // enum defining the possible states of a logic line
{
  LOGIC_UNDEFINED = -1,
  LOGIC_LOW,
  LOGIC_HIGH
};

//---Forward Declarations------------------------------------------------------
class CNandGate;  // forward declaration

//---CWire Interface-----------------------------------------------------------
// CWire is used to connect devices in this simulation
// A CWire has a single input, and may drive multiple outputs
// The global variable MaxFanout controls how many outputs each wire can have
// Each wire output drives a specific input of a specific gate
// The wire's input is controlled via the DriveLevel function
class CWire
{
  public:
    void Init();
    
    // AddOutputConnection adds to the list of outputs that this wire drives
    // It accepts as parameters the nand gate whose input should be driven
    // and the index specifying which of that gate's inputs should be driven.
    void AddOutputConnection( CNandGate* apGateToDrive, int aGateInputToDrive );
    
    // DriveLevel drives the wire's value, so that each of its connected outputs
    // get set to the corresponding level
    void DriveLevel( eLogicLevel aNewLevel );
    
  private:
    int mNumOutputConnections;            // how many outputs are connected
    CNandGate* mpGatesToDrive[MaxFanout]; // list of connected gates
    int mGateInputIndices[MaxFanout];     // list of input to drive in each gate
};

//---CNandGate Interface-------------------------------------------------------
class CNandGate
{
  public:
    void Init();
    void ConnectOutput( CWire* apOutputConnection );
    void DriveInput( int aInputIndex, eLogicLevel aNewLevel );
    eLogicLevel GetOutputState();
    
  private:
    void ComputeOutput();

    eLogicLevel mInputs[ InputsPerGate ];
    eLogicLevel mOutputValue;
    CWire* mpOutputConnection;
};

//---
class CTester
{
  public:
    void Init();
    void AddInputToDrive( CWire* apInputToDrive );
    void AddOutputToObserve( CNandGate* apGateToObserve );
    void RunTest();

  private:
    void PrintStateBinary();
    void DriveOutputs();
    eLogicLevel StateToLogicLevel( int WhichBit );
    
    int mNumInputsToDrive;
    CWire* mInputsToDrive[MaxTesterBits];
    int mNumOutputsToObserve;
    CNandGate* mpGatesToObserve[MaxOutputBits];
    int mCurState;
};

//---
class CCircuit
{
  public:
    void Init();    
    void RunTester();
    
  private:
    CTester mTester;
    CNandGate mGates[NumNandGates];
    CWire mWires[NumWires];
};


//---main----------------------------------------------------------------------
int main()
{
  CCircuit MyCircuit;
  MyCircuit.Init();
  MyCircuit.RunTester();

  return 0;
}


//---CCircuit Implementation---------------------------------------------------
void CCircuit::Init()
{
  for( int i=0; i<NumNandGates; ++i )
    mGates[i].Init();
    
  for( int i=0; i<NumWires; ++i )
    mWires[i].Init();

  // mWires[0] and [1] are input wires for the circuit
  mWires[0].AddOutputConnection( &mGates[0], 0 );
  mWires[0].AddOutputConnection( &mGates[1], 0 );
  mWires[1].AddOutputConnection( &mGates[0], 1 );
  
  // mWires[2] is a connection between the output of mGates[0] and mGates[1] input 1
  mWires[2].AddOutputConnection( &mGates[1], 1 );
  mGates[0].ConnectOutput( &mWires[2] );

  // Set up the tester
  mTester.Init();
  mTester.AddOutputToObserve( &mGates[1] );
  mTester.AddInputToDrive( &mWires[1] );
  mTester.AddInputToDrive( &mWires[0] );
}
//---
void CCircuit::RunTester()
{
  mTester.RunTest();
}


//---CTester Implementation----------------------------------------------------
void CTester::Init()
{ 
  mCurState = 0;
  mNumInputsToDrive = 0;
  mNumOutputsToObserve = 0;
}
//---
void CTester::AddInputToDrive( CWire* apInputToDrive )
{
  mInputsToDrive[mNumInputsToDrive] = apInputToDrive;
  ++mNumInputsToDrive;
}
//---
void CTester::AddOutputToObserve( CNandGate* apGateToObserve )
{
  mpGatesToObserve[mNumOutputsToObserve] = apGateToObserve;
  ++mNumOutputsToObserve;
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
    for( int iOut=0; iOut<mNumOutputsToObserve; ++iOut )
      std::cout << mpGatesToObserve[iOut]->GetOutputState() << " ";
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
    mInputsToDrive[i]->DriveLevel( StateToLogicLevel(i) );
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
void CWire::Init()
{
  mNumOutputConnections = 0;
}
//---
void CWire::AddOutputConnection( CNandGate* apGateToDrive, int aGateInputToDrive )
{
  mpGatesToDrive[mNumOutputConnections] = apGateToDrive;
  mGateInputIndices[mNumOutputConnections] = aGateInputToDrive;
  ++mNumOutputConnections;
}
//---
void CWire::DriveLevel( eLogicLevel aNewLevel )
{
  for( int i=0; i<mNumOutputConnections; ++i )
    mpGatesToDrive[i]->DriveInput( mGateInputIndices[i], aNewLevel );
}

//---CNandGate Implementation--------------------------------------------------
void CNandGate::Init() 
{
  mInputs[0] = mInputs[1] = LOGIC_UNDEFINED;
  mpOutputConnection = NULL;
  ComputeOutput();
}
//---
void CNandGate::ConnectOutput( CWire* apOutputConnection )
{
  mpOutputConnection = apOutputConnection;
}
//---
void CNandGate::DriveInput( int aInputIndex, eLogicLevel aNewLevel )
{
  mInputs[aInputIndex] = aNewLevel;
  ComputeOutput();
}
//---
eLogicLevel CNandGate::GetOutputState() 
{ 
  return mOutputValue; 
}
//---
void CNandGate::ComputeOutput()
{
  eLogicLevel NewVal = LOGIC_HIGH;
  if( mInputs[0] == LOGIC_UNDEFINED || mInputs[1] == LOGIC_UNDEFINED )
    NewVal = LOGIC_UNDEFINED;
  else if( mInputs[0] == LOGIC_HIGH && mInputs[1] == LOGIC_HIGH )
    NewVal = LOGIC_LOW;
  mOutputValue = NewVal;
  
  if( mpOutputConnection != NULL )
    mpOutputConnection->DriveLevel( mOutputValue );
}


