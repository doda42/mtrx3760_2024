// Instrumented version to get the class interaction sequence
// 
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
// Parts of the code are well-designed and well-written, but some rules of 
// object-oriented design are broken, in particular the contents of main are
// not well encapsulated in objects. There are also examples of poor coding 
// style including a lack of comments throughout.
//
// Copyright (c) Donald Dansereau, 2023


//--Includes-------------------------------------------------------------------
#include <iostream>

//--Consts and enums-----------------------------------------------------------
const int InputsPerGate = 2;                 // number of inputs per nand gate
const int MaxFanout = 2;                     // maximum fanout: max gate inputs that one gate output can drive

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


//---main----------------------------------------------------------------------
int main()
{
  const int NumNandGates = 2;                  // number of nand gates
  const int NumWires = 3;                      // number of wires

  CNandGate MyGates[NumNandGates];
  CWire MyWires[NumWires];

  for( int i=0; i<NumNandGates; ++i )
    MyGates[i].Init();
    
  for( int i=0; i<NumWires; ++i )
    MyWires[i].Init();

  // MyWires[0] and [1] are input wires for the circuit
  MyWires[0].AddOutputConnection( &MyGates[0], 0 );
  MyWires[0].AddOutputConnection( &MyGates[1], 0 );
  MyWires[1].AddOutputConnection( &MyGates[0], 1 );
  
  // MyWires[2] is a connection between the output of MyGates[0] and MyGates[1] input 1
  MyWires[2].AddOutputConnection( &MyGates[1], 1 );
  MyGates[0].ConnectOutput( &MyWires[2] );
  
  // Test each of the possible input states
  std::cout << "start" << std::endl;
  MyWires[0].DriveLevel( LOGIC_LOW );
//  MyWires[1].DriveLevel( LOGIC_LOW );
//  std::cout << "Testing input 0 0 result: " << MyGates[1].GetOutputState() << std::endl;
//  
//  MyWires[0].DriveLevel( LOGIC_LOW );
//  MyWires[1].DriveLevel( LOGIC_HIGH );
//  std::cout << "Testing input 0 1 result: " << MyGates[1].GetOutputState() << std::endl;
//  
//  MyWires[0].DriveLevel( LOGIC_HIGH );
//  MyWires[1].DriveLevel( LOGIC_LOW );
//  std::cout << "Testing input 1 0 result: " << MyGates[1].GetOutputState() << std::endl;
//  
//  MyWires[0].DriveLevel( LOGIC_HIGH );
//  MyWires[1].DriveLevel( LOGIC_HIGH );
//  std::cout << "Testing input 1 1 result: " << MyGates[1].GetOutputState() << std::endl;

  return 0;
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
  std::cout << "CWire::Drivelevel" << std::endl;
  for( int i=0; i<mNumOutputConnections; ++i )
    mpGatesToDrive[i]->DriveInput( mGateInputIndices[i], aNewLevel );
  std::cout << "CWire::Drivelevel done" << std::endl;
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
  std::cout << "CNandGate::DriveInput" << std::endl;
  mInputs[aInputIndex] = aNewLevel;
  ComputeOutput();
  std::cout << "CNandGate::DriveInput done" << std::endl;
}
//---
eLogicLevel CNandGate::GetOutputState() 
{ 
  return mOutputValue; 
}
//---
void CNandGate::ComputeOutput()
{
  std::cout << "CNandGate::ComputeOutput" << std::endl;
  eLogicLevel NewVal = LOGIC_HIGH;
  if( mInputs[0] == LOGIC_UNDEFINED || mInputs[1] == LOGIC_UNDEFINED )
    NewVal = LOGIC_UNDEFINED;
  else if( mInputs[0] == LOGIC_HIGH && mInputs[1] == LOGIC_HIGH )
    NewVal = LOGIC_LOW;
  mOutputValue = NewVal;
  
  if( mpOutputConnection != NULL )
    mpOutputConnection->DriveLevel( mOutputValue );
  std::cout << "CNandGate::ComputeOutput done" << std::endl;
}


