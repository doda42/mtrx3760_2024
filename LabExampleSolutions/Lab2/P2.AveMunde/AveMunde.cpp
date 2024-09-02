// Technical guidance only
// not an example of good coding style
#include <iostream>


class Speaker
{
	public:
		Speaker(): _greetCount(0) {}
		virtual ~Speaker(){ std::cout << "DTor" << std::endl; }  // check for virtual!
		virtual void GreetUser() = 0;
		void ReportGreets() { std::cout << "run count: " << _greetCount << std::endl; }
	protected:
		int _greetCount;
};

class EnglishSpeaker: public Speaker
{
	public:
		virtual void GreetUser() { std::cout << "Hello, World" << std::endl; ++_greetCount;}
};

class LatinSpeaker: public Speaker
{
	public:
		virtual void GreetUser() { std::cout << "Ave, Munde" << std::endl; ++_greetCount;}
};

class FrenchSpeaker: public Speaker
{
	public:
		virtual void GreetUser() { std::cout << "Salut, Monde" << std::endl; ++_greetCount;}
};

class TalkativeEnglishSpeaker: public EnglishSpeaker
{
	public:
		virtual ~TalkativeEnglishSpeaker() { std::cout << "Talkative DTor" << std::endl; }
		virtual void GreetUser() { EnglishSpeaker::GreetUser(); std::cout << "How ya goin'?" << std::endl; } // note no ++_greetCount
};

int main()
{
	const int NumSpeakers = 6;
	Speaker* AllSpeakers[NumSpeakers];


	std::cout << "Speaker type list?  (list 6 numbers in range 0..3)> ";
	for( int i=0; i<NumSpeakers; ++i )
	{
		int SpeakerType;
		std::cin >> SpeakerType;
		
		Speaker* pNewSpeaker = NULL;
		switch( SpeakerType )
		{
			case 0: pNewSpeaker = new TalkativeEnglishSpeaker; break;
			case 1: pNewSpeaker = new EnglishSpeaker; break;
			case 2: pNewSpeaker = new FrenchSpeaker; break;						
			case 3: pNewSpeaker = new LatinSpeaker; break;
		};
	
	
		AllSpeakers[i] = pNewSpeaker;
	}
	
		
	for( int j=0; j<3; ++j )
		for( int i=0; i<NumSpeakers; ++i )
		{
			AllSpeakers[i]->GreetUser();	
		}

	for( int i=0; i<NumSpeakers; ++i )
	{
		AllSpeakers[i]->ReportGreets();
	}

	for( int i=0; i<NumSpeakers; ++i )
	{
		delete AllSpeakers[i];
	}


}
