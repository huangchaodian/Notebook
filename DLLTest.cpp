#define DLLEXPORT_TEST
#include "DLLTest.h"
#include <iostream>
using namespace std;
DLLTest::DLLTest(void)
{
	cout<<"DLLTest construct "<<endl;
}

DLLTest::~DLLTest(void)
{
	cout<<"DLLTest destroy "<<endl;
}

int Add(int x, int y) 
{ 
	cout<<"DLLTest ADD"<<endl;
	return (x+y); 
} 