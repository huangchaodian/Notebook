#pragma once
#ifdef DLLEXPORT_TEST
#define DLL_TEST  _declspec (dllexport) 
#else 
#define DLL_TEST _declspec (dllimport) 
#endif
 class DLL_TEST DLLTest
{
public:
	DLLTest(void);
	~DLLTest(void);
};
int DLL_TEST Add(int x, int y);
//add path to lib path
//#pragma comment(lib,"TestSTL.lib")
//#include "../../TestSTL/TestSTL/DLLTest.h"