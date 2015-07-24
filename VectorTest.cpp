#include <vector>
#include <iostream>
#include "Vect.h"
#include "TemplateTest.hpp"
#include "TemplateTest.hpp"
using namespace std;

int main(){
	TemplateTest<int>TempA;
	TempA.print(3);
	{
		Vect va;
		{
			vector<Vect>v;//(av,av+5);
			//v.reserve(4);
			v.push_back(va);
			cout<<".....push one........"<<v.size()<<endl;
			v.push_back(va);
			cout<<".....push one........"<<v.size()<<endl;
			v.push_back(va);
			cout<<".....push one........"<<v.size()<<endl;
			v.push_back(va);
			cout<<".....push one........"<<v.size()<<endl;
			v.push_back(va);
			cout<<".....push one........"<<v.size()<<endl;
			//copy(v.begin(),v.end(),v.begin());
		}
		cout<<"end"<<endl;
	}
	cin.get();
}