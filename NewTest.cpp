#include <iostream>
#include <string>
#include <memory>
using namespace std;
class M{
public:
	string name;
	M(string a):name(a){
		cout<<a<<endl;
	}
	~M(){
		cout<<"destroy M"<<endl;
	}
	void * operator new(std::size_t size) throw(std::bad_alloc) {
		cout<<"new M "<<endl;
		if (size == 0)size = 1;
		void* p;
		while ((p = ::malloc(size)) == 0) {
			std::new_handler nh = std::get_new_handler();
			if (nh)nh();else throw std::bad_alloc();
		}
		return p;
	}
	void operator delete(void *p){
		if (p){
			::free(p);
		}
	}
	void *operator new (size_t size,void *p){
		cout<<"place M"<<endl;
		return p;//important !!! will not call the construct function if not return p or return 0;
	}
	void print(){cout<<"print M"<<name<<endl;}
};
void NewTestmain(){
//void main(){
	do {	
		cout<<"test"<<endl;
		string str="construct";
		auto p=new M(str);p->print();
		char buf[sizeof(M)];
		new(buf) M(str);
		delete p;
		auto func=[](){
			set_new_handler(nullptr);
			cout<<"memory not enough...."<<endl;
		};
		auto bc=set_new_handler(func);
		try{
			char *p = new char[0x7fffffff];
		}catch(bad_alloc &){
			set_new_handler(bc);
			cout<<"exception happened ..."<<endl;
		}
	} while (false);

	while (true);
}