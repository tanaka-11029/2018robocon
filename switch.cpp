#include <iostream>
#include<pigpiod_if2.h>
/*#include "/home/pi/2018robocon/RasPiDS3/RasPiDS3.hpp"*/


using namespace std;
//using namespace RPDS3;

int main(){
	int pi=pigpio_start(NULL,NULL);
	set_mode(pi,16,PI_INPUT);
	set_pull_up_down(pi,16,PI_PUD_DOWN);
	
	istream::int_type ch;
	while((ch =cin.get()) != 'a
			'){
		
		if(gpio_read(pi,16)==1){
			cout<<"スイッチが押されています。\n";
		}
		
		else{
			cout<"スイッチが押されていません。\n";
		}

	}

	pigpio_stop(pi);


	return 0;
}
