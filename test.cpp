#include<iostream>
#include<ctime>
#include<pigpiod_if2.h>

using namespace std;

int main(){
	int pi=pigpio_start(0, 0);
	cout << pi << endl;
	set_mode(pi,16,PI_INPUT);
	set_pull_up_down(pi,16,PI_PUD_DOWN);
	int res;
		
	while(1){

		if(gpio_read(pi,16)){
			cout<<"スイッチが押されています"<< endl;
			
		}else{
			cout<<"スイッチが押されていません"<< endl;
		}

		time_sleep(0.1);
	}

		pigpio_stop(pi);
	}

		return 0;

}
