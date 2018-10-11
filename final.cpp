#include <iostream>
#include <cmath>
#include <atomic>
#include <cstdio>
#include <ctime>
#include <pigpio.h>
#include <pthread.h>
#include <sys/time.h>
#include <stdlib.h>
#include "/home/pi/2018robocon/PigpioMS/PigpioMS.hpp"
#include "/home/pi/2018robocon/RasPiDS3/RasPiDS3.hpp"
#include "/home/pi/2018robocon/Sensor-master/RotaryInc/RotaryInc.hpp"
#include "/home/pi/2018robocon/Sensor-master/GY521/GY521.hpp"
#include<pigpiod_if2.h>

using namespace std;
using namespace RPMS;
using namespace RPDS3;
using namespace RPGY521;
using namespace std;

//double pid(double ,double);

MotorSerial ms;
DualShock3 Controller;
//rotaryInc rotary(5,6,false);//仮

int main(void){
	int pi=pigpio_start(0, 0);
	set_mode(pi,16,PI_INPUT);
	set_pull_up_down(pi,16,PI_PUD_DOWN);

	Controller.update();
	try{
		ms.init();
	}catch (runtime_error exception){
		cout << "スタートできません。" << endl;
		return -1;
	}
	struct timeval mytime;
	gpioSetMode(13, PI_OUTPUT);
	cout << "プログラム開始" <<endl;
	gpioWrite(13,1);
	int status;
	int right_x,right_y,left_x,left_y,right_t,left_t;
	int v[4];
	double theta;
	unsigned long long int pre_t;
	int a,b,c,d;
	int omega;
	Controller.yReverseSet(true);
	UPDATELOOP(Controller,!Controller.button(START) || !Controller.button(RIGHT)){
		if(Controller.press(SELECT)){
			cout << "プログラム一時停止" <<endl;
			ms.send(255,255,0);//safeoperation
			UPDATELOOP(Controller,!Controller.press(START)){
				gettimeofday(&mytime, NULL);//現在時刻取得
				gpioWrite(13,mytime.tv_usec/500000);//0.5秒ごとに点滅
				if(Controller.button(START) && Controller.button(RIGHT)){
					gpioWrite(13,0);
					cout << "プログラム終了" <<endl;
					return 0;
				}
			}
			cout << "プログラム再開" <<endl;
			gpioWrite(13,1);
		}
		right_x = Controller.stick(RIGHT_X);//アナログ値代入
		right_y = Controller.stick(RIGHT_Y);
		right_t = Controller.stick(RIGHT_T);
		left_x = Controller.stick(LEFT_X);
		left_y = Controller.stick(LEFT_Y);
		left_t = Controller.stick(LEFT_T);

		omega=right_t-left_t;


		v[0]=-right_x+right_y+omega;
		v[1]=right_x+right_y-omega;
		v[2]=-right_x+right_y-omega;
		v[3]=right_x+right_y+omega;


		for(int count=0;count < 4;count++){
			if(v[count] <-250){
				v[count] = -250;
			}
		}

		for(int num=0; num<4; num++){
			if(v[num] >250){
				v[num] = 250;
			}
		}

		a = ms.send(4,2,-v[0]);
		b = ms.send(8,2,v[1]);
		c = ms.send(8,3,v[2]);
		d = ms.send(4,3,-v[3]);

		cout << a <<"\t"<< b <<"\t"<< c <<"\t"<< d <<"\t"<< endl;//モータードライバ返り値表示

		if(Controller.pressed(CIRCLE)){//包んでポン
			ms.send(7, 10, 255);
			if(gpio_read(pi,16)){
				ms.send(7,10,255);
				break;
			}



	}
	ms.send(255,255,0);//safeoperation
	gpioWrite(13,0);
	pigpio_stop(pi);
	cout << "プログラム終了" <<endl;
	return 0;
}


