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
