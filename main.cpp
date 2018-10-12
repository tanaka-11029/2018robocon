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

MotorSerial ms;//Pigpio is the best
DualShock3 Controller;
//DualShock3 LED ("/dev/input/js1",false,0);
//rotaryInc rotary(5,6,false);//仮

int main(void){
	/*if(gpioInitialise() < 0){//msを使っているときは必要ない
	  cout << "pigpio setup error" << endl;
	  return -1;
	  }*/
	Controller.update();
	try{
		ms.init();
	}catch (runtime_error exception){
		cout << "スタートできません。" << endl;
		return -1;
	}
	ms.send(10, 100,0);
	gpioSetMode(26, PI_INPUT);
	gpioSetPullUpDown(26, PI_PUD_UP); 
	gpioSetMode(13, PI_OUTPUT);
	struct timeval mytime;
	bool zone=false;
	bool already = false;
	cout << "キャリブレーション待機" << endl;
	cout << "コート：青" << endl;
	gpioDelay(2000000);
	UPDATELOOP(Controller,!Controller.button(SQUARE) || !Controller.button(RIGHT)){
		gettimeofday(&mytime, NULL);
		gpioWrite(13,mytime.tv_usec/250000%2);
		if(Controller.button(START) && Controller.button(DOWN)){
			gpioWrite(13,0);
			ms.send(10,30,0);
			ms.send(10, 20,1);
			cout << "プログラム終了" <<endl;
			ms.send(255,255,0);
			return 0;
		}
		if(mytime.tv_usec/500000){
			if(already){
				ms.send(10, 101,zone);
			}else{
				ms.send(10,101,2);
			}
		}

		if(Controller.press(TRIANGLE)){//赤
			zone = true;
			already=true;
			ms.send(10, 101,1);
			cout << "コート：赤" << endl;
		}
		if(Controller.press(CIRCLE)){//青
			zone = false;
			already=true;
			ms.send(10,101,1);
			cout << "コート：青" << endl;
		}
	}
	GY521 gyro;
	gyro.start();
	gyro.resetYaw(0);
	cout << "プログラム開始" <<endl;
	double speed =1;
	int xx,yy;
	int right_x,right_y,left_x,left_y,right_t,left_t;
	//	int v1,v2,v3,v4;
	int v[4];
	double theta;
	//	double rf,rb,lf,lb;
	//	int range;
	int a,b,c,d;
	int count=0;
	double omega;
	double last;
	double Yaw;
	bool correct=false;
	bool back = false;
	bool left = false;
	bool right = false;
	bool rear = false;
	bool rearl = false;
	bool rearr = false;
	bool select = false;
	bool emergency = false;
	bool yorokobi = false;
	bool loop = false;
	bool send = false;
	Controller.yReverseSet(true);
	gpioWrite(13, 1);
	UPDATELOOP(Controller,!Controller.button(START) || !Controller.button(DOWN)){
		//LED.update();
		if(Controller.press(SELECT)){
			select = true;
			cout << "プログラム一時停止" <<endl;
			ms.send(255,255,0);//safeoperation
			ms.send(10,104,zone);
			UPDATELOOP(Controller,!Controller.press(START)){
				gettimeofday(&mytime, NULL);//現在時刻取得
				if(mytime.tv_usec/50000){
					if(send){
						loop=true;
					}
				}else if(!send){
					send=true;
				}
				gpioWrite(13,mytime.tv_usec/500000);//0.5秒ごとに点滅
				
				if(gpioRead(26) && loop){
					ms.send(10,102,1);
				}else if(loop){
					if(count){
						ms.send(10,103,count%7);
					}else{
						ms.send(10,104,zone);
					}
				}

				if(Controller.press(UP))count++;
				if(Controller.press(CROSS))count=0;

				if(Controller.button(START) && Controller.button(DOWN)){
					gpioWrite(13,0);
					ms.send(10,30,0);
					ms.send(10, 20,1);
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
		gyro.updata();
		Yaw = gyro.yaw;

		gettimeofday(&mytime, NULL);//現在時刻取得

		if(mytime.tv_usec/50000){
			if(send){
				loop=true;
			}
		}else if(!send){
			send=true;
		}

		if(!yorokobi && loop){//テープLED制御
			cout<<"loop!!";
			if(gpioRead(26)){
				cout << "非常";
				ms.send(10,105,zone);
				emergency = true;
			}else if(!select){
//				select=true;
				emergency=false;
				ms.send(10,106,zone);
			}else{
				ms.send(10,107,zone);
				emergency=false;
			}
			send=false;
			loop=false;
		}

		if(zone){//赤
			if(Controller.press(UP)){
				right = true;
				correct = false;
				back = false;
			}else if(Controller.press(LEFT)){
				correct = true;
				right = false;
				back = false;
			}else if(Controller.press(RIGHT)){
				back = true;
				correct= false;
				right = false;

			}
		}else{//青
			if(Controller.press(UP)){
				left = true;
				back = false;
				correct=false;
			}else if(Controller.press(LEFT)){
				back = true;
				correct=false;
				left=false;
			}else if(Controller.press(RIGHT)){
				correct = true;
				left=false;
				back=false;
			}
		}

		if(Controller.button(DOWN) && Controller.button(R1)){//フラグリセット
			gyro.resetYaw(0);
			rear = false;
			rearr = false;
			rearl = false;
			correct = false;
			right = false;
			left = false;
			back = false;
		}

		if(!rear){
			if(last>170&&Yaw<10){
				rear=true;
				rearl=true;
			}else if(last<-170&&Yaw>-10){
				rear=true;
				rearr=true;
			}/*else if(rearl && last >= 0 && last < 10 && Yaw < 0 && Yaw > -10){
			   rear = true;
			   rearl = false;
			   rearr = false;
			   }else if(rearr && last <= 0 && last > -10 && Yaw > 0 && Yaw < 10){
			   rear=true;
			   rearl = false;
			   rearr = false;
			   }*/
		}else{
			if(last>170&&Yaw<10){
				rear=false;
				rearl=false;
				rearr=false;
			}else if(last<-170&&Yaw>-10){
				rear=false;
				rearl=false;
				rearr=false;
			}else if(last >= 0 && last < 10 && Yaw < 0 && Yaw > -10){
				if(!rearl){
					rear=false;
				}else{
					rearr=true;
					rearl=false;
				}
			}else if(last <= 0 && last > -10 && Yaw > 0 && Yaw < 10){
				if(!rearr){
					rear=false;
				}else{
					rearl=true;
					rearr=false;
				}
			}
		}
		last = Yaw;


		if(/*right_x != 0 || right_y != 0 || */Controller.press(CROSS)) {
			omega = 0;
			correct = false;
			right = false;
			left = false;
			back = false;
			yorokobi=false;
		} else if(Controller.button(R2) || Controller.button(L2)){	
			omega = left_t-right_t;
		}else{
			if(correct){
				cout<<"correct";
				omega=300/M_PI*atan(( rear ? ( Yaw > 0 ? (180-Yaw) : (-180-Yaw)) : (0-Yaw) )*M_PI/70);
				if((!rear && Yaw < 2 && Yaw > -2) || (rear &&( Yaw >178||Yaw < -178)))correct = false;
			}else if(back){
				cout<<"back";
				omega=300/M_PI*atan(( !rear ? ( Yaw > 0 ? (180-Yaw) : (-180-Yaw)) : (0-Yaw))*M_PI/70);
				if((rear && Yaw < 2 && Yaw > -2) || (!rear && (Yaw > 178 || Yaw < -178)))back = false;
			}else if(right){
				cout<<"right";
				omega=300/M_PI*atan(( rear ? ( Yaw > 0 ? 90 : (90+Yaw)) : ( Yaw > 0 ? (-Yaw+90) : 90 ))*M_PI/70);
				if((!rear && Yaw < 92 && Yaw > 88) || (rear && Yaw < -88 && Yaw > -92))right = false;
			}else if(left){
				cout<<"left";
				omega=300/M_PI*atan(( !rear ? ( Yaw >  0 ? -90 : (-90-Yaw)) : ( Yaw > 0) ? 90-Yaw : -90 )*M_PI/70);
				if((rear && Yaw < 92 && Yaw > 88) || (!rear && Yaw < -88 && Yaw > -92))left = false;
			}else{
				omega = left_t-right_t;
			}
		}
		theta=-Yaw*M_PI/180;

		yy=!rear ? -(right_x*cos(theta)-right_y*sin(theta)) : right_x*cos(theta)-right_y*sin(theta);
		xx=!rear ? -(right_x*sin(theta)+right_y*cos(theta)) : right_x*sin(theta)+right_y*cos(theta);

		if(!zone){//青
			v[0]=-xx+yy-omega;//+
			v[1]=xx+yy-omega;//-
			v[2]=-xx+yy+omega;//-
			v[3]=xx+yy+omega;//+
		}else{//赤
			v[0]=xx-yy-omega;//+
			v[1]=-xx-yy-omega;//-
			v[2]=xx-yy+omega;//-
			v[3]=-xx-yy+omega;//+
		}



		/*	v[0]=-right_x+right_y+omega;
			v[1]=right_x+right_y-omega;
			v[2]=-right_x+right_y-omega;
			v[3]=right_x+right_y+omega;*/


		for(int count=0; count < 4 ;count++){
			if(v[count] <-230){
				v[count] = -230;
			}
			if(v[count] >230){
				v[count] = 230;
			}
		}

		if(Controller.button(L1)){
			speed=0.5;
		}

		else{
			speed=1;
		}



		a = ms.send(8,4,-(speed*v[0]));
		b = ms.send(8,3,speed*v[1]);
		c = ms.send(4,3,-speed*v[2]);
		d = ms.send(4,4,(speed*v[3]));

		//cout << a <<"\t"<< b <<"\t"<< c <<"\t"<< d <<"\t"<< Yaw <<"\t"<< omega <<"\t"<< rear <<"\t"<<endl;//モータードライバ返り値表示

		cout<<v[0]<<"\t"<<v[1]<<"\t"<<v[2]<<"\t"<<v[3]<<"\t"<< Yaw <<"\t"<< omega <<"\t"<< rear <<"\t"<<rearr<<"\t"<<rearl<<"\n";

		//time_sleep(0.1);

		/*theta = atan2(-right_y,right_x);//角度計算
		//		cout << -right_y << "\t" << right_x << "\t" << theta*180/M_PI << endl;
		//		theta = theta ? -theta : theta;//角度反転
		range = sqrt(right_x * right_x + right_y * right_y);

		if(theta >= 0 && theta < M_PI_2){
		rf = 4/M_PI * theta - 1;
		rb = 1;
		}else if(theta >= M_PI_2 && theta < M_PI){
		rf = 1;
		rb = -4/M_PI * theta + 3;
		}else if(theta >= -M_PI && theta < -M_PI_2){
		rf = -4/M_PI * theta - 3;
		rb = -1;
		l
		}else if(theta >= -M_PI_2 && theta < 0){
		rf = -1;
		rb = 4/M_PI * theta + 1;
		}

		a = -ms.send(4,2,-rf * range);//ms.send(アドレス,コマンド,PWM値)
		b = -ms.send(4,3,-rb * range);

		if(!Controller.button(TRIANGLE)){//デュアルモード切替
		c = ms.send(8,2,rb * range);
		d = ms.send(8,3,rf * range);
		}else{

		theta = atan2(-left_y,left_x);
		//			theta = theta ? -theta : theta;
		range = sqrt(left_x * left_x + left_y * left_y);

		if(theta >= 0 && theta < M_PI_2){
		lb = 4/M_PI * theta - 1;
		lf = 1;
		}else if(theta >= M_PI_2 && theta < M_PI){
		lb = 1;
		lf = -4/M_PI * theta + 3;
		}else if(theta >= -M_PI && theta < -M_PI_2){
		lb = -4/M_PI * theta - 3;
		lf = -1;
		}else if(theta > -M_PI_2 && theta < 0){
		lb = -1;
		lf = 4/M_PI * theta + 1;
		}
		c = ms.send(8,2,lf * range);
		d = ms.send(8,3,lb * range);

		}*/

		if(Controller.button(TRIANGLE,true)){ //包んでポン
			ms.send(7, 2 ,-230);
			if(!emergency)ms.send(10, 12, 27+zone);
			//cout<<"move1"<<endl;
		}else if(Controller.button(R1) && Controller.button(TRIANGLE)){ //包んでポン
			ms.send(7, 2 ,230);
			if(!emergency)ms.send(10, 12 ,7+zone);
			//cout<<"-move1"<<endl;
		}else{
			ms.send(7, 2 ,0);
			if(!emergency){/*
				if(LED.button(TRIANGLE)){
					success1=true;
					ms.send(10, 12, 7+zone);
				}else if(LED.press(UP)){
					success1 = false;
				}else if(!success1){
					ms.send(10, 12, 17+zone);
				}*/
				ms.send(10,12,17+zone);
			}
			//cout<<"stop1"<<endl;
		}	

		if(Controller.button(SQUARE,true)){ //包んでポン
			ms.send(7, 3 ,230);
			if(!emergency)ms.send(10, 13, 27+zone);
			//cout<<"move2"<<endl;
		}else if(Controller.button(R1) && Controller.button(SQUARE)){ //包んでポン
			ms.send(7, 3 ,-230);
			if(!emergency)ms.send(10, 13, 7+zone);
			//cout<<"-move2"<<endl;
		}else{
			ms.send(7,3,0);
			if(!emergency){/*
				if(LED.button(SQUARE)){
					success2=true;
					ms.send(10, 13, 7+zone);
				}else if(LED.press(LEFT)){
					success2=false;
				}else if(!success2){
					ms.send(10, 13, 17+zone);
				}*/
				ms.send(10 , 13, 17+zone);
			}
		}	

		if(Controller.button(CIRCLE,true)){ //包んでポン
			ms.send(7, 4 ,230);
			if(!emergency)ms.send(10, 15, 27+zone);
			//cout<<"move3"<<endl;
		}else if(Controller.button(R1) && Controller.button(CIRCLE)){ //包んでポン
			ms.send(7, 4 ,-230);
			if(!emergency)ms.send(10, 15, 7+zone);
			//cout<<"-move3"<<endl;
		}else{
			ms.send(7,4,0);
			if(!emergency){/*
				if(LED.button(CIRCLE)){
					success3=true;
					ms.send(10, 15, 7+zone);
				}else if(LED.press(RIGHT)){
					success3 =false;
				}else if(!success3){
					ms.send(10, 15, 17+zone);
				}*/
				ms.send(10,15,17+zone);
			}
		}	

		if(Controller.button(R1) && Controller.button(L1)){
			yorokobi=true;
			emergency=true;
			ms.send(10,109,0);
		}
		//gpioDelay(1000);//プリント関数使用時の重さを回避するため1msのウェイトをかける
	}
	gpioDelay(5000);
	ms.send(10, 30,0);
	ms.send(10, 20,1);
	ms.send(255, 255, 0);//safeoperation
	gpioWrite(13, 0);
	cout << "プログラム終了" <<endl;
	return 0;
}

