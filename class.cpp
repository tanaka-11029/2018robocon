#include<stdio.h>
#include<string.h>
class human{
	private:
	int age;
	char name[20];
	public:
	void nameset(char a[20],int g){
		strcpy(name,a);
		age = g;
	}
	void nameprint(){
		printf("Name:%s\tAge:%d\n",name,age);
	}

};

int baka(int a){
	return a+1;
}

int main(void){
	human member[10];
	member[0].nameset("Osada Uma", 16);
	member[1].nameset("Higashi Yukihiro",16);
	for(int i= 0;i < 2;i++){
		member[i].nameprint();
	}
	return 0;
}
