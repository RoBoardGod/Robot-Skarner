#include <stdio.h>
#include <stdlib.h> 
#include <time.h>
#include <math.h>
#include "roboard.h"
#include "conio.h"
#include "windows.h"
#include <string.h>
#include <irrKlang.h>
using namespace irrklang;
#define RAD_TO_DEG   (57.2957795147199)     
#define DEG_TO_RAD   (0.01745329251944)     
#define DEG_TO_PWM   (9.23076923076923)
struct point{
	double x,y,z;
};
char* sound[15] = {"skarner_attack1.mp3","skarner_attack3.mp3","skarner_attack5.mp3","skarner_attack6.mp3","skarner_attack7.mp3",
					"skarner_dying4.mp3","skarner_laugh1.mp3","skarner_laugh4.mp3","skarner_move1.mp3","skarner_move2.mp3",
					"skarner_move3.mp3","skarner_move4.mp3","skarner_move5.mp3","skarner_move6.mp3","skarner_move9.mp3"};
class CMyFileFactory : public irrklang::IFileFactory
{
public:
 virtual irrklang::IFileReader* createFileReader(const ik_c8* filename)
 {
  FILE* file = fopen((char*)filename, "rb");
  if (!file) return 0;

  return new CMyReadFile(file, filename);
 }

protected:
 class CMyReadFile : public irrklang::IFileReader
 {
 public:
  CMyReadFile(FILE* openedFile, const ik_c8* filename)
  {
   File = openedFile;
   strcpy(Filename, filename);
   long cur_offset, end_offset;
   cur_offset = ftell(openedFile);
   fseek(openedFile, 0L, SEEK_END);
   end_offset = ftell(openedFile);
   fseek(openedFile, cur_offset, SEEK_SET);
   FileSize = (ik_s32)end_offset;
  }

  ~CMyReadFile()
  {
   fclose(File);
  }

  ik_s32 read(void* buffer, ik_u32 sizeToRead)
  {
   return (ik_s32)fread(buffer, 1, sizeToRead, File);
  }

  bool seek(ik_s32 finalPos, bool relativeMovement)
  {
   return fseek(File, finalPos, relativeMovement ? SEEK_CUR : SEEK_SET) == 0;
  }

  ik_s32 getSize()
  {
   return FileSize;
  }

  ik_s32 getPos()
  {
   return ftell(File);
  }

  const ik_c8* getFileName()
  {
   return Filename;
  }

  FILE* File;
  char Filename[1024];
  ik_s32 FileSize;

 }; //end class CMyReadFile
}; //end class CMyFileFactory

unsigned long center[32] = { 1600, 1530, 1530, 
							 1450, 1430, 1500, 
							 1490, 1560, 1500,  
							 1450, 1470, 1520,  
							 1590, 1470, 1550,  
							 1490, 1460, 1520,
							 2100, 1850,  900,
							 1150, 1500, 2200,
							 0,0,0,
							 0,0,0,
							 0,0};
/*unsigned long home_frame[32] = { 1600, 1530, 1530, 
							 1450, 1430, 1500, 
							 1490, 1560, 1500,  
							 1450, 1470, 1520,  
							 1590, 1470, 1550,  
							 1490, 1460, 1520,
							 2100, 1850,  900,
							 1150, 1500, 2200,
							 0,0,0,
							 0,0,0,
							 0,0};*/
unsigned long home_frame[32] = {1670, 2230, 1520, 
							 1550, 2100, 1520, 
							 1410, 880, 1500,  
							 1390, 770, 1500,  
							 1500, 770, 1530,  
							 1570, 2160, 1500,
							 2100, 1000,  900,
							 2000, 1500, 2200,
							 0,0,0,
							 0,0,0,
							 0,0};
int direction[18] = {-1, 1,-1,
					 -1, 1,-1,
					  1,-1, 1,
				      1,-1, 1,
					  1,-1, 1,
					 -1, 1,-1};
double L1 = 3.965, L2 = 6.870, L3 = 18.809;
bool forword = true;
int moveleg = 0;
int movelegmap[6] = {2,4,5,1,0,3};
double alldegree[18] = {0};
unsigned long motion_frame[32] = {0};
int tailshake = 0;
bool tailforword = true;
double body_pitch = 0.0,body_roll = 0.0;
//hand action
int mode = 0;
int walkmode = 0;
bool swingmode = false;
double swingcount = 0;
void home(){
    tailshake = 0;
	motion_frame[18] = 2100;
	motion_frame[19] = 1250;
	motion_frame[20] = 900;
	motion_frame[21] = 1800;
	motion_frame[22] = 1500;
	motion_frame[23] = 1500;
}
void righthand(){
	tailshake = 0;
	motion_frame[18] = 2100;
	motion_frame[19] = 1250;
	motion_frame[20] = 1400;
	motion_frame[21] = 1700;
	motion_frame[23] = 1500;
}
void lefthand(){
	tailshake = 0;
	motion_frame[18] = 1530;
	motion_frame[19] = 1300;
	motion_frame[20] = 900;
	motion_frame[21] = 1800;
	motion_frame[23] = 1500;
}
void bothhand(){
	tailshake = 0;
	motion_frame[18] = 1530;
	motion_frame[19] = 1300;
	motion_frame[20] = 1400;
	motion_frame[21] = 1700;
	motion_frame[23] = 1500;
}
void tail(int val){
	motion_frame[18] = 2100;
	motion_frame[19] = 1250;
	motion_frame[20] = 900;
	motion_frame[21] = 1800;
	motion_frame[22] = 1500 + val;
	motion_frame[23] = 1500;
}
void tail_atk(){
	tailshake = 0;
	motion_frame[18] = 2100;
	motion_frame[19] = 1250;
	motion_frame[20] = 900;
	motion_frame[21] = 1800;
	motion_frame[22] = 1500;
	motion_frame[23] = 2050;
}
void laugh(){
	tailshake = 0;
	motion_frame[18] = 1530;
	motion_frame[19] = 1300;
	motion_frame[20] = 1400;
	motion_frame[21] = 1700;
	motion_frame[23] = 1800;
}



double LtoD(double Line1,double Line2,double other){
	return acos((pow(Line1,2) + pow(Line2,2) - pow(other,2))/(2*Line1*Line2));
}
void InverseKinematics(point* fp,double degree[3]){
	double fx,fy;
	double vx,vy,vd;
	double vL;
	degree[2] = atan(fp->z/fp->x);
	fx = ((fp->x - L1) * cos(-degree[2]) + fp->z * -sin(-degree[2]));
	fy = fp->y;
	degree[2] = degree[2] * RAD_TO_DEG;
	
	vL = sqrt(pow(fx,2) + pow(fy,2));
	degree[0] = LtoD(L2,L3,vL) * RAD_TO_DEG - 90;
	
	vd = DEG_TO_RAD * 180 - LtoD(vL,L3,L2);
	vx = cos(vd) * L3 + vL;
	vy = sin(vd) * L3;
	vd = atan(fy/fx);
	fx = cos(vd) * vx ;
	fy = sin(vd) * vx;
	vx = fx - (sin(vd) * vy);
	vy = fy + (cos(vd) * vy);
	degree[1] = atan(vy/vx)  * RAD_TO_DEG;
}
void InverseKinematics2(point* fp,double degree[3]){
	double fx,fy;
	degree[2] = atan(fp->z/fp->x);
	fx = (fp->x * cos(-degree[2]) + fp->z * sin(-degree[2])) - L1;
	fy = fp->y;	
	degree[2] = degree[2] * RAD_TO_DEG;
	
	double a, b, c, temp1,temp2, temp3, root;
	a = -2.0 * L2 * fy;
	b = 2.0 * L2 * fx;
	c = L3*L3 - L2*L2 - (fx)*(fx) - (fy)*(fy);
	temp1 = a*c;
	temp2 = sqrt(a*a*c*c - (a*a+b*b)*(c*c-b*b));
	temp3 = a*a+b*b;

	root = (temp1+ temp2)/temp3;
	degree[1] = asin(root)*RAD_TO_DEG;

	temp1 = 1.0 - root*root;
	temp1 = sqrt(temp1);
	root = (fx-L2*temp1)/L3;
	degree[0] = asin(root)*RAD_TO_DEG;
}

#define onRB
int main(){
while(true){
	if(getch() == 27){
	    goto restart;
	}
}

restart:
#ifdef onRB
	roboio_SetRBVer(RB_100RD);
	for(int i = RCSERVO_PINS1; i <= RCSERVO_PINS24; i++){
		rcservo_SetServo(i, RCSERVO_DMP_RS1270);
	}
	unsigned long usedchannel = 0;
	for(int i = 0; i < 24; i++)
		usedchannel = usedchannel + (RCSERVO_USECHANNEL0 << i);	//set usepins
	rcservo_Init(usedchannel);
	rcservo_EnterPlayMode_HOME(home_frame);
#endif
	ISoundEngine* audioEngine = createIrrKlangDevice(ESOD_AUTO_DETECT, ESEO_MULTI_THREADED | ESEO_LOAD_PLUGINS);
	 if (audioEngine){
		CMyFileFactory* demofileFactory = new CMyFileFactory();
		audioEngine->addFileFactory(demofileFactory);
		demofileFactory->drop();
	 } 
	double r = 3;
	double speed = 45;
	point* p[6];
	for(int i = 0; i < 6; i++)
		p[i] = (point*)malloc(sizeof(point));

	int mode = 0;
	double count = 90;
	double count2 = 90;
	double count3 = 90;

	point* dp = (point*)malloc(sizeof(point));
	
	double shift_x = 0;
	double shift_y = 3.0;
	
	dp->x = L1+L2+shift_x;
	dp->y = -L3+shift_y;
	dp->z = 0;
	double open1 = 13.0;
	double open2 = 5.0;
	double open3 = 6.0;
	bool flag = true;
	//wake up
	
				//後
					p[1]->x = dp->x;
					p[1]->y = dp->y;
					p[1]->z = dp->z - open1;
					p[2]->x = dp->x;
					p[2]->y = dp->y;
					p[2]->z = dp->z - open1;
				//中
					p[0]->x = dp->x + open2;
					p[0]->y = dp->y;
					p[0]->z = dp->z;
					p[3]->x = dp->x + open2;
					p[3]->y = dp->y;
					p[3]->z = dp->z;
				//前
					p[4]->x = dp->x;
					p[4]->y = dp->y;
					p[4]->z = dp->z + open3;
					p[5]->x = dp->x;
					p[5]->y = dp->y;
					p[5]->z = dp->z + open3;

			for(int i = 0; i < 6; i++)
				InverseKinematics(p[i],&alldegree[i*3]);
			for(int i = 0; i < 18; i++)
				motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
				home();
#ifdef onRB
				rcservo_SetAction(motion_frame, 5000);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
#endif
	srand (time(NULL));
	while(flag){
		if(swingmode){
			swingcount = (int)(swingcount+(speed/2)) % 360;
			body_pitch = sin(swingcount * DEG_TO_RAD) * 2.0;
		    body_roll = cos(swingcount * DEG_TO_RAD) * 2.0;
		}
		if(kbhit()){
			switch(getch()){
				case 27:
					flag = false;
					break;
				case 'w': 
					if(walkmode == 0){
							//後
							if(forword){
								p[1]->x = dp->x;
								p[1]->y = dp->y + body_pitch - body_roll;
								p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
								p[2]->x = dp->x;
								p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
								p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
							}else{
								p[1]->x = dp->x;
								p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
								p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
								p[2]->x = dp->x;
								p[2]->y = dp->y + body_pitch + body_roll;
								p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
							}
							//中
							if(forword){
								p[0]->x = dp->x + open2;
								p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
								p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
								p[3]->x = dp->x + open2;
								p[3]->y = dp->y + body_roll;
								p[3]->z = dp->z + r-(2 * r * (count/180));
							}else{
								p[0]->x = dp->x + open2;
								p[0]->y = dp->y - body_roll;
								p[0]->z = dp->z + r-(2 * r * (count/180));
								p[3]->x = dp->x + open2;
								p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
								p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
							}
							//前
							if(forword){
								p[5]->x = dp->x;
								p[5]->y = dp->y - body_pitch - body_roll;
								p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
								p[4]->x = dp->x;
								p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
								p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
							}else{
								p[5]->x = dp->x;
								p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
								p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
								p[4]->x = dp->x;
								p[4]->y = dp->y - body_pitch + body_roll;
								p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
							}
						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);

						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

						if(mode != 1){
							count = 0;
							count2 = 0;
							count3 = 0;

							mode = 1;
							if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
						}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
						
						}
						count+=speed;
						if(count >= 180){
							forword = !forword;
							count = 0;
						}
					}else if(walkmode == 1){
						//後
							p[1]->x = dp->x;
							p[1]->y = dp->y;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x;
							p[2]->y = dp->y;
							p[2]->z = dp->z - open1;
						//中
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y;
							p[0]->z = dp->z;
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y;
							p[3]->z = dp->z;
						//前
							p[5]->x = dp->x;
							p[5]->y = dp->y;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x;
							p[4]->y = dp->y;
							p[4]->z = dp->z + open3;
							
						

						for(int i = 0; i <= moveleg; i++)
							p[moveleg-i]->z = p[moveleg-i]->z  + r-(2 * r * ((count+(i*180))/900));
						//要移動的腳
							p[moveleg]->x = dp->x + (moveleg==0 || moveleg==3 ? open2 : 0);
							p[moveleg]->y = dp->y + sin(count * DEG_TO_RAD) * r;
							p[moveleg]->z = dp->z - cos(count * DEG_TO_RAD) * r - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
						for(int i = moveleg+1; i < 6; i++)
							p[i]->z = p[i]->z  + r-(2 * r * ((count+((6-i+moveleg)*180))/900));
						

						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);

						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

						if(mode != 1){
							count = 0;
							count2 = 0;
							count3 = 0;

							mode = 1;
							if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
						}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
						
						}
						count+=speed;
						if(count >= 180){
							forword = !forword;
							moveleg = moveleg < 5 ? moveleg + 1 : 0;
							count = 0;
						}
					}else if(walkmode == 2){
						//後
							p[1]->x = dp->x;
							p[1]->y = dp->y;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x;
							p[2]->y = dp->y;
							p[2]->z = dp->z - open1;
						//中
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y;
							p[0]->z = dp->z;
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y;
							p[3]->z = dp->z;
						//前
							p[5]->x = dp->x;
							p[5]->y = dp->y;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x;
							p[4]->y = dp->y;
							p[4]->z = dp->z + open3;
							
						
						p[moveleg]->x = dp->x + (moveleg==0 || moveleg==3 ? open2 : 0);
						p[moveleg]->y = dp->y + sin((count/2) * DEG_TO_RAD) * r;
						p[moveleg]->z = dp->z - cos((count/2) * DEG_TO_RAD) * r - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
						
						int nextleg = movelegmap[moveleg];
						for(int i = 3; i >=0; i--){
							p[nextleg]->z = p[nextleg]->z  + r-(2 * r * (count+(180*i))/720);
							nextleg = movelegmap[nextleg];
						}
						p[nextleg]->x = dp->x + (nextleg==0 || nextleg==3 ? open2 : 0);
						p[nextleg]->y = dp->y + sin((count/2+90) * DEG_TO_RAD) * r;
						p[nextleg]->z = dp->z - cos((count/2+90) * DEG_TO_RAD) * r - (nextleg==1 || nextleg==2 ? open1 : 0) + (nextleg==4 || nextleg==5 ? open3 : 0);
						

						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);

						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

						if(mode != 1){
							count = 0;
							count2 = 0;
							count3 = 0;

							mode = 1;
							if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
						}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
						
						}
						count=count + speed*2;
						if(count >= 180){
							forword = !forword;
							if(moveleg == 1)
								moveleg = 4;
							else if(moveleg == 4)
								moveleg = 0;
							else if(moveleg == 0)
								moveleg = 2;
							else if(moveleg == 2)
								moveleg = 5;
							else if(moveleg == 5)
								moveleg = 3;
							else if(moveleg == 3)
								moveleg = 1;
							count = 0;
						}
					}
					break;
				case 's':
					if(walkmode == 0){
							
						//後
						if(forword){
							p[1]->x = dp->x;
							p[1]->y = dp->y + body_pitch - body_roll;
							p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
							p[2]->x = dp->x;
							p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
							p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
						}else{
							p[1]->x = dp->x;
							p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
							p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
							p[2]->x = dp->x;
							p[2]->y = dp->y + body_pitch + body_roll;
							p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
						}
						//中
						if(forword){
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
							p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y + body_roll;
							p[3]->z = dp->z + r-(2 * r * (count/180));
						}else{
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y - body_roll;
							p[0]->z = dp->z + r-(2 * r * (count/180));
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
							p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						}
						//前
						if(forword){
							p[5]->x = dp->x;
							p[5]->y = dp->y - body_pitch - body_roll;
							p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
							p[4]->x = dp->x;
							p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
							p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						}else{
							p[5]->x = dp->x;
							p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
							p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
							p[4]->x = dp->x;
							p[4]->y = dp->y - body_pitch + body_roll;
							p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
						}
						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);

						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

						if(mode != 1){
							count = 0;
							count2 = 0;
							count3 = 0;

							mode = 1;
							if(audioEngine)
								audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
							#ifdef onRB
								rcservo_SetAction(motion_frame, 250);
								while (rcservo_PlayAction() != RCSERVO_PLAYEND);
							#else
								system("cls");
								for(int i = 0; i < 6; i++){
									for(int j = 0; j < 3; j++){
										printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
									}
									printf("\n");
								}
								printf("%.0f",count);
							#endif
						}else{
							#ifdef onRB
								rcservo_SetAction(motion_frame, 100);
								while (rcservo_PlayAction() != RCSERVO_PLAYEND);
							#else
								system("cls");
								for(int i = 0; i < 6; i++){
									for(int j = 0; j < 3; j++){
										printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
									}
									printf("\n");
								}
								printf("%.0f",count);
							#endif
						
						}
						count-=speed;
						if(count <= 0){
							forword = !forword;
							count = 180;
						}
					}else if(walkmode == 1){
						//後
							p[1]->x = dp->x;
							p[1]->y = dp->y;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x;
							p[2]->y = dp->y;
							p[2]->z = dp->z - open1;
						//中
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y;
							p[0]->z = dp->z;
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y;
							p[3]->z = dp->z;
						//前
							p[5]->x = dp->x;
							p[5]->y = dp->y;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x;
							p[4]->y = dp->y;
							p[4]->z = dp->z + open3;
							
						for(int i = 0; i <= moveleg; i++)
							p[moveleg-i]->z = p[moveleg-i]->z + r-(2 * r * ((800-(180-count+(i*180)))/900));
						//要移動的腳
							p[moveleg]->x = dp->x + (moveleg==0 || moveleg==3 ? open2 : 0);
							p[moveleg]->y = dp->y + sin(count * DEG_TO_RAD) * r;
							p[moveleg]->z = dp->z - cos(count * DEG_TO_RAD) * r - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
						for(int i = moveleg+1; i < 6; i++)
							p[i]->z = p[i]->z  + r-(2 * r * ((900-(180-count+((6-i+moveleg)*180)))/900));
						
						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);

						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

						if(mode != 1){
							count = 0;
							count2 = 0;
							count3 = 0;

							mode = 1;
							if(audioEngine)
								audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
							#ifdef onRB
								rcservo_SetAction(motion_frame, 250);
								while (rcservo_PlayAction() != RCSERVO_PLAYEND);
							#else
								system("cls");
								for(int i = 0; i < 6; i++){
									for(int j = 0; j < 3; j++){
										printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
									}
									printf("\n");
								}
								printf("%.0f",count);
							#endif
						}else{
							#ifdef onRB
								rcservo_SetAction(motion_frame, 100);
								while (rcservo_PlayAction() != RCSERVO_PLAYEND);
							#else
								system("cls");
								for(int i = 0; i < 6; i++){
									for(int j = 0; j < 3; j++){
										printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
									}
									printf("\n");
								}
								printf("%.0f",count);
							#endif
						
						}
						count-=speed;
						if(count <= 0){
							forword = !forword;
							moveleg = moveleg < 5 ? moveleg + 1 : 0;
							count = 180;
						}
					}else if(walkmode == 2){
						//後
							p[1]->x = dp->x;
							p[1]->y = dp->y;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x;
							p[2]->y = dp->y;
							p[2]->z = dp->z - open1;
						//中
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y;
							p[0]->z = dp->z;
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y;
							p[3]->z = dp->z;
						//前
							p[5]->x = dp->x;
							p[5]->y = dp->y;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x;
							p[4]->y = dp->y;
							p[4]->z = dp->z + open3;
							

						p[moveleg]->x = dp->x + (moveleg==0 || moveleg==3 ? open2 : 0);
						p[moveleg]->y = dp->y + sin((count/2+90) * DEG_TO_RAD) * r;
						p[moveleg]->z = dp->z - cos((count/2+90) * DEG_TO_RAD) * r - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
						
						int nextleg = movelegmap[moveleg];
						for(int i = 3; i >=0; i--){
							p[nextleg]->z = p[nextleg]->z  + r-(2 * r * (720-(180-count+(180*i)))/720);
							nextleg = movelegmap[nextleg];
						}
						p[nextleg]->x = dp->x + (nextleg==0 || nextleg==3 ? open2 : 0);
						p[nextleg]->y = dp->y + sin((count/2) * DEG_TO_RAD) * r;
						p[nextleg]->z = dp->z - cos((count/2) * DEG_TO_RAD) * r - (nextleg==1 || nextleg==2 ? open1 : 0) + (nextleg==4 || nextleg==5 ? open3 : 0);
						
						
						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);

						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

						if(mode != 1){
							count = 0;
							count2 = 0;
							count3 = 0;

							mode = 1;
							if(audioEngine)
								audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
							#ifdef onRB
								rcservo_SetAction(motion_frame, 250);
								while (rcservo_PlayAction() != RCSERVO_PLAYEND);
							#else
								system("cls");
								for(int i = 0; i < 6; i++){
									for(int j = 0; j < 3; j++){
										printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
									}
									printf("\n");
								}
								printf("%.0f",count);
							#endif
						}else{
							#ifdef onRB
								rcservo_SetAction(motion_frame, 100);
								while (rcservo_PlayAction() != RCSERVO_PLAYEND);
							#else
								system("cls");
								for(int i = 0; i < 6; i++){
									for(int j = 0; j < 3; j++){
										printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
									}
									printf("\n");
								}
								printf("%.0f",count);
							#endif
						
						}
						count=count-speed*2;
						if(count <= 0){
							forword = !forword;
							if(moveleg == 1)
								moveleg = 4;
							else if(moveleg == 4)
								moveleg = 0;
							else if(moveleg == 0)
								moveleg = 2;
							else if(moveleg == 2)
								moveleg = 5;
							else if(moveleg == 5)
								moveleg = 3;
							else if(moveleg == 3)
								moveleg = 1;
							count = 180;
						}
					}
				break;
				case 'a': 
					if(walkmode == 0){
						//後
						if(forword){
							p[1]->x = dp->x + cos(speed * DEG_TO_RAD) * (r - (2 * r * ((180-count3)/180)));
							p[1]->y = dp->y + body_pitch - body_roll;
							p[1]->z = dp->z + sin(speed * DEG_TO_RAD) * (r - (2 * r * ((180-count3)/180))) - open1;
							p[2]->x = dp->x + cos(speed * DEG_TO_RAD) * (-cos(count3 * DEG_TO_RAD) * r);
							p[2]->y = dp->y + sin(count3 * DEG_TO_RAD) * r + body_pitch + body_roll;
							p[2]->z = dp->z + sin(speed * DEG_TO_RAD) * (-cos(count3 * DEG_TO_RAD) * r) - open1;
						}else{
							p[1]->x = dp->x + cos(speed * DEG_TO_RAD) * (cos(count3 * DEG_TO_RAD) * r);
							p[1]->y = dp->y + sin(count3 * DEG_TO_RAD) * r + body_pitch - body_roll;
							p[1]->z = dp->z + sin(speed * DEG_TO_RAD) * (cos(count3 * DEG_TO_RAD) * r) - open1;
							p[2]->x = dp->x + cos(speed * DEG_TO_RAD) * (r-(2 * r * (count3/180)));
							p[2]->y = dp->y + body_pitch + body_roll;
							p[2]->z = dp->z + sin(speed * DEG_TO_RAD) * (r-(2 * r * (count3/180))) - open1;
						}
						//中
						if(forword){
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y + sin(count3 * DEG_TO_RAD) * r - body_roll;
							p[0]->z = dp->z - (-cos(count3 * DEG_TO_RAD)*r);
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y + body_roll;
							p[3]->z = dp->z - (r-(2 * r * ((180-count3)/180)));
						}else{
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y - body_roll;
							p[0]->z = dp->z - (r-(2 * r * (count3/180)));
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y + sin(count3 * DEG_TO_RAD) * r + body_roll;
							p[3]->z = dp->z - (-cos((180-count3) * DEG_TO_RAD) * r);
						}
						//前
						if(forword){
							p[5]->x = dp->x - cos(-speed * DEG_TO_RAD) * (r-(2 * r * ((180-count3)/180)));
							p[5]->y = dp->y - body_pitch - body_roll;
							p[5]->z = dp->z - sin(-speed * DEG_TO_RAD) * (r-(2 * r * ((180-count3)/180))) + open3;
							p[4]->x = dp->x - cos(-speed * DEG_TO_RAD) * (-cos(count3 * DEG_TO_RAD) * r);
							p[4]->y = dp->y + sin(count3 * DEG_TO_RAD) * r - body_pitch + body_roll;
							p[4]->z = dp->z - sin(-speed * DEG_TO_RAD) * (-cos(count3 * DEG_TO_RAD) * r) + open3;
						}else{
							p[5]->x = dp->x - cos(-speed * DEG_TO_RAD) * (-cos((180-count3) * DEG_TO_RAD) * r);
							p[5]->y = dp->y + sin(count3 * DEG_TO_RAD) * r - body_pitch - body_roll;
							p[5]->z = dp->z - sin(-speed * DEG_TO_RAD) * (-cos((180-count3) * DEG_TO_RAD) * r) + open3;
							p[4]->x = dp->x - cos(-speed * DEG_TO_RAD) * (r-(2 * r * (count3/180)));
							p[4]->y = dp->y - body_pitch + body_roll;
							p[4]->z = dp->z - sin(-speed * DEG_TO_RAD) * (r-(2 * r * (count3/180))) + open3;
						}
					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 2){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 2;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count3+=speed;
					if(count3 >= 180){
						forword = !forword;
						count3 = 0;
					}
				}else if(walkmode == 1){
					
						
						//後
							p[1]->x = dp->x;
							p[1]->y = dp->y;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x;
							p[2]->y = dp->y;
							p[2]->z = dp->z - open1;
						//中
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y;
							p[0]->z = dp->z;
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y;
							p[3]->z = dp->z;
						//前
							p[5]->x = dp->x;
							p[5]->y = dp->y;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x;
							p[4]->y = dp->y;
							p[4]->z = dp->z + open3;

						for(int i = 0; i <= moveleg; i++)
							if(moveleg-i >= 2 && moveleg-i <=4)
								p[moveleg-i]->z = p[moveleg-i]->z + r-(2 * r * ((900-(count3+(i*180)-180))/900));
							else
								p[moveleg-i]->z = p[moveleg-i]->z  + r-(2 * r * ((count3+(i*180))/900));
						//要移動的腳
							p[moveleg]->x = dp->x + (moveleg==0 || moveleg==3 ? open2 : 0);
							p[moveleg]->y = dp->y + sin(count3 * DEG_TO_RAD) * r;
							p[moveleg]->z = dp->z - cos(count3 * DEG_TO_RAD) * r * (moveleg>= 2 && moveleg<=4 ? -1 : 1) - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
						for(int i = moveleg+1; i < 6; i++)
							if(i >= 2 && i <=4)
								p[i]->z = p[i]->z  + r-(2 * r * ((900-(count3+((6-i+moveleg)*180-180)))/900));
							else
								p[i]->z = p[i]->z  + r-(2 * r * ((count3+((6-i+moveleg)*180))/900));


					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 2){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 2;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count3+=speed;
					if(count3 >= 180){
						forword = !forword;
						moveleg = moveleg < 5 ? moveleg + 1 : 0;
						count3 = 0;
					}
				}else if(walkmode == 2){
				
						
						//後
							p[1]->x = dp->x;
							p[1]->y = dp->y;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x;
							p[2]->y = dp->y;
							p[2]->z = dp->z - open1;
						//中
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y;
							p[0]->z = dp->z;
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y;
							p[3]->z = dp->z;
						//前
							p[5]->x = dp->x;
							p[5]->y = dp->y;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x;
							p[4]->y = dp->y;
							p[4]->z = dp->z + open3;

					p[moveleg]->x = dp->x + (moveleg==0 || moveleg==3 ? open2 : 0);
					p[moveleg]->y = dp->y + sin(((moveleg >= 2 && moveleg <=4 ? count3 : 180-count3)/2+90*(moveleg >= 2 && moveleg <=4 ? 0:1)) * DEG_TO_RAD) * r;
					p[moveleg]->z = dp->z - cos(((moveleg >= 2 && moveleg <=4 ? count3 : 180-count3)/2+90*(moveleg >= 2 && moveleg <=4 ? 0:1)) * DEG_TO_RAD) * r - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
					
					int nextleg = movelegmap[moveleg];
					for(int i = 3; i >=0; i--){
						if(nextleg >= 2 && nextleg <=4)
							p[nextleg]->z = p[nextleg]->z  + r-(2 * r * (count3+(180*i)-180)/720);
						else
							p[nextleg]->z = p[nextleg]->z  + r-(2 * r * (720-(count3+(180*i)-180))/720);

						nextleg = movelegmap[nextleg];
					}
					p[nextleg]->x = dp->x + (nextleg==0 || nextleg==3 ? open2 : 0);
					p[nextleg]->y = dp->y + sin(((nextleg >= 2 && nextleg <=4 ? count3 : 180-count3)/2+90*(nextleg >= 2 && nextleg <=4 ? 1:0)) * DEG_TO_RAD) * r;
					p[nextleg]->z = dp->z - cos(((nextleg >= 2 && nextleg <=4 ? count3 : 180-count3)/2+90*(nextleg >= 2 && nextleg <=4 ? 1:0)) * DEG_TO_RAD) * r - (nextleg==1 || nextleg==2 ? open1 : 0) + (nextleg==4 || nextleg==5 ? open3 : 0);
						

					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 2){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 2;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count3=count3+speed*2;
					if(count3 >= 180){
						forword = !forword;
						if(moveleg == 1)
							moveleg = 4;
						else if(moveleg == 4)
							moveleg = 0;
						else if(moveleg == 0)
							moveleg = 2;
						else if(moveleg == 2)
							moveleg = 5;
						else if(moveleg == 5)
							moveleg = 3;
						else if(moveleg == 3)
							moveleg = 1;
						count3 = 0;
					}
				}
				break;
				case 'd': 
					if(walkmode == 0){
						//後
						if(forword){
							p[1]->x = dp->x + cos(speed * DEG_TO_RAD) * (r - (2 * r * ((180-count3)/180)));
							p[1]->y = dp->y + body_pitch - body_roll;
							p[1]->z = dp->z + sin(speed * DEG_TO_RAD) * (r - (2 * r * ((180-count3)/180))) - open1;
							p[2]->x = dp->x + cos(speed * DEG_TO_RAD) * (-cos(count3 * DEG_TO_RAD) * r);
							p[2]->y = dp->y + sin(count3 * DEG_TO_RAD) * r + body_pitch + body_roll;
							p[2]->z = dp->z + sin(speed * DEG_TO_RAD) * (-cos(count3 * DEG_TO_RAD) * r) - open1;
						}else{
							p[1]->x = dp->x + cos(speed * DEG_TO_RAD) * (cos(count3 * DEG_TO_RAD) * r);
							p[1]->y = dp->y + sin(count3 * DEG_TO_RAD) * r + body_pitch - body_roll;
							p[1]->z = dp->z + sin(speed * DEG_TO_RAD) * (cos(count3 * DEG_TO_RAD) * r) - open1;
							p[2]->x = dp->x + cos(speed * DEG_TO_RAD) * (r-(2 * r * (count3/180)));
							p[2]->y = dp->y + body_pitch + body_roll;
							p[2]->z = dp->z + sin(speed * DEG_TO_RAD) * (r-(2 * r * (count3/180))) - open1;
						}
						//中
						if(forword){
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y + sin(count3 * DEG_TO_RAD) * r - body_roll;
							p[0]->z = dp->z - (-cos(count3 * DEG_TO_RAD)*r);
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y + body_roll;
							p[3]->z = dp->z - (r-(2 * r * ((180-count3)/180)));
						}else{
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y - body_roll;
							p[0]->z = dp->z - (r-(2 * r * (count3/180)));
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y + sin(count3 * DEG_TO_RAD) * r + body_roll;
							p[3]->z = dp->z - (-cos((180-count3) * DEG_TO_RAD) * r);
						}
						//前
						if(forword){
							p[5]->x = dp->x - cos(-speed * DEG_TO_RAD) * (r-(2 * r * ((180-count3)/180)));
							p[5]->y = dp->y - body_pitch - body_roll;
							p[5]->z = dp->z - sin(-speed * DEG_TO_RAD) * (r-(2 * r * ((180-count3)/180))) + open3;
							p[4]->x = dp->x - cos(-speed * DEG_TO_RAD) * (-cos(count3 * DEG_TO_RAD) * r);
							p[4]->y = dp->y + sin(count3 * DEG_TO_RAD) * r - body_pitch + body_roll;
							p[4]->z = dp->z - sin(-speed * DEG_TO_RAD) * (-cos(count3 * DEG_TO_RAD) * r) + open3;
						}else{
							p[5]->x = dp->x - cos(-speed * DEG_TO_RAD) * (-cos((180-count3) * DEG_TO_RAD) * r);
							p[5]->y = dp->y + sin(count3 * DEG_TO_RAD) * r - body_pitch - body_roll;
							p[5]->z = dp->z - sin(-speed * DEG_TO_RAD) * (-cos((180-count3) * DEG_TO_RAD) * r) + open3;
							p[4]->x = dp->x - cos(-speed * DEG_TO_RAD) * (r-(2 * r * (count3/180)));
							p[4]->y = dp->y - body_pitch + body_roll;
							p[4]->z = dp->z - sin(-speed * DEG_TO_RAD) * (r-(2 * r * (count3/180))) + open3;
						}
					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 2){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 2;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count3-=speed;
					if(count3 <= 0){
						forword = !forword;
						count3 = 180;
					}
				
				}else if(walkmode == 1){
					
						
						//後
							p[1]->x = dp->x;
							p[1]->y = dp->y;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x;
							p[2]->y = dp->y;
							p[2]->z = dp->z - open1;
						//中
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y;
							p[0]->z = dp->z;
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y;
							p[3]->z = dp->z;
						//前
							p[5]->x = dp->x;
							p[5]->y = dp->y;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x;
							p[4]->y = dp->y;
							p[4]->z = dp->z + open3;

						for(int i = 0; i <= moveleg; i++)
							if(moveleg-i >= 2 && moveleg-i <=4)
								p[moveleg-i]->z = p[moveleg-i]->z  + r-(2 * r * (((180-count3)+(i*180))/900));
							else
								p[moveleg-i]->z = p[moveleg-i]->z + r-(2 * r * ((900-((180-count3)+(i*180)-180))/900));
						//要移動的腳
							p[moveleg]->x = dp->x + (moveleg==0 || moveleg==3 ? open2 : 0);
							p[moveleg]->y = dp->y + sin(count3 * DEG_TO_RAD) * r;
							p[moveleg]->z = dp->z - cos(count3 * DEG_TO_RAD) * r * (moveleg>= 2 && moveleg<=4 ? -1 : 1) - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
						for(int i = moveleg+1; i < 6; i++)
							if(i >= 2 && i <=4)
								p[i]->z = p[i]->z  + r-(2 * r * (((180-count3)+((6-i+moveleg)*180))/900));
							else
								p[i]->z = p[i]->z  + r-(2 * r * ((900-((180-count3)+((6-i+moveleg)*180-180)))/900));

						for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 2){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 2;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count3-=speed;
					if(count3 <= 0){
						forword = !forword;
						moveleg = moveleg < 5 ? moveleg + 1 : 0;
						count3 = 180;
					}
				}else if(walkmode == 2){
				
						
						//後
							p[1]->x = dp->x;
							p[1]->y = dp->y;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x;
							p[2]->y = dp->y;
							p[2]->z = dp->z - open1;
						//中
							p[0]->x = dp->x + open2;
							p[0]->y = dp->y;
							p[0]->z = dp->z;
							p[3]->x = dp->x + open2;
							p[3]->y = dp->y;
							p[3]->z = dp->z;
						//前
							p[5]->x = dp->x;
							p[5]->y = dp->y;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x;
							p[4]->y = dp->y;
							p[4]->z = dp->z + open3;

					p[moveleg]->x = dp->x + (moveleg==0 || moveleg==3 ? open2 : 0);
					p[moveleg]->y = dp->y + sin(((moveleg >= 2 && moveleg <=4 ? count3 : 180-count3)/2+90*(moveleg >= 2 && moveleg <=4 ? 1:0)) * DEG_TO_RAD) * r;
					p[moveleg]->z = dp->z - cos(((moveleg >= 2 && moveleg <=4 ? count3 : 180-count3)/2+90*(moveleg >= 2 && moveleg <=4 ? 1:0)) * DEG_TO_RAD) * r - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
					
					int nextleg = movelegmap[moveleg];
					for(int i = 3; i >=0; i--){
						if(nextleg >= 2 && nextleg <=4)
							p[nextleg]->z = p[nextleg]->z  + r-(2 * r * (720-((180-count3)+(180*i)-180))/720);
						else
							p[nextleg]->z = p[nextleg]->z  + r-(2 * r * ((180-count3)+(180*i)-180)/720);

						nextleg = movelegmap[nextleg];
					}
					p[nextleg]->x = dp->x + (nextleg==0 || nextleg==3 ? open2 : 0);
					p[nextleg]->y = dp->y + sin(((nextleg >= 2 && nextleg <=4 ? count3 : 180-count3)/2+90*(nextleg >= 2 && nextleg <=4 ? 0:1)) * DEG_TO_RAD) * r;
					p[nextleg]->z = dp->z - cos(((nextleg >= 2 && nextleg <=4 ? count3 : 180-count3)/2+90*(nextleg >= 2 && nextleg <=4 ? 0:1)) * DEG_TO_RAD) * r - (nextleg==1 || nextleg==2 ? open1 : 0) + (nextleg==4 || nextleg==5 ? open3 : 0);
						
					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 2){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 2;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count3=count3-speed*2;
					if(count3 <= 0){
						forword = !forword;
						if(moveleg == 1)
							moveleg = 4;
						else if(moveleg == 4)
							moveleg = 0;
						else if(moveleg == 0)
							moveleg = 2;
						else if(moveleg == 2)
							moveleg = 5;
						else if(moveleg == 5)
							moveleg = 3;
						else if(moveleg == 3)
							moveleg = 1;
						count3 = 180;
					}
				}
				break;
				case 'q': 
					if(walkmode == 0){
						//後
						if(!forword){
							p[1]->x = dp->x + r - (2 * r * (count2/180)) ;
							p[1]->y = dp->y + body_pitch - body_roll;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x + cos(count2 * DEG_TO_RAD) * r ;
							p[2]->y = dp->y + sin(count2 * DEG_TO_RAD) * r + body_pitch + body_roll;
							p[2]->z = dp->z - open1;
						}else{
							p[1]->x = dp->x - cos(count2 * DEG_TO_RAD) * r;
							p[1]->y = dp->y + sin(count2 * DEG_TO_RAD) * r + body_pitch - body_roll;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x - (r-(2 * r * (count2/180)));
							p[2]->y = dp->y + body_pitch + body_roll;
							p[2]->z = dp->z - open1;
						}
						//中
						if(!forword){
							p[0]->x = dp->x - cos(count2 * DEG_TO_RAD) * r + open2;
							p[0]->y = dp->y + sin(count2 * DEG_TO_RAD) * r - body_roll;
							p[0]->z = dp->z;
							p[3]->x = dp->x - (r-(2 * r * (count2/180))) + open2;
							p[3]->y = dp->y + body_roll;
							p[3]->z = dp->z;
						}else{
							p[0]->x = dp->x + r-(2 * r * (count2/180)) + open2;
							p[0]->y = dp->y - body_roll;
							p[0]->z = dp->z;
							p[3]->x = dp->x + cos(count2 * DEG_TO_RAD) * r + open2;
							p[3]->y = dp->y + sin(count2 * DEG_TO_RAD) * r + body_roll;
							p[3]->z = dp->z;
						}
						//前
						if(!forword){
							p[5]->x = dp->x + r-(2 * r * (count2/180));
							p[5]->y = dp->y - body_pitch - body_roll;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x + cos(count2 * DEG_TO_RAD) * r ;
							p[4]->y = dp->y + sin(count2 * DEG_TO_RAD) * r - body_pitch + body_roll;
							p[4]->z = dp->z+ open3;
						}else{
							p[5]->x = dp->x -cos(count2 * DEG_TO_RAD) * r;
							p[5]->y = dp->y + sin(count2 * DEG_TO_RAD) * r - body_pitch - body_roll;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x - (r-(2 * r * (count2/180)));
							p[4]->y = dp->y - body_pitch + body_roll;
							p[4]->z = dp->z + open3;
						}
					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 3){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 3;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count2+=speed;
					if(count2 >= 180){
						forword = !forword;
						count2 = 0;
					}
					
				}else if(walkmode == 1){
					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y;
						p[4]->z = dp->z + open3;
						
							
					for(int i = 0; i <= moveleg; i++)
						p[moveleg-i]->x = p[moveleg-i]->x + (r-(2 * r * ((count2+(i*180)-180)/900)))*(moveleg-i >=2 && moveleg-i <=4 ? -1 : 1);
					//要移動的腳
					p[moveleg]->x = dp->x - cos(count2 * DEG_TO_RAD) * r * (moveleg >=2 && moveleg <=4 ? -1 : 1) + (moveleg==0 || moveleg==3 ? open2 : 0);
						p[moveleg]->y = dp->y + sin(count2 * DEG_TO_RAD) * r;
						p[moveleg]->z = dp->z - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
					for(int i = moveleg+1; i < 6; i++)
						p[i]->x = p[i]->x  + (r-(2 * r * ((count2+((6-i+moveleg)*180)-180)/900)))*(i >=2 && i <=4 ? -1 : 1);
						

					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 3){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 3;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count2+=speed;
					if(count2 >= 180){
						forword = !forword;
						moveleg = moveleg < 5 ? moveleg + 1 : 0;
						count2 = 0;
					}
				}else if(walkmode == 2){
				//後
						p[1]->x = dp->x;
						p[1]->y = dp->y;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y;
						p[4]->z = dp->z + open3;
						
					p[moveleg]->x = dp->x - cos((count2/2) * DEG_TO_RAD) * r * (moveleg >=2 && moveleg <=4 ? -1 : 1) + (moveleg==0 || moveleg==3 ? open2 : 0);
					p[moveleg]->y = dp->y + sin((count2/2) * DEG_TO_RAD) * r;
					p[moveleg]->z = dp->z - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
					
					int nextleg = movelegmap[moveleg];
					for(int i = 3; i >=0; i--){
						p[nextleg]->x = p[nextleg]->x  + (r-(2 * r * (count2+(180*i))/720))*(nextleg >=2 && nextleg <=4 ? -1 : 1);
						nextleg = movelegmap[nextleg];
					}
					p[nextleg]->x = dp->x - cos((count2/2+90) * DEG_TO_RAD) * r * (nextleg >=2 && nextleg <=4 ? -1 : 1) + (nextleg==0 || nextleg==3 ? open2 : 0);
					p[nextleg]->y = dp->y + sin((count2/2+90) * DEG_TO_RAD) * r;
					p[nextleg]->z = dp->z - (nextleg==1 || nextleg==2 ? open1 : 0) + (nextleg==4 || nextleg==5 ? open3 : 0);
						



					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 3){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 3;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count2=count2+speed*2;
					if(count2 >= 180){
						forword = !forword;
						if(moveleg == 1)
							moveleg = 4;
						else if(moveleg == 4)
							moveleg = 0;
						else if(moveleg == 0)
							moveleg = 2;
						else if(moveleg == 2)
							moveleg = 5;
						else if(moveleg == 5)
							moveleg = 3;
						else if(moveleg == 3)
							moveleg = 1;
						count2 = 0;
					}
				}
				break;
				case 'e': 
					if(walkmode == 0){
						//後
						if(!forword){
							p[1]->x = dp->x + r - (2 * r * (count2/180)) ;
							p[1]->y = dp->y + body_pitch - body_roll;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x + cos(count2 * DEG_TO_RAD) * r ;
							p[2]->y = dp->y + sin(count2 * DEG_TO_RAD) * r + body_pitch + body_roll;
							p[2]->z = dp->z - open1;
						}else{
							p[1]->x = dp->x - cos(count2 * DEG_TO_RAD) * r;
							p[1]->y = dp->y + sin(count2 * DEG_TO_RAD) * r + body_pitch - body_roll;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x - (r-(2 * r * (count2/180)));
							p[2]->y = dp->y + body_pitch + body_roll;
							p[2]->z = dp->z - open1;
						}
						//中
						if(!forword){
							p[0]->x = dp->x - cos(count2 * DEG_TO_RAD) * r + open2;
							p[0]->y = dp->y + sin(count2 * DEG_TO_RAD) * r - body_roll;
							p[0]->z = dp->z;
							p[3]->x = dp->x - (r-(2 * r * (count2/180))) + open2;
							p[3]->y = dp->y + body_roll;
							p[3]->z = dp->z;
						}else{
							p[0]->x = dp->x + r-(2 * r * (count2/180)) + open2;
							p[0]->y = dp->y - body_roll;
							p[0]->z = dp->z;
							p[3]->x = dp->x + cos(count2 * DEG_TO_RAD) * r + open2;
							p[3]->y = dp->y + sin(count2 * DEG_TO_RAD) * r + body_roll;
							p[3]->z = dp->z;
						}
						//前
						if(!forword){
							p[5]->x = dp->x + r-(2 * r * (count2/180));
							p[5]->y = dp->y - body_pitch - body_roll;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x + cos(count2 * DEG_TO_RAD) * r ;
							p[4]->y = dp->y + sin(count2 * DEG_TO_RAD) * r - body_pitch + body_roll;
							p[4]->z = dp->z+ open3;
						}else{
							p[5]->x = dp->x -cos(count2 * DEG_TO_RAD) * r;
							p[5]->y = dp->y + sin(count2 * DEG_TO_RAD) * r - body_pitch - body_roll;
							p[5]->z = dp->z + open3;
							p[4]->x = dp->x - (r-(2 * r * (count2/180)));
							p[4]->y = dp->y - body_pitch + body_roll;
							p[4]->z = dp->z + open3;
						}
					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 1){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 1;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count2-=speed;
					if(count2 <= 0){
						forword = !forword;
						count2 = 180;
					}
					
				}else if(walkmode == 1){
					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y;
						p[4]->z = dp->z + open3;

					for(int i = 0; i <= moveleg; i++)
						p[moveleg-i]->x = p[moveleg-i]->x + (r-(2 * r * (((180-count2)+(i*180))/900)))*(moveleg-i >=2 && moveleg-i <=4 ? 1 : -1);
					//要移動的腳
					p[moveleg]->x = dp->x - cos(count2 * DEG_TO_RAD) * r * (moveleg >=2 && moveleg <=4 ? -1 : 1) + (moveleg==0 || moveleg==3 ? open2 : 0);
						p[moveleg]->y = dp->y + sin(count2 * DEG_TO_RAD) * r;
						p[moveleg]->z = dp->z - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
					for(int i = moveleg+1; i < 6; i++)
						p[i]->x = p[i]->x  + (r-(2 * r * (((180-count2)+((6-i+moveleg)*180))/900)))*(i >=2 && i <=4 ? 1 : -1);

					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 1){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 1;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count2-=speed;
					if(count2 <= 0){
						forword = !forword;
						moveleg = moveleg < 5 ? moveleg + 1 : 0;
						count2 = 180;
					}
				}else if(walkmode == 2){
					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y;
						p[4]->z = dp->z + open3;



					p[moveleg]->x = dp->x - cos((count2/2+90) * DEG_TO_RAD) * r * (moveleg >=2 && moveleg <=4 ? -1 : 1) + (moveleg==0 || moveleg==3 ? open2 : 0);
					p[moveleg]->y = dp->y + sin((count2/2+90) * DEG_TO_RAD) * r;
					p[moveleg]->z = dp->z - (moveleg==1 || moveleg==2 ? open1 : 0) + (moveleg==4 || moveleg==5 ? open3 : 0);
					
					int nextleg = movelegmap[moveleg];
					for(int i = 3; i >=0; i--){
						p[nextleg]->x = p[nextleg]->x  + (r-(2 * r * ((180-count2)+(180*i))/720))*(nextleg >=2 && nextleg <=4 ? 1 : -1);
						nextleg = movelegmap[nextleg];
					}
					p[nextleg]->x = dp->x - cos((count2/2) * DEG_TO_RAD) * r * (nextleg >=2 && nextleg <=4 ? -1 : 1) + (nextleg==0 || nextleg==3 ? open2 : 0);
					p[nextleg]->y = dp->y + sin((count2/2) * DEG_TO_RAD) * r;
					p[nextleg]->z = dp->z - (nextleg==1 || nextleg==2 ? open1 : 0) + (nextleg==4 || nextleg==5 ? open3 : 0);

					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);

					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;

					if(mode != 1){
						count = 0;
						count2 = 0;
						count3 = 0;

						mode = 1;
						if(audioEngine)
							audioEngine->play2D(sound[rand() % 7 + 8], false, false, true);
						#ifdef onRB
							rcservo_SetAction(motion_frame, 250);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					}else{
						#ifdef onRB
							rcservo_SetAction(motion_frame, 100);
							while (rcservo_PlayAction() != RCSERVO_PLAYEND);
						#else
							system("cls");
							for(int i = 0; i < 6; i++){
								for(int j = 0; j < 3; j++){
									printf("%.2f %d\t",alldegree[i*3+j],motion_frame[i*3+j]);
								}
								printf("\n");
							}
							printf("%.0f",count);
						#endif
					
					}
					count2=count2-speed*2;
					if(count2 <= 0){
						forword = !forword;
						if(moveleg == 1)
							moveleg = 4;
						else if(moveleg == 4)
							moveleg = 0;
						else if(moveleg == 0)
							moveleg = 2;
						else if(moveleg == 2)
							moveleg = 5;
						else if(moveleg == 5)
							moveleg = 3;
						else if(moveleg == 3)
							moveleg = 1;
						count2 = 180;
					}
				
				}
				break;
				case '1':
					speed = speed < 180 ? speed += 2 : 180;
					printf("speed = %.2f",speed);
					break;
				case '2':
					speed = speed < 180 ? speed -= 2 : 180;
					printf("speed = %.2f",speed);
					break;
					
				case '3':
					open1 += 0.5;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 500);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("open1 = %.2f\n",open1);
					break;
				case '4':
					open1 -= 0.5;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 500);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("open1 = %.2f\n",open1);
					break;
				
				case '5':
					open2 += 0.5;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 500);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("open2 = %.2f\n",open2);
					break;
				case '6':
					open2 -= 0.5;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 500);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("open2 = %.2f\n",open2);
					break;
					
				case '7':
					open3 += 0.5;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 500);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("open3 = %.2f\n",open3);
					break;
				case '8':
					open3 -= 0.5;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 500);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("open3 = %.2f\n",open3);
					break;
				case 'v':
					shift_x += 0.5;
					dp->x = L1+L2+shift_x;
					dp->y = -L3+shift_y;
					dp->z = 0;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 300);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("shift X = %.2f\n",shift_x);
					break;
				case 'b':
					shift_x -= 0.5;
					dp->x = L1+L2+shift_x;
					dp->y = -L3+shift_y;
					dp->z = 0;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 300);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("shift X = %.2f\n",shift_x);
					break;
				case 'n':
					shift_y += 0.2;
					dp->x = L1+L2+shift_x;
					dp->y = -L3+shift_y;
					dp->z = 0;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 300);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("shift Y = %.2f\n",shift_y);
					break;
				case 'm':
					shift_y -= 0.2;
					dp->x = L1+L2+shift_x;
					dp->y = -L3+shift_y;
					dp->z = 0;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 300);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("shift Y = %.2f\n",shift_y);
					break;
					case 't':
					body_pitch += 0.2;
					dp->x = L1+L2+shift_x;
					dp->y = -L3+shift_y;
					dp->z = 0;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 300);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("body pitch = %.2f\n",body_pitch);
					break;
					case 'g':
					body_pitch -= 0.2;
					dp->x = L1+L2+shift_x;
					dp->y = -L3+shift_y;
					dp->z = 0;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 300);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("body pitch = %.2f\n",body_pitch);
					break;
					
					case 'f':
					body_roll += 0.2;
					dp->x = L1+L2+shift_x;
					dp->y = -L3+shift_y;
					dp->z = 0;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 300);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("body roll = %.2f\n",body_roll);
					break;
					
					case 'h':
					body_roll -= 0.2;
					dp->x = L1+L2+shift_x;
					dp->y = -L3+shift_y;
					dp->z = 0;
					//後
					if(forword){
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z + r - (2 * r * (count/180)) - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch + body_roll;
						p[2]->z = -cos(count * DEG_TO_RAD) * r - open1;
					}else{
						p[1]->x = dp->x;
						p[1]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_pitch - body_roll;
						p[1]->z = dp->z - cos(count * DEG_TO_RAD) * r - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z + r-(2 * r * (count/180)) - open1;
					}
					//中
					if(forword){
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_roll;
						p[0]->z = dp->z -cos(count * DEG_TO_RAD) * r;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z + r-(2 * r * (count/180));
					}else{
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z + r-(2 * r * (count/180));
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + sin(count * DEG_TO_RAD) * r + body_roll;
						p[3]->z = dp->z -cos(count * DEG_TO_RAD) * r;
					}
					//前
					if(forword){
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + r-(2 * r * (count/180)) + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch + body_roll;
						p[4]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
					}else{
						p[5]->x = dp->x;
						p[5]->y = dp->y + sin(count * DEG_TO_RAD) * r - body_pitch - body_roll;
						p[5]->z = dp->z -cos(count * DEG_TO_RAD) * r + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + r-(2 * r * (count/180)) + open3;
					}
				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);

				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
				rcservo_SetAction(motion_frame, 300);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					printf("body roll = %.2f\n",body_roll);
					break;
				case 'u':		
					count = 0;
					count2 = 0;
					count3 = 0;

					mode = 0;
					if (audioEngine)
						audioEngine->play2D(sound[rand() % 5 + 0], false, false, true);

					//後
						p[1]->x = dp->x-5;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z-5 - open1;
						p[2]->x = dp->x+5;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z-5 - open1;
					//中
						p[0]->x = dp->x-5 + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z-5;
						p[3]->x = dp->x+5 + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z-5;
					//前
						p[5]->x = dp->x-5;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z-5 + open3;
						p[4]->x = dp->x+5;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z-5 + open3;


				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);
				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
				
					lefthand();
	#ifdef onRB
					rcservo_SetAction(motion_frame, 300);
					while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif

					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + open3;

				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);
				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
					home();
	#ifdef onRB
					rcservo_SetAction(motion_frame, 300);
					while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					break;
				case 'i':		
					count = 0;
					count2 = 0;
					count3 = 0;

					mode = 0;
					if (audioEngine)
						audioEngine->play2D(sound[rand() % 5 + 0], false, false, true);

					//後
						p[1]->x = dp->x+5;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z-5 - open1;
						p[2]->x = dp->x-5;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z-5 - open1;
					//中
						p[0]->x = dp->x+5 + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z-5;
						p[3]->x = dp->x-5 + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z-5;
					//前
						p[5]->x = dp->x+5;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z-5 + open3;
						p[4]->x = dp->x-5;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z-5 + open3;

				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);
				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
					righthand();
	#ifdef onRB
					rcservo_SetAction(motion_frame, 300);
					while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					

					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + open3;

				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);
				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
					home();
	#ifdef onRB
					rcservo_SetAction(motion_frame, 300);
					while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					break;
				case 'o':
					count = 0;
					count2 = 0;
					count3 = 0;

					mode = 0;
					if (audioEngine)
						audioEngine->play2D(sound[rand() % 5 + 0], false, false, true);

					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z-5 - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z-5 - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z-5;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z-5;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z-5 + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z-5 + open3;

					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);
					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
					bothhand();
	#ifdef onRB
					rcservo_SetAction(motion_frame, 300);
					while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					

					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + open3;

					for(int i = 0; i < 6; i++)
						InverseKinematics(p[i],&alldegree[i*3]);
					for(int i = 0; i < 18; i++)
						motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
					home();
	#ifdef onRB
					rcservo_SetAction(motion_frame, 300);
					while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					break;
				case 'p':
					count = 0;
					count2 = 0;
					count3 = 0;

					mode = 0;
					if (audioEngine)
						audioEngine->play2D(sound[rand() % 5 + 0], false, false, true);

					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y-4 + body_pitch - body_roll;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y-4 + body_pitch + body_roll;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y-1 - body_roll;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y-1 + body_roll;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y+2 - body_pitch - body_roll;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y+2 - body_pitch + body_roll;
						p[4]->z = dp->z + open3;

				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);
				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
					tail_atk();
	#ifdef onRB
					rcservo_SetAction(motion_frame, 100);
					while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
				
					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y + body_pitch - body_roll;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y + body_pitch + body_roll;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y - body_pitch - body_roll;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y - body_pitch + body_roll;
						p[4]->z = dp->z + open3;

				for(int i = 0; i < 6; i++)
					InverseKinematics(p[i],&alldegree[i*3]);
				for(int i = 0; i < 18; i++)
					motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
					tail_atk();
					home();
	#ifdef onRB
					rcservo_SetAction(motion_frame, 1000);
					while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					break;
				case 'j':
					count = 0;
					count2 = 0;
					count3 = 0;

					mode = 0;
					if (audioEngine)
						audioEngine->play2D(sound[rand() % 2 + 6], false, false, true);

					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y+3.5 + body_pitch - body_roll;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y+3.5 + body_pitch + body_roll;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y-3.5 - body_pitch - body_roll;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y-3.5 - body_pitch + body_roll;
						p[4]->z = dp->z + open3;
						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);
						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
						laugh();
	#ifdef onRB
				rcservo_SetAction(motion_frame, 500);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif

					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y +2.5 + body_pitch - body_roll;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y -2.5 + body_pitch + body_roll;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y +2.5- body_roll;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y -2.5+ body_roll;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y +2.5 - body_pitch - body_roll;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y -2.5 - body_pitch + body_roll;
						p[4]->z = dp->z + open3;
						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);
						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
						home();
	#ifdef onRB
				rcservo_SetAction(motion_frame, 500);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y+3.5 + body_pitch - body_roll;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y+3.5 + body_pitch + body_roll;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y - body_roll;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y + body_roll;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y-3.5 - body_pitch - body_roll;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y-3.5 - body_pitch + body_roll;
						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);
						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
						laugh();
	#ifdef onRB
				rcservo_SetAction(motion_frame, 500);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					//後
						p[1]->x = dp->x;
						p[1]->y = dp->y +2.5 + body_pitch - body_roll;
						p[1]->z = dp->z - open1;
						p[2]->x = dp->x;
						p[2]->y = dp->y -2.5 + body_pitch + body_roll;
						p[2]->z = dp->z - open1;
					//中
						p[0]->x = dp->x + open2;
						p[0]->y = dp->y +2.5- body_roll;
						p[0]->z = dp->z;
						p[3]->x = dp->x + open2;
						p[3]->y = dp->y -2.5+ body_roll;
						p[3]->z = dp->z;
					//前
						p[5]->x = dp->x;
						p[5]->y = dp->y +2.5 - body_pitch - body_roll;
						p[5]->z = dp->z + open3;
						p[4]->x = dp->x;
						p[4]->y = dp->y -2.5 - body_pitch + body_roll;
						p[4]->z = dp->z + open3;
						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);
						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
						home();
	#ifdef onRB
				rcservo_SetAction(motion_frame, 500);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					break;
				case 'z':
					walkmode = 0;
					moveleg = 0;
					break;
				case 'x':
					walkmode = 1;
					moveleg = 0;
					break;
				case 'c':
					walkmode = 2;
					moveleg = 0;
					break;
				case '0':
					walkmode = 0;
					swingmode = !swingmode;
					swingcount = 0;
					body_pitch=0;
					body_roll=0;
					moveleg = 0;
					break;
				case ' ':
					count = 0;
					count2 = 0;
					count3 = 0;

					mode = 0;
					if(tailforword){
						tailshake += speed;
						//後
							p[1]->x = dp->x-tailshake/40;
							p[1]->y = dp->y-2.5 + body_pitch - body_roll;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x+tailshake/40;
							p[2]->y = dp->y-2.5 + body_pitch + body_roll;
							p[2]->z = dp->z - open1;
						//中
							p[0]->x = dp->x-tailshake/40 + open2;
							p[0]->y = dp->y - body_roll;
							p[0]->z = dp->z;
							p[3]->x = dp->x+tailshake/40 + open2;
							p[3]->y = dp->y + body_roll;
							p[3]->z = dp->z;
						//前
							p[4]->x = dp->x+tailshake/40;
							p[4]->y = dp->y+2.5 - body_pitch - body_roll;
							p[4]->z = dp->z + open3;
							p[5]->x = dp->x-tailshake/40;
							p[5]->y = dp->y+2.5 - body_pitch + body_roll;
							p[5]->z = dp->z + open3;


						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);
						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
					}else{
						//後
							p[1]->x = dp->x-tailshake/40;
							p[1]->y = dp->y-2.5 + body_pitch - body_roll;
							p[1]->z = dp->z - open1;
							p[2]->x = dp->x+tailshake/40;
							p[2]->y = dp->y-2.5 + body_pitch + body_roll;
							p[2]->z = dp->z - open1;
						//中
							p[0]->x = dp->x-tailshake/40 + open2;
							p[0]->y = dp->y - body_roll;
							p[0]->z = dp->z;
							p[3]->x = dp->x+tailshake/40 + open2;
							p[3]->y = dp->y + body_roll;
							p[3]->z = dp->z;
						//前
							p[4]->x = dp->x+tailshake/40;
							p[4]->y = dp->y+2.5 - body_pitch - body_roll;
							p[4]->z = dp->z + open3;
							p[5]->x = dp->x-tailshake/40;
							p[5]->y = dp->y+2.5 - body_pitch + body_roll;
							p[5]->z = dp->z + open3;

						for(int i = 0; i < 6; i++)
							InverseKinematics(p[i],&alldegree[i*3]);
						for(int i = 0; i < 18; i++)
							motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
						tailshake -= speed;
					}
					if(tailshake-speed <= -200 || tailshake+speed >= 200)
						tailforword = !tailforword;
					tail(tailshake);
	#ifdef onRB
				rcservo_SetAction(motion_frame, 100);
				while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
					break;
			}
		}else if(swingmode){
				dp->x = L1+L2+shift_x;
				dp->y = -L3+shift_y;
				dp->z = 0;
				
				//後
					p[1]->x = dp->x;
					p[1]->y = dp->y + body_pitch - body_roll;
					p[1]->z = dp->z - open1;
					p[2]->x = dp->x;
					p[2]->y = dp->y + body_pitch + body_roll;
					p[2]->z = dp->z - open1;
				//中
					p[0]->x = dp->x + open2;
					p[0]->y = dp->y - body_roll;
					p[0]->z = dp->z;
					p[3]->x = dp->x + open2;
					p[3]->y = dp->y + body_roll;
					p[3]->z = dp->z;
				//前
					p[5]->x = dp->x;
					p[5]->y = dp->y - body_pitch - body_roll;
					p[5]->z = dp->z + open3;
					p[4]->x = dp->x;
					p[4]->y = dp->y - body_pitch + body_roll;
					p[4]->z = dp->z + open3;
			for(int i = 0; i < 6; i++)
				InverseKinematics(p[i],&alldegree[i*3]);
			for(int i = 0; i < 18; i++)
				motion_frame[i] = (int)(alldegree[i] * DEG_TO_PWM * direction[i] + center[i]) / 1;
	#ifdef onRB
			rcservo_SetAction(motion_frame, 300);
			while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	#endif
		}
	}
	audioEngine->removeAllSoundSources();
	audioEngine->play2D(sound[5], false, false, true);
#ifdef onRB
	rcservo_SetAction(home_frame, 5000);
	while (rcservo_PlayAction() != RCSERVO_PLAYEND);
	rcservo_Close();
#endif
	while(true){
		if(kbhit()){
			switch(getch()){
			case 'q':
	           return 0;
			case 27:
				goto restart;
			}
		}
	}

	return 0;
}