
#include <stdio.h>
#include <math.h>
#include <conio.h>
#include "EPCIOIni.h"
#include "MEPCIODev.h"
#include "mv5.h"

#include "stdafx.h"
#include <iostream>

#include <time.h>

#include <opencv2/opencv.hpp> 
#include "opencv2\highgui\highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <windows.h>
#include "PtGreyAPI.h"


#define CARD_INDEX1				0
#define CHANNEL_INDEX1			0
#define CHANNEL_INDEX2			2
#define ESC						0x1b
#define PI 3.14159265359
//initial functions
void	init_encoder(void);
void	init_da(void);
double	LSF(double p);
double	LSF2(double q);
void	close_board(void);

//time setting
double	Time = 0;
double	Time_END = 400;

int	time_counter = 0;
double	v;
double	v2;
double	v_limit;
double  v_limit2;


double  p_cmd = 0;
double  p_err;


double	v_cmd;
double	vol;
double  torque;
double  v_err;
double  vPre = 0;
double  vIntegral = 0;


double  p_cmd2 = 0;
double  p_err2;
double  xKp2;

double	v_cmd2;
double	vol2;
double  torque2;
double  v_err2;
double  vPre2 = 0;
double  vIntegral2 = 0;


double	time_ms;
double	time_interval = 0.001;
double  gearRatio = 10.0;
double  gearRatio2 = 57.0/20.0;


double L[4] = { 0 };
double L_2[4] = { 0 };
double p;
double p2;
long pulse;
long pulse2;

PtGreyCamera cam1;
Mat rawImg;
Mat rawImg_1280X1024, rawImg_640X512,rawImg_saad;
Mat gray(512,640,CV_8U);
Mat output(512,640,CV_8U);
//Mat readin = imread("output.bmp",CV_LOAD_IMAGE_COLOR);
Mat target ;
Mat displayImg;


Point supercenter(320,256); 
Point lefttop (-1,-1);
Point rightdown (-1,-1);
Point backgroundcenter(320,256);
//邊界設定
int img_top = 210;
int	img_bot = 292; 
int img_left = 260;
int img_right = 380;

double locus_t_up=0,locus_t_down=0,over_up=0,over_down=0;
double initial = 0;

double locus_t_left=0,locus_t_right=0,over_left=0,over_right=0;
double initial_xy = 0;


using namespace std;
using namespace cv;
void opencamera();
void detect(Mat gray);
void onMouse(int event,int x, int y ,int flags,void* param);
void TSS_search(Mat input_background1 ,Mat input_target1);
void tracking_control();




char g_file_name[] = "record.txt";
FILE *Ptr_File ;



void _stdcall  Timer_ISR_Function(LIOINT *pstINTsource) // 自己寫的中斷程式
{
	if (pstINTsource->TIMER)
	{
		time_counter++;
		time_ms = time_counter*0.001;//將plus的數量轉換成時間(ms)
		EPCIO_ENC_GetValue(CHANNEL_INDEX1, &pulse, CARD_INDEX1);//讀取目前計數值
		EPCIO_ENC_GetValue(CHANNEL_INDEX2, &pulse2, CARD_INDEX1);//讀取目前計數值

		p = (-pulse/10000.0) * (2*PI) / gearRatio;
		p2 = (-pulse2/10000.0) * (2*PI) / gearRatio2;

		
		v = LSF(p);
		v2 = LSF2(p2);




		//------------------------------------------------------------------------
		double  xKp = 130.0;      //130
		double  vKp = 1.6;       //1.6
		double  vKi = 0.1;

		//p_cmd = 0.75*sin(time_ms);
/*
		p_err = p_cmd - p;
		v_cmd = p_err * xKp;
*/


		v_err = v_cmd - v;
		vIntegral = vIntegral + (vPre + (v_err)) * time_interval / 2;
		torque = v_err * vKp + vIntegral * vKi;

		
	


		double  xKp2 = 120.0;     //120
		double  vKp2 = 0.15;    //0.16
		double  vKi2 = 0.1;

		//p_cmd2 = 0.75*sin(time_ms);
/*
		p_err2 = p_cmd2 - p2;
		v_cmd2 = p_err2 * xKp2;
*/
		v_err2 = v_cmd2 - v2;
		vIntegral2 = vIntegral2 + (vPre2 + (v_err2)) * time_interval / 2;
		torque2 = v_err2 * vKp2 + vIntegral2 * vKi2;


		//----------------------------------------------------------------------


		vol = (torque / gearRatio) / 1.27 * 10.0;
 
		vol2 = (torque2 / gearRatio2) / 1.27 * 10.0;

		

		//極限是+-10v 保險起見給+-6v
		if (vol >= 6)
		{
			vol = 6;
		}
		else if (vol <= -6)
		{
			vol = -6;
		}

		if (vol2 >= 6)
		{
			vol2 = 6;
		}
		else if (vol2 <= -6)
		{
			vol2 = -6;
		}

		EPCIO_DAC_SetOutput(CHANNEL_INDEX1, vol, CARD_INDEX1);
		EPCIO_DAC_SetOutput(CHANNEL_INDEX2, vol2, CARD_INDEX1);
		printf("position= %5f  Velocity is %5f  rpm  time is %4f \r", p, v ,time_ms); 
		fprintf(Ptr_File,"%5f\t %5f\t %5f\t %5f\t %5f\t %5f\t %5f\t %5f\t %5f\t %5f\t %5f\t\n",time_ms,p,p_cmd,v,v_cmd,torque,p2,p_cmd2,v2,v_cmd2,torque2);



	}

}

void main(){

	char cKey = 0;
	int nRet = 0;
	
	nRet = EPCIO6000_Init(NULL, NULL, NULL, NULL, NULL, NULL, NULL,Timer_ISR_Function, NULL, CARD_INDEX1);//回傳值決定初始化成功與否
	
	if (!nRet)//若啟動失敗則回傳Fail.
	{

		printf("Initialization Fails\n");
		return;
	}
	Ptr_File = fopen(g_file_name, "wt");
	
	printf("Press any key to start.......");//提示使用者
	while (!_kbhit());//當有按下任意按鍵

	EPCIO_ResetModule(RESET_ALL, CARD_INDEX1);
	EPCIO_LIO_SetTimer(40000, CARD_INDEX1);//25ns*40000=1ms
	EPCIO_LIO_EnableTimerInt(CARD_INDEX1);
	EPCIO_LIO_EnableTimer(CARD_INDEX1);

	init_encoder();
	init_da();

	



	// ((cKey != ESC) && Time < Time_END) //當按下的按鍵不等於ESC 且 還沒到結束時間

	
	opencamera();
	

	EPCIO_DAC_SetOutput(CARD_INDEX1, 0, CARD_INDEX1); //目前的DAC輸出設定為0v,正負10v
	EPCIO_LIO_DisableTimerInt(CARD_INDEX1);
	EPCIO_LIO_DisableTimer(CARD_INDEX1);



	close_board();
	fclose(Ptr_File);
	return;

}


double LSF(double p)
{
	int i;

	for (i = 1; i <= 3; i++)
	{
		L[i - 1] = L[i];
	}
	L[3] = p;

	return (L[3] * 0.3 + L[2] * 0.1 + L[1] * (-0.1) + L[0] * (-0.3)) / time_interval;
}

double LSF2(double q)
{
	int j;

	for (j = 1; j <= 3; j++)
	{
		L_2[j - 1] = L_2[j];
	}
	L_2[3] = q;

	return (L_2[3] * 0.3 + L_2[2] * 0.1 + L_2[1] * (-0.1) + L_2[0] * (-0.3)) / time_interval;
}

void init_da(void)
{
	EPCIO_LIO_EnablePulseDAC(CARD_INDEX1);
	EPCIO_DAC_StartConv(CARD_INDEX1);
	
	EPCIO_LIO_ServoOn(CHANNEL_INDEX1, CARD_INDEX1);
	//EPCIO_DAC_SetCmdSource(CHANNEL_INDEX1, DAC_CMD_SOFT, CARD_INDEX1);//給命令來源
	//EPCIO_ENC_EnableCompInt(CHANNEL_INDEX1, CARD_INDEX1);

	EPCIO_LIO_ServoOn(CHANNEL_INDEX2, CARD_INDEX1);
	//EPCIO_DAC_SetCmdSource(CHANNEL_INDEX2, DAC_CMD_SOFT, CARD_INDEX1);

}
void init_encoder(void)
{
	EPCIO_ENC_StartInput(CARD_INDEX1);//開啟計數器

	EPCIO_ENC_SetInputRate(CHANNEL_INDEX1, ENC_RATE_X4, CARD_INDEX1);//設置解碼倍率
	EPCIO_ENC_SetInputType(CHANNEL_INDEX1, ENC_TYPE_AB, CARD_INDEX1);//設置輸入形式
	EPCIO_ENC_ClearCounter(CHANNEL_INDEX1, CARD_INDEX1);//清除計數器
	
	
	EPCIO_ENC_SetInputRate(CHANNEL_INDEX2, ENC_RATE_X4, CARD_INDEX1);//設置解碼倍率(4倍頻)
	EPCIO_ENC_SetInputType(CHANNEL_INDEX2, ENC_TYPE_AB, CARD_INDEX1);//設置輸入形式
	EPCIO_ENC_ClearCounter(CHANNEL_INDEX2, CARD_INDEX1);//清除計數器

}

void close_board(void)
{
	EPCIO_DAC_SetOutput(CHANNEL_INDEX1, 0.0, CARD_INDEX1);//結束時電壓歸零
	EPCIO_LIO_ServoOff(CHANNEL_INDEX1, CARD_INDEX1);


	EPCIO_DAC_SetOutput(CHANNEL_INDEX2, 0.0, CARD_INDEX1);//結束時電壓歸零
	EPCIO_LIO_ServoOff(CHANNEL_INDEX2, CARD_INDEX1);

	
	EPCIO_DAC_StopConv(CARD_INDEX1);
	EPCIO_Close(CARD_INDEX1);
	
}

void opencamera()
{
	cam1.CheckCameras();
	// Create a PtGreyCamera and connect it to a camera by index
	if(!cam1.ConnectCameraFromIndex(0))
		return;
	if(!cam1.SetCameraConfiguration_640X512())
		return;
	if(!cam1.SetExposure())
		return;
	if(!cam1.SetWhiteBalance())
		return;			
	// Switch on the capture ability
	if(!cam1.StartCapture())
		return;

	bool PrintManual=true;
	char key = 0;

	while(key != 'q')
	{


		if(PrintManual)
		{
			cout << endl << endl;
			cout << "Press Q to go back to Main Control Menu." << endl;
			cout << "Press D to detect circle." << endl;
			cout << endl;
			PrintManual=false;
		}



		if(cam1.ReadImage(rawImg_640X512))
		{
			//Mat image_;
			system("cls");
			setMouseCallback("Img",onMouse , NULL);
			detect(rawImg_640X512);
			imshow("Img",displayImg);
		}
		else
			break;
		
		

		if(key=='s')
			imwrite("test.bmp",rawImg_640X512);
		key = waitKey(20);
		
	}

		

	cam1.StopCapture();
	cam1.DisconnectCamera();
	destroyWindow("Img");
	
}

void detect(Mat gray)
{
	
	target = imread("output.bmp",CV_LOAD_IMAGE_COLOR);
	
	displayImg = gray.clone();
	

	TSS_search(gray , target);
	//tracking_control();

}


void onMouse(int event,int x, int y ,int flags,void* param)
{


	if(event==CV_EVENT_LBUTTONDOWN)
	{
		lefttop.x = x;
		lefttop.y = y;
	}

	if(event==CV_EVENT_LBUTTONUP)
	{
		rightdown.x = x;
		rightdown.y = y;
		int temp;

			
		if(rightdown.x < lefttop.x)
		{
			temp = rightdown.x;
			rightdown.x = lefttop.x;
			lefttop.x = temp;
		
		}
		if(rightdown.y < lefttop.y)
		{
			temp = rightdown.y;
			rightdown.y = lefttop.y;
			lefttop.y = temp;
		}



		Mat templateImage = displayImg.clone();//rawImg_640X512.clone();

		Rect target = Rect (lefttop.x , lefttop.y, rightdown.x - lefttop.x+1, rightdown.y - lefttop.y +1 ); 
	
		Mat tplImage = templateImage(target).clone(); 

		imwrite ("output.bmp", tplImage);
		cout << "test";
	}



}



void TSS_search(Mat input_background1 ,Mat input_target1)
{

	int count=0;
	//轉灰階
	Mat input_background ;
	cvtColor(input_background1 ,input_background,CV_BGR2GRAY);
	Mat input_target ;
	cvtColor(input_target1,input_target  ,CV_BGR2GRAY);
	
	Point target_center(input_target.cols/2 ,input_target.rows/2 );
	
	Point cp[9];
	int StepSize[3] = {52,26,10};
	int temp = 0;
	int min_i = 0;
	int compare[9]={0};
	int compare_min_i=0;
	int top_bound = 0;
	int	bot_bound = input_background.rows-1; 
	int left_bound = 0;
	int right_bound = input_background.cols-1;


	for(int StepIndex=0 ; StepIndex<3 ; StepIndex++){
			
		    //定義九宮格
			cp[0].x = supercenter.x - StepSize[StepIndex];
			cp[1].x = supercenter.x ;
			cp[2].x = supercenter.x + StepSize[StepIndex];

			cp[0].y = supercenter.y - StepSize[StepIndex];
			cp[1].y = supercenter.y - StepSize[StepIndex];
			cp[2].y = supercenter.y - StepSize[StepIndex];

			cp[3].x = supercenter.x - StepSize[StepIndex];
			cp[4].x = supercenter.x ;
			cp[5].x = supercenter.x + StepSize[StepIndex];

			cp[3].y = supercenter.y ;
			cp[4].y = supercenter.y ;
			cp[5].y = supercenter.y ;

			cp[6].x = supercenter.x - StepSize[StepIndex];
			cp[7].x = supercenter.x ;
			cp[8].x = supercenter.x + StepSize[StepIndex];

			cp[6].y = supercenter.y + StepSize[StepIndex];
			cp[7].y = supercenter.y + StepSize[StepIndex];
			cp[8].y = supercenter.y + StepSize[StepIndex];

			temp = 0;

			for(int i=0;i<9;i++){

			    if(cp[i].x > right_bound || cp[i].x < left_bound || cp[i].y < top_bound || cp[i].y > bot_bound) continue;

				if((cp[i].x+input_target.cols/2) > right_bound || (cp[i].x-input_target.cols/2) < left_bound || (cp[i].y-input_target.rows/2) < top_bound || (cp[i].y+input_target.rows/2)  > bot_bound)continue;
				
				temp++;
				for(int k=(-input_target.rows/2);k<input_target.rows/2;k++){
					uchar *data = input_background.ptr<uchar>(cp[i].y+k);
					uchar *data2 = input_target.ptr<uchar>(input_target.rows/2+k);

					for(int j=(-input_target.cols/2);j<input_target.cols/2;j++){
						compare[i]+=abs(data[cp[i].x+j]-data2[input_target.cols/2+j]);
					}
				}
				if(temp==1){
					compare_min_i=compare[i];
				}
				else if(compare_min_i>compare[i]){
					compare_min_i=compare[i];
					supercenter.x = cp[i].x; 
					supercenter.y = cp[i].y; 

				}
			
		    }
	
	}

    Point tl(supercenter.x - input_target.cols/2 , supercenter.y - input_target.rows/2);
	Point br(supercenter.x + input_target.cols/2 ,supercenter.y + input_target.rows/2);
	
	

	rectangle(displayImg , tl , br , Scalar::all(0) , 3);

}


void tracking_control()
{
	int distance_x = supercenter.x-backgroundcenter.x;
	int distance_y = supercenter.y-backgroundcenter.y;

	int x_interval2 = 160;
	int x_interval1 = 80;
	int y_interval2 = 140;
	int y_interval1 = 70;



	if(supercenter.x>img_right)
	{		
		if(supercenter.x>(img_right+x_interval2))
		{	
			v_cmd = -1;
		}
		else if(supercenter.x>(img_right+x_interval1))
		{
		    v_cmd = -0.8;
		}
		else{
			v_cmd = -0.6;		
		}
	}

	else if(supercenter.x<img_left)
	{
		if(supercenter.x<(img_right-x_interval2))
		{
		    v_cmd = 1;
		}
		else if(supercenter.x<(img_right-x_interval1))
		{
			v_cmd = 0.8;
		}
		else{
			v_cmd = 0.6;		
		}
	}
	else
	{
		v_cmd =0;
	}


	if(supercenter.y>img_bot)
	{
		if(supercenter.y>(img_bot+y_interval2))
		{
			v_cmd2=-1;
		}
		else if(supercenter.y>(img_bot+y_interval1))
		{
			v_cmd2=-0.8;
		}
		else
		{
			v_cmd2=-0.6;
		}
		
	}

	else if(supercenter.y < img_top)
	{
		
		if(supercenter.y<(img_bot-y_interval2))
		{
			v_cmd2=1;
		}
		else if(supercenter.y<(img_bot-y_interval1))
		{
			v_cmd2=0.8;
		}
		else
		{
			v_cmd2=0.6;
		}
	}

	else
	{
		v_cmd2=0;

	}

	

}


