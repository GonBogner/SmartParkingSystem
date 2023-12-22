//Gon Bogner and Shai Gal Netser Smart Parking System:


#include <unistd.h>  //Used for UART
#include <fcntl.h>  //Used for UART
#include <termios.h>  //Used for UART
#include <string.h>
#include <cstdio>
#include <sstream>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "tclap/CmdLine.h"
#include "support/filesystem.h"
#include "support/timing.h"
#include "support/platform.h"
#include "video/videobuffer.h"
#include "motiondetector.h"
#include "alpr.h"
#include <wiringPi.h>
#include <stdio.h>

//pin difentions
#define LED 7
#define TRIG 21
#define ECHO 22
#define MOTOR1 2
#define MOTOR2 3
#define SW 6




//UART defenitions
char ctrlZ[]="\032";
char enter[]="\r";
int uart0_filestream = -1;



//counters
#define MAIN_LOOP_SLEEP_IN_MS 10


//UART func
int OpenUART()
{
	struct termios options;
	uart0_filestream = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
	//ERROR - CAN'T OPEN SERIAL PORT
	printf("\nError - Unable to open UART.  Ensure it is not in use by another application\n");
	return -1;
	}
	else
		printf("\nUART Opened");
		
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
	return 0;
}

void SendUART( char *msg)
{
	unsigned int len;
	len=strlen(msg);
	if (uart0_filestream != -1)
		{
		int count = write(uart0_filestream, msg, len);
		//Filestream, bytes to write, number of bytes to write
		if (count <= 0)
			{
			printf("\nUART send error   %d bytes transmit",count);
			}
			else
			{
			printf("\nUART send ok  %d bytes transmit '%s'",count, msg);
			}
	}
}

void RcvUART( char *msg)
{
	if (uart0_filestream != -1)
		{
		msg[0]=0;
		// Read up to 255 characters from the port if they are there
		int count = read(uart0_filestream, (void*)msg, 255);
		//Filestream, buffer to store in, number of bytes to read (max)
		if (count < 0)
			{
			//An error occured (will occur if there are no bytes)
			printf("\nUART rcv err %d bytes rcv",count);
			}
			else if (count == 0)
			{
			//No data waiting
			printf("\nUART rcv 0 bytes rcv");
			}
			else
			{
			//Bytes received
			msg[count] = '\0';
			printf("\nUART rcv %d bytes  msg=%s\n", count, msg);
			}
		}
}



void CloseUART()
{
//----- CLOSE THE UART -----
close(uart0_filestream);
}



int main_UART(char num[])
{
	char at[]= "AT";
	char cmgf[]= "AT+CMGF=1";
	char cmgs[]= "AT+CMGS=";
	char text[]= "Welcome to the parking lot!";
	char toPrint[300] = "";
	char rcvmsg[300];
	if (OpenUART() == -1)
	{
		std::cout << "Error during thr UART procces."<< std::endl;
		return 0;
	}
	SendUART(at);
	SendUART(enter);
	sleep(3); //3 sec
	SendUART(cmgf);
	SendUART(enter);
	sleep(3); //3 sec
	strcat(toPrint,cmgs);
	strcat(toPrint,num);
	SendUART(toPrint);
	SendUART(enter);
	sleep(3); //3 sec
	SendUART(text);
	SendUART(ctrlZ);
	sleep(3); //3 sec
	CloseUART();
	return 1;
  
}












//hardwere func
int led_blink(int x)
{
	if (x==1)
	{	
		digitalWrite(LED,HIGH);
		x=0;
	} 
	else
	{
		digitalWrite(LED,LOW);
		x=1;
	}
	return x;
}


void led_reset()
{
	digitalWrite(LED,LOW);
}


bool gate_is_moving()
{
	return digitalRead(SW);
}

int hc04()
{

	//TRIG pin must start LOW
    digitalWrite(TRIG, LOW);
    delay(30);
    
    
	//Send trig pulse
	digitalWrite(TRIG, HIGH);
	delayMicroseconds(20);
	digitalWrite(TRIG, LOW);

	//Wait for echo start
	while(digitalRead(ECHO) == LOW);

	//Wait for echo end
	long startTime = micros();
	while(digitalRead(ECHO) == HIGH);
	long travelTime = micros() - startTime;

	//Get distance in cm
	int distance = travelTime / 58;


	return distance;
}
void default_status()
{
	//SETUP
	wiringPiSetup();
	pinMode(LED, OUTPUT);
	pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(MOTOR1, OUTPUT);
	pinMode(MOTOR2, OUTPUT);
	pinMode(SW, INPUT);
	//VALUES
	digitalWrite(TRIG, HIGH);
	digitalWrite(LED,LOW);
	digitalWrite(MOTOR1,LOW);
	digitalWrite(MOTOR2,LOW);
}

void shut_motor()
{
	digitalWrite(MOTOR1,LOW);
	digitalWrite(MOTOR2,LOW);
}


void open_gate()
{
	int x=1;
	std::cout << "opening gate" <<std::endl;
	digitalWrite(MOTOR1,HIGH);
	digitalWrite(MOTOR2,LOW);
	delay(2500);
	std::cout << "Starting push is over" <<std::endl;
	while(1)
	{	
		digitalWrite(MOTOR1,HIGH);
		digitalWrite(MOTOR2,LOW);
		x=led_blink(x);
		if (!gate_is_moving())//the gate opened fully
		{	
			std::cout << "THE GATE IS OPEN" <<std::endl;
			shut_motor();
			std::cout << "Waiting for user to get in..." <<std::endl;
			delay(10000);//gate need to be open for some time
			break;
		}
	}
}


void close_gate()
{
	int distance,x=1;
	std::cout << "closing gate" <<std::endl;
	digitalWrite(MOTOR1,LOW);
	digitalWrite(MOTOR2,HIGH);
	delay(2500);
	std::cout << "Starting push is over" <<std::endl;
	while(1)
	{
		digitalWrite(MOTOR1,LOW);
		digitalWrite(MOTOR2,HIGH);
		x=led_blink(x);
		distance=hc04();
		delay(50);
		std::cout << "Current Distance:" << distance << std::endl;
		if (!gate_is_moving())//the gate closed fully
		{
			std::cout << "THE GATE IS CLOSED" <<std::endl;
			shut_motor();
			break;	
		}
		if (distance<60)//something on the way, we dont want to hurt it. reverse!.
		{
			shut_motor();
			delay(100);
			open_gate();
			digitalWrite(MOTOR1,LOW);
			digitalWrite(MOTOR2,HIGH);
			delay(2500);
            std::cout << "Starting push is over" <<std::endl;
		}
	}		
}


struct user
{ 
	std::string name;
	char phone_num[18];
	std::string license_plate; 
};




using namespace alpr;


const std::string MAIN_WINDOW_NAME = "ALPR main window";

const bool SAVE_LAST_VIDEO_STILL = false;
const std::string LAST_VIDEO_STILL_LOCATION = "/tmp/laststill.jpg";
const std::string WEBCAM_PREFIX = "/dev/video";
MotionDetector motiondetector;
bool do_motiondetection = true;

/** Function Headers */
std::string detectandshow(Alpr* alpr, cv::Mat frame, std::string region);

bool measureProcessingTime = false;
std::string templatePattern;

// This boolean is set to false when the user hits terminates (e.g., CTRL+C )
// so we can end infinite loops for things like video processing.
bool program_active = true;




using namespace cv;
//MAIN MAIN MAIN MAIN MAIN
int main( int argc, const char** argv )
{
	
  bool status=false;//status of gate,defult off
  default_status();
  user users[]={{"Shai Gal Netser","\"+972526481655\"","786P0J"},{"Gon Bogner","\"+972547606046\"","9431023"}};
  std::vector<std::string> filenames;
  std::string configFile = "";
  bool outputJson = false;
  int seektoms = 0;
  bool detectRegion = false;
  std::string country = "il";
  int topn;
  //bool debug_mode = true;
  
  cv::Mat frame;
  

  Alpr alpr(country, configFile);
  alpr.setTopN(topn);
  
  //alpr.getConfig()->setDebug(true);
  
  if (detectRegion)
    alpr.setDetectRegion(detectRegion);

  if (templatePattern.empty() == false)
    alpr.setDefaultRegion(templatePattern);

  if (alpr.isLoaded() == false)
  {
    std::cerr << "Error loading OpenALPR" << std::endl;
    return 1;
  }

      int webcamnumber = 0;
      

      int framenum = 0;
      cv::VideoCapture cap(webcamnumber);
      if (!cap.isOpened())
      {
        std::cerr << "Error opening webcam" << std::endl;
        return 1;
      }
      
      std::cout << "starting..."<< std::endl;
      
      std::string license_plate,last_plate="000000";
      
      int waiting_counter = 0;
      namedWindow("Gate",CV_WINDOW_NORMAL);
      
      while (cap.read(frame))// take frame from camera
      {
		cv::imshow("Gate", frame);
        if (framenum == 0)
          motiondetector.ResetMotionDetection(&frame);
        
        std::string temp_result = detectandshow(&alpr, frame, "");
                
        if ( temp_result != license_plate )
			{
				license_plate = temp_result;
				std::cout << "Detected License Plate: " << license_plate << std::endl;
				for(int use=0;use<2;use++)
				{
					if (license_plate==users[use].license_plate)
					{
						if (last_plate!=license_plate)
						{
							last_plate=license_plate;
						// 1) Gate is closed -> Open Gate
						// 2) Gate is openning -> do nothing, probably wont happen
						// 3) Gate is Open -> wait sone time, then close it
						// 4) Gate is closing -> reverse direction (meaning, openning),check if something is intterupting

						std::cout << "found user's license plate: " << users[use].name << std::endl;
						if(!gate_is_moving())//starting postion-gate is closed
						{
							open_gate();//open gate
							led_reset();
							if(main_UART(users[use].phone_num)==1)//send text messege
									std::cout << "Text messege sent!"<< std::endl;		
							close_gate();//closing gate
							led_reset();
						}
					  }
					  else
						std::cout << "Car is already inside"<< std::endl;
					}	
				}				
			}	   
	    sleep_ms(MAIN_LOOP_SLEEP_IN_MS);
        framenum++;
 	 
	} // main while
 
  return 0;
}

std::string detectandshow( Alpr* alpr, cv::Mat frame, std::string region)
{

  timespec startTime;
  getTimeMonotonic(&startTime);

  std::vector<AlprRegionOfInterest> regionsOfInterest;
  if (do_motiondetection)
  {
	  cv::Rect rectan = motiondetector.MotionDetect(&frame);
	  if (rectan.width>0) regionsOfInterest.push_back(AlprRegionOfInterest(rectan.x, rectan.y, rectan.width, rectan.height));
  }
  else regionsOfInterest.push_back(AlprRegionOfInterest(0, 0, frame.cols, frame.rows));
  AlprResults results;
  if (regionsOfInterest.size()>0)
  {
	   results = alpr->recognize(frame.data, frame.elemSize(), frame.cols, frame.rows, regionsOfInterest);
  }

  timespec endTime;
  getTimeMonotonic(&endTime);
  double totalProcessingTime = diffclock(startTime, endTime);
  if (measureProcessingTime)
    std::cout << "Total Time to process image: " << totalProcessingTime << "ms." << std::endl;
  
  std::string result;
  	
	if ( results.plates.size() )
		{
			result = results.plates[0].topNPlates[0].characters;
			std::replace(result.begin(), result.end(), '\n','-');
		}
		

 
  return result;
}

