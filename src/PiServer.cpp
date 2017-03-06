#include <pthread.h>
#include <PiTracker.h>
#include <PingPong.h>
#include <PiTerm.h>
#include <iostream>
#include <cstdio>

#include <unistd.h>
#include<libusb-1.0/libusb.h>
#include<iostream>

#include<ros/ros.h>
#include<polhemus_connect/polhemus_pose.h>

#include <visualization_msgs/Marker.h>
#include <signal.h>

#define BUFFER_SIZE   1000

using namespace std;

// setup for Polhemus data to ROS
polhemus_connect::polhemus_pose pol_pose;

// Set Vars for Polhemus
CNX_STRUCT cnxStruct;
CNX_PARAMS cp;

PingPong pong;

BYTE buf[BUFFER_SIZE];
int len=0;
int bw;

int keepLooping=0;
pthread_t thread_id;
READ_WRITE_STRUCT readStruct={&pong,keepLooping,&thread_id,cnxStruct.pTrak};
READ_WRITE_STRUCT writeStruct={&pong,keepLooping,&thread_id,NULL};

// control main loop
bool running = true;

void cleanup_tracker(void) {

  cnxStruct.pTrak->CloseTrk();  // close down the tracker connection
  usleep(2000);
  delete cnxStruct.pTrak;
}

//Sets connection to Tracker
bool connect(LPCNX_STRUCT _pcs, CNX_PARAMS _cp){

  int cnxSuccess;
  char* str;
  char* port;

  if (_pcs->cnxType==USB_CNX){
    do {
      cnxSuccess=_pcs->pTrak->UsbConnect(usbTrkParams[_pcs->trackerType].vid,usbTrkParams[_pcs->trackerType].pid,
				      usbTrkParams[_pcs->trackerType].writeEp,usbTrkParams[_pcs->trackerType].readEp);
      printf("Connecting to %s over USB \n",  trackerNames[_pcs->trackerType] );

    } while (cnxSuccess!=0);

     printf("Connected to %s over USB \n", trackerNames[_pcs->trackerType] );
  }

  return true;

}

// TODO Cannot get prog to respect SIGINT
void sigintHandler(int a)
{
   running = false;
   return;
}

// parses data from Polhemus for publishing
bool sendMsg(BYTE buf_[BUFFER_SIZE])
{
    // Cast as char*
  std::string text( (char*)buf_ );

   // Read lines
   std::stringstream ss( text );
   std::string line;
  
    while( true ) {
      std::getline(ss, line);
      if( ss.eof() ) { break; }
      std::stringstream ss2(line);
      // if outputing x,y,z and euler
      ss2 >> pol_pose.id >> pol_pose.X >> pol_pose.Y >> pol_pose.Z >> pol_pose.ThetaX >> pol_pose.ThetaY >> pol_pose.ThetaZ;
      // if outputing x,y,z and Quarternions
      //ss2 >> pol_pose.id >> pol_pose.X >> pol_pose.Y >> pol_pose.Z >> pol_pose.ThetaX >> pol_pose.ThetaY >> pol_pose.ThetaZ >> pol_pose.ThetaW;
    }    

   return false;
}

// reads current Polhemus data and sends for parsing
void readPol()
{
    while (true){

    // Write command
    cnxStruct.pTrak->WriteTrkData( (void*)"p",1);
    usleep(1000);
    len=cnxStruct.pTrak->ReadTrkData(buf,BUFFER_SIZE);  

    if (len>0 && len<BUFFER_SIZE){

      buf[len]=0;  // null terminate
      do {
	bw=pong.WritePP(buf,len);  // write to buffer
	usleep(1000);
      
      }while(!bw);
    }

 // Read buffer
    len = pong.ReadPP(buf);
 
    while(len) {
      buf[len] = 0;
      len = pong.ReadPP(buf);
    if( !sendMsg(buf) ) { return; };
    }
   
  }

   usleep(1000); // rest for 10ms
  return;
}

// Main
int main(int argc,char **argv){
  // TODO Need to get program to exit cleanly with SIGINT command (^c) in terminal
  signal(SIGINT, sigintHandler);

  atexit(cleanup_tracker);

  //Connects to Polhemus
  cnxStruct.cnxType=USB_CNX;
  cnxStruct.trackerType=TRKR_LIB;
  
  // sets up PingPong Buffer
  if (pong.InitPingPong(BUFFER_SIZE)<0){
    fprintf(stderr,"Memory Allocation Error setting up buffers\n");
    return -1;
  }
  
  // Sets up communication with the tracker
  cnxStruct.pTrak=new PiTracker;
  if (!cnxStruct.pTrak){
    printf("Memory Allocation Error creating tracker communications module\n");
    return -3;
  }

  // Verify Polhemus connection
  if( !connect( &cnxStruct, cp ) ) {
    printf( "Could not create connection USB. Exiting \n");
    return -3;
  }

  // first establish comm and clear out any residual trash data
  do {
    cnxStruct.pTrak->WriteTrkData( (void*)"\r",1);  // send just a cr, should return an short "Invalid Command" response
    usleep(100000);
    len=cnxStruct.pTrak->ReadTrkData(buf,BUFFER_SIZE);  // keep trying till we get a response

  } while (!len);


    // CONFIGURATION
  int key_return;
  key_return = 0xFF0D;
  
  // Set units in cm  
  cnxStruct.pTrak->WriteTrkData( (void*)"U1",2); // Change to U0 for inches
  cnxStruct.pTrak->WriteTrkData(&key_return, 1);
  usleep(100000);

  // x,y,z and quaternion, return carriage see p.39 of Polhemus liberty manual for details timestamp at end (8) can be published
  //cnxStruct.pTrak->WriteTrkData( (void*)"O*,2,7,1",8);
  // x,y,z and Euler, return carriage
  cnxStruct.pTrak->WriteTrkData( (void*)"O*,2,4,1",8);
  cnxStruct.pTrak->WriteTrkData(&key_return, 1);
  usleep(100000);

  // Set up ROS node
  ros::init(argc, argv, "polhemus_data");
  ros::NodeHandle np("~");
  // set rate to hz (e.g. 50 = arm updates at 50hz)
  ros::Rate rate(200);
  //Set up publisher for Polhemus Data
  ros::Publisher polhemus_pub = np.advertise<polhemus_connect::polhemus_pose>("pol_output", 100);
  
  // Main loop
  while(running)
  {
    readPol();
    polhemus_pub.publish(pol_pose);
    ros::spinOnce();  
  }  
  
  printf("\nexiting\n");
  
  return 0;
}
