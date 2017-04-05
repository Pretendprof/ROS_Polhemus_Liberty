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

#define BUFFER_SIZE   250

using namespace std;

// setup for Polhemus data to ROS
polhemus_connect::polhemus_pose pol_pose;
polhemus_connect::polhemus_pose pol_pose1;

// Set Vars for Polhemus
CNX_STRUCT cnxStruct;
CNX_PARAMS cp;

PingPong pong;

BYTE buf[BUFFER_SIZE];
int len=0;
int bw;

int sensorNum = 1;

int keepLooping=0;
pthread_t thread_id;
READ_WRITE_STRUCT readStruct={&pong,keepLooping,&thread_id,cnxStruct.pTrak};
READ_WRITE_STRUCT writeStruct={&pong,keepLooping,&thread_id,NULL};

// control main loop
bool running = true;

void cleanup_tracker(void) {

  cnxStruct.pTrak->CloseTrk();  // close down the tracker connection
  usleep(200000);
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
  int counter = 1;
    // Cast as char*
  std::string text( (char*)buf_ );

   // Read lines
   std::stringstream ss( text );
   std::string line;
  
    while( true ) {
      std::getline(ss, line);
      if( ss.eof() ) { break; }
      std::stringstream ss2(line);
      // Set up for two markers
      // TODO change to for loop with argv setting number of markers (may need to move publisher up here)
      if (counter == 1)
      {
      // if outputing x,y,z and euler
       ss2 >> pol_pose.id >> pol_pose.X >> pol_pose.Y >> pol_pose.Z >> pol_pose.ThetaX >> pol_pose.ThetaY >> pol_pose.ThetaZ;//
       counter = 2;
     }
     else
     {
       ss2>> pol_pose1.id >> pol_pose1.X >> pol_pose1.Y >> pol_pose1.Z >> pol_pose1.ThetaX >> pol_pose1.ThetaY >> pol_pose1.ThetaZ;
       counter = 1;
     }
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
//cout << buf;
      
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

  bool aligned = false;
  
  if (argc > 0) // see if number of markers is specified (specified by adding number of markers to file run command)
  {
     sensorNum = atoi(argv[argc-1]);
  }
  else // there is an input... currently used for polhemus offset
  {
     sensorNum = 1;
  }

  printf("starting server\n");
  atexit(cleanup_tracker);

  //Connects to Polhemus
  cnxStruct.cnxType=USB_CNX;
  cnxStruct.trackerType=TRKR_LIB_HS;
  
  // sets up PingPong Buffer
  if (pong.InitPingPong(BUFFER_SIZE)<0){
    fprintf(stderr,"Memory Allocation Error setting up buffers\n");
    return -1;
  }
  printf("1..");
  // Sets up communication with the tracker
  cnxStruct.pTrak=new PiTracker;
  if (!cnxStruct.pTrak){
    printf("Memory Allocation Error creating tracker communications module\n");
    return -3;
  }
  printf("2..");
  // Verify Polhemus connection
  if( !connect( &cnxStruct, cp ) ) {
    printf( "Could not create connection USB. Exiting \n");
    return -3;
  }

  // first establish comm and clear out any residual trash data
  do {
    cnxStruct.pTrak->WriteTrkData( (void*)"\r",1);  // send just a cr, should return an short "Invalid Command" response
    usleep(10000);
    len=cnxStruct.pTrak->ReadTrkData(buf,BUFFER_SIZE);  // keep trying till we get a response
  //  printf("establishing connection\n");
  } while (!len);

  // CONFIGURATION
  int key_return;
  key_return = 0xFF0D;

  // set output format to ascii
  cnxStruct.pTrak->WriteTrkData( (void*)"F0",2); // F sets format 0=ascii, 1=binary
  cnxStruct.pTrak->WriteTrkData(&key_return, 1);
  usleep(50000); 

  // Set units in cm  
  cnxStruct.pTrak->WriteTrkData( (void*)"U1",2); // Change to U0 for inches
  cnxStruct.pTrak->WriteTrkData(&key_return, 1);
  usleep(10000);

  // Align receptors 
  printf("Aligning receptors\n");
  int cA;
  cA = 0xFF01; //"ctrl-a" unicode
  // set offset for each receptor. The next three lines must be done for each receptor
  // change values in second line depeding on receptor locations
  cnxStruct.pTrak->WriteTrkData( &cA,1); //send ^a
  cnxStruct.pTrak->WriteTrkData( (void*)"2,0,0,78,0,0,-180",17); // sets offset for receptor 2 in cm/euler 
  cnxStruct.pTrak->WriteTrkData(&key_return, 1);
  usleep(200000); 
  
  // Launches each marker
  // Launch maker near receptor 1
  cnxStruct.pTrak->WriteTrkData( (void*) "L1",2); // F sets format 0=ascii, 1=binary
  cnxStruct.pTrak->WriteTrkData(&key_return, 1);
  usleep(1000000);

  // launch maker near receptor 2
  cnxStruct.pTrak->WriteTrkData( (void*) "L2",2); // F sets format 0=ascii, 1=binary
  cnxStruct.pTrak->WriteTrkData(&key_return, 1);
  usleep(1000000);

  // set output format
  // see p.39 of Polhemus liberty manual for details. timestamp at end (8) can be published
  
  // x,y,z and quaternion, 
  //cnxStruct.pTrak->WriteTrkData( (void*)"O*,2,7,1",8);

  // x,y,z and Euler
  cnxStruct.pTrak->WriteTrkData( (void*)"O*,2,4,1",8);
  cnxStruct.pTrak->WriteTrkData(&key_return, 1);
  usleep(100000);

  // this provides output feedback to confirm setup
  printf("verify ouput: \n");
  readPol();
  cout<<buf;
  printf("\nIf output has correct number of sensors/format, system is ready\n");

  // Set up ROS node
  ros::init(argc, argv, "polhemus_data");
  ros::NodeHandle np("~");
  // set rate to hz (e.g. 188 = arm updates at 188hz)
  ros::Rate rate(188); 
  //Set up publisher for Polhemus Data
  ros::Publisher polhemus_pub = np.advertise<polhemus_connect::polhemus_pose>("pol_output", 3);
  ros::Publisher polhemus_pub1 = np.advertise<polhemus_connect::polhemus_pose>("pol_output1", 3);
  
  // Main loop
  while(running)
  {
    readPol();
    polhemus_pub.publish(pol_pose);
    polhemus_pub1.publish(pol_pose1);
    ros::spinOnce();  

  }  
  
  printf("\nexiting\n");
  
  return 0;
}
