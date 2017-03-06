#include <pthread.h>
#include <PiTracker.h>
#include <PingPong.h>
#include <PiTerm.h>
#include <iostream>
#include <cstdio>

#include <unistd.h>
//#include <libusb.h>
#include<libusb-1.0/libusb.h>
#include<iostream>

#include<ros/ros.h>

#define BUFFER_SIZE   1000

using namespace std;

CNX_STRUCT cnxStruct;
CNX_PARAMS cp;

PingPong pong;

// from PiTerm Main loop
  int keepLooping=0;

  pthread_t thread_id;
  READ_WRITE_STRUCT readStruct={&pong,keepLooping,&thread_id,cnxStruct.pTrak};
  READ_WRITE_STRUCT writeStruct={&pong,keepLooping,&thread_id,NULL};  // will add textview after it's crea

void cleanup_tracker(void) {

  cnxStruct.pTrak->CloseTrk();  // close down the tracker connection
  usleep(2000);
  delete cnxStruct.pTrak;
}

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


// This is the thread that reads data from the tracker and stores in the
// ping pong buffer
void* ReadTrackerThread(void* pParam){

  BYTE buf[BUFFER_SIZE];
  LPREAD_WRITE_STRUCT prs=(LPREAD_WRITE_STRUCT)pParam;
  PiTracker* pTrak=(PiTracker*)prs->pParam;
  int len=0;
  int bw;

  // first establish comm and clear out any residual trash data
  do {
    pTrak->WriteTrkData((void*)"\r",1);  // send just a cr, should return an short "Invalid Command" response
    usleep(100000);
    len=pTrak->ReadTrkData(buf,BUFFER_SIZE);  // keep trying till we get a response
  } while (!len);


  while (prs->keepLooping){

    len=pTrak->ReadTrkData(buf,BUFFER_SIZE);  // read tracker data
    if (len>0 && len<BUFFER_SIZE){
      buf[len]=0;  // null terminate
      do {
	bw=prs->pPong->WritePP(buf,len);  // write to buffer
	usleep(1000);
      }while(!bw);
    }
    usleep(2000);  // rest for 2ms
  }

  return NULL;
}

void readPol()
{

 // Start reading
  BYTE buf[BUFFER_SIZE];
  int len=0;
  int bw;

    // first establish comm and clear out any residual trash data
  do {
    cnxStruct.pTrak->WriteTrkData( (void*)"\r",1);  // send just a cr, should return an short "Invalid Command" response
    usleep(100000);
    len=cnxStruct.pTrak->ReadTrkData(buf,BUFFER_SIZE);  // keep trying till we get a response

  } while (!len);


    // CONFIGURATION
  int key_return;
  key_return = 0xFF0D;
  
//  // Set units in cm  
    cnxStruct.pTrak->WriteTrkData( (void*)"U1",2);
    cnxStruct.pTrak->WriteTrkData(&key_return, 1);
    usleep(100000);

    while (true){

    // Write command
    cnxStruct.pTrak->WriteTrkData( (void*)"p",1);
    usleep(100000);
    len=cnxStruct.pTrak->ReadTrkData(buf,BUFFER_SIZE);  

    if (len>0 && len<BUFFER_SIZE){
      buf[len]=0;  // null terminate
      do {
	bw=pong.WritePP(buf,len);  // write to buffer
	usleep(1000);
       
      }while(!bw);
    }

 //    Read buffer and publish on terminal
    len = pong.ReadPP(buf);
    while(len) {
      buf[len] = 0;
      len = pong.ReadPP(buf);
       cout<<"a "<<buf;
      //if( !sendMsg(buf) ) { printf("Error sending to channel \n"); return -1; };
    }
  }

   usleep(10000); // rest for 10ms
}

int main(int argc,char* argv[]){
  
  atexit(cleanup_tracker);
  //Connects to Polhemus
  cnxStruct.cnxType=USB_CNX;
  cnxStruct.trackerType=TRKR_LIB;

  if (pong.InitPingPong(BUFFER_SIZE)<0){
    fprintf(stderr,"Memory Allocation Error setting up buffers\n");
    return -1;
  }
  
  // The communication with the tracker
  cnxStruct.pTrak=new PiTracker;
  if (!cnxStruct.pTrak){
    printf("Memory Allocation Error creating tracker communications module\n");
    return -3;
  }

    // Create connection
  if( !connect( &cnxStruct, cp ) ) {
    printf( "Could not create connection USB. Exiting \n");
    return -3;
  }
  
  // Setup Ros Node

  // need to cha+
  readPol();
}
