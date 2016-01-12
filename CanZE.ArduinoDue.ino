// used to easily swap between the native (fast) and the default (slow) port
// on the Arduino Due .. or any other port of course ;-)
#define Output SerialUSB
#define Bluetooth Serial1
#define BT_SPEED 1311600

// define what features to use
// comment out if you don't need them
#define USE_BT  1
#define FORMAT "bob"
//#define ANALYSE  1


// load the SPI library (needed for the CAN, SD & LCD library)
#include <SPI.h>

// load the CAN library
#include "variant.h"
#include <due_can.h>   


#include "structs.h"


CAN_FRAME* dataArray = 0;
int dataArraySize = 0;

CAN_FRAME EMPTY;

ISO_MESSAGE isoMessage;

boolean outputReceivedFrames = 0;

long T = millis();




// variables needed to count the number of messages per second
long unsigned count = 0;  // number of received messages
double totalRate = 0;          // reception rate based on millis()
double filteredRate = 0;          // reception rate based on millis()
#define LIMIT 1600
int saves = 0;

// read buffer
String readBuffer = "";
String filter = "";

void setup()
{
  // init EMPTY frame
  EMPTY.length=8;
  for(int i=0; i<EMPTY.length; i++)
    EMPTY.data.bytes[i]=0;
  
  // wait for the serial connection to come up
  //while (!Output);
  // initialise the serial connection
  Output.begin(921600);
  //Output.begin(115200);

  #ifdef USE_BT
    Bluetooth.begin(BT_SPEED);
  #endif
  
  // Initialize CAN0 and CAN1, Set the proper baud rates here
  Can0.begin(CAN_BPS_500K);
  //Can1.begin(CAN_BPS_500K);
  
  setFilter(0);
}

void setFilter(int filter)
{
  //By default there are 7 mailboxes for each device that are RX boxes
  //This sets each mailbox to have an open filter that will accept extended
  //or standard frames

  //extended
  for (filter = 0; filter < 3; filter++) {
    Can0.setRXFilter(filter, 0, 0, true);
    //Can1.setRXFilter(filter, 0, 0, true);
  }  
  //standard
  for (int filter = 3; filter < 7; filter++) {
    Can0.setRXFilter(filter, 0, 0, false);
    //Can1.setRXFilter(filter, 0, 0, false);
  }   
}

String getHex(int num)
{
  String stringOne =  String(num, HEX); 
  if(stringOne.length()<2) stringOne="0"+stringOne;
  return stringOne;
}

String getHexSimple(int num)
{
  String stringOne =  String(num, HEX); 
  return stringOne;
}

void loop()
{
  CAN_FRAME incoming;

  if (Can0.available() > 0) 
  {
    Can0.read(incoming); 
    #ifdef ANALYSE
      processFrame(incoming,0);
    #endif
    storeFrame(incoming);
  }
  /*if (Can1.available() > 0) 
  {
    Can1.read(incoming); 
    #ifdef ANALYSE
      processFrame(incoming,1);
    #endif
    storeFrame(incoming);
  }*/

  readIncoming();
}


void readIncoming()
{
  // Output
  if(Output.available())
  {
    char ch = Output.read();
    if(ch=='\n' || ch=='\r')
    {
      if(readBuffer!="")
      {
        processCommand(readBuffer);
        readBuffer="";        
      }
    }
    else
    {
      readBuffer+=ch;
    }
  }
  /**/
  // Bluetooth
  if(Bluetooth.available())
  {
    char ch = Bluetooth.read();
    if(ch=='\n' || ch=='\r')
    {
      if(readBuffer!="")
      {
        processCommand(readBuffer);
        readBuffer="";        
      }
    }
    else
    {
      readBuffer+=ch;
    }
    
  }  
}


COMMAND decodeCommand(String &input)
{
  COMMAND result;

  // trim whitespaces
  input.trim();

  // stop if input is empty
  if(input.length()==0) return result;
  
  // the first letter is the command
  result.cmd = input.charAt(0);
  input.remove(0,1); 

  // if there is something more,
  if(input.length()!=0)
  {
    // get the ID
    char ch;
    String id = "";
    do
    {
      ch=input.charAt(0);
      if(ch!=',') id+=ch;
      input.remove(0,1); 
    }
    while(input.length()!=0 && ch!=',');
    result.id=hexToDec(id);
  }
  
  // if there is something more,
  if(input.length()!=0)
  {
      // get the REQUEST
      char ch;
      String request = "";
      do
      {
        ch=input.charAt(0);
        if(ch!=',') request+=ch;
        input.remove(0,1); 
      }
      while(input.length()!=0 && ch!=',');
      for(int i=0; i<request.length(); i+=2)
      {
        result.request[result.requestLength]=hexToDec(request.substring(i,i+2));
        result.requestLength++;
      }
  }

  // if there is something more,
  if(input.length()!=0)
  {
      // get the REPLY
      char ch;
      String reply = "";
      do
      {
        ch=input.charAt(0);
        if(ch!=',') reply+=ch;
        input.remove(0,1); 
      }
      while(input.length()!=0 && ch!=',');
      for(int i=0; i<reply.length(); i+=2)
      {
        result.reply[result.replyLength]=hexToDec(reply.substring(i,i+2));
        result.replyLength++;
      }
  }
  return result;
}

void printCommand(COMMAND &command)
{
  // id
  Output.println(String(command.cmd)+".id      = "+getHexSimple(command.id));
  // request
  Output.print(String(command.cmd)+".request = ");
  for(int i=0; i<command.requestLength; i++)
    Output.print(getHex(command.request[i])+" ");
  Output.println();
  // reply
  Output.print(String(command.cmd)+".reply   = ");
  for(int i=0; i<command.replyLength; i++)
    Output.print(getHex(command.reply[i])+" ");
  Output.println();
}

String frameToBOB(CAN_FRAME &frame)
{
  String dataString = String(frame.id,HEX)+",";
  for(int i = 0; i<frame.length; i++) 
  {
    dataString+=getHex(frame.data.bytes[i]);
  }  
  dataString+="\n";
  return dataString;
}

String messageToBOB(ISO_MESSAGE &message)
{
  String dataString = String(message.id,HEX)+",";
  for(int i = 0; i<message.length; i++) 
  {
    dataString+=getHex(message.data[i]);
  }  
  dataString+=",";
  for(int i = 0; i<message.replyLength; i++) 
  {
    dataString+=getHex(message.reply[i]);
  }  
  dataString+="\n";
  return dataString;
}

void processCommand(String &line)
{
  COMMAND command = decodeCommand(line);
  //printCommand(command);
  
  if(command.cmd=='f')
  {
    /*String id = getHexSimple(command.id);
    int pos = filter.indexOf(id);
    // only add if not yet there
    if(pos==-1)
      filter+=id+",";*/
  }
  // remove filter
  else if(command.cmd=='r')
  {
    String id = getHexSimple(command.id);
    int pos = filter.indexOf(id);
    // only remove if present
    if(pos!=-1)
      filter.remove(pos,id.length()+1);
  }
  // clear filter
  else if(command.cmd=='c')
  {
    filter="";
  }
  // all frames
  else if(command.cmd=='a')
  {
    Output.println("Framecount = "+String(dataArraySize));
    for(int i=0; i<dataArraySize; i++)
    {
      CAN_FRAME frame = dataArray[i];
      String sendData = frameToOutput(frame);
      Output.print(sendData);
      #ifdef USE_BT
        Bluetooth.print(sendData);
      #endif
    }
  }
  // get frame
  else if(command.cmd=='g')
  {
    CAN_FRAME frame = getFrameById(command.id);
    String sendData = frameToOutput(frame);
    Output.print(sendData);
    #ifdef USE_BT
      Bluetooth.print(sendData);
    #endif
  }
  // send a request and wait for the response
  else if(command.cmd=='i')
  {
     // only accept this command if the requested ID belongs to an ISO-TP frame
     if(command.id>0x700)
     {
        //Output.println("Request for: "+getHex(command.id));
        // store ID
        isoMessage.id=command.id;
        isoMessage.requestId = getRequestId(command.id);
        // store reply
        isoMessage.replyLength=command.replyLength;
        for(int i=0; i<command.replyLength; i++)
        isoMessage.reply[i]=command.reply[i];
        // store request
        isoMessage.requestLength=command.requestLength;
        for(int i=0; i<command.requestLength; i++)
        isoMessage.request[i]=command.request[i];

        if(isoMessage.requestId>0)
        {
          // buidl the CAN frame
          CAN_FRAME frame;
          // set the ID
          frame.id=isoMessage.requestId;
          // set the length
          frame.length=command.requestLength+1;
          frame.length=8;
          // zero out frame
          for(int i=0; i<8; i++)
            frame.data.bytes[i]=0;
          // fill up the first byte 0x00 & requestLength
          frame.data.bytes[0]=command.requestLength;
          // fill up the other bytes
          for(int i=0; i<command.requestLength; i++)
            frame.data.bytes[i+1]=command.request[i];
          // send the frame
          //Output.println("Send: "+frameToBOB(frame));

          int OK = 0;
          do {
            OK=Can0.sendFrame(frame);
          } while(OK!=1);
          
          // activate filter
          Can0.watchFor(isoMessage.id);
          setFilter(isoMessage.id);

          T = millis();
          // timing
          //Output.println("Request for: "+getHex(command.id));
          Output.println("--------------------------------------------------------------------------------");
          Output.print(String(millis()-T)+" > REQU: "+frameToBOB(frame));
          //Output.println(String(millis()-T)+" > REQU: "+getHex(command.id));
          //Can1.sendFrame(frame);
          // --> any incoming frames with the given id will be handeled by "storeFrame" and send off if complete

          //delay(4);
          //sendFlowControlFrame(isoMessage);

          // request the remaining 0x2 frames
          /*
          delay(10);
          CAN_FRAME flow;
          flow.id=isoMessage.requestId;
          flow.length=8;
          flow.data.bytes[0] = 0x30;  // type 3, clear to send
          flow.data.bytes[1] = 0x00;  // no flow control
          flow.data.bytes[2] = 0x00;  // no delay between frames
          flow.data.bytes[3] = 0; // fill-up 
          flow.data.bytes[4] = 0; // fill-up
          flow.data.bytes[5] = 0; // fill-up
          flow.data.bytes[6] = 0; // fill-up
          flow.data.bytes[7] = 0; // fill-up
      
          OK = 0;
          do {
            OK=Can0.sendFrame(flow);
          } while(OK!=1);
      
          Output.print(String(millis()-T)+" > FLOW: "+frameToBOB(flow));
          */
        
        }
     }
  }
  else if(command.cmd=='t')
  {
     CAN_FRAME frame;
     frame.id=command.id;
     frame.length=command.requestLength;
     for(int i=0; i<command.requestLength; i++)
        frame.data.bytes[i]=command.request[i];
     //Output.println(frameToBOB(frame));
     storeFrame(frame);
  }
  // debug filters to output
  else if(command.cmd=='o')
  {
    #ifdef USE_BT
      Output.println("Filters: "+filter);
    #endif
    Bluetooth.println("Filters: "+filter);
  }
  // get frame rate
  else if(command.cmd=='s')
  {
      Output.println("Frame Rate: "+String(totalRate));
      Bluetooth.println("Frame Rate: "+String(totalRate));

      Output.println("Filtered Frame Rate: "+String(filteredRate));
      Bluetooth.println("Filtered Frame Rate: "+String(filteredRate));
  }
  // toggle output
  else if(command.cmd=='l')
  {
      outputReceivedFrames=!outputReceivedFrames;
      Output.print("outputReceivedFrames is now ");
      if(outputReceivedFrames)
        Output.println("ON");
      else
        Output.println("OFF");
  }
}

unsigned int hexToDec(String hexString) {
  
  unsigned int decValue = 0;
  int nextInt;
  
  for (int i = 0; i < hexString.length(); i++) {
    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }
  
  return decValue;
}

//Get the value of XOR'ing all the bytes together. This creates a reasonable checksum that can be used
//to make sure nothing too stupid has happened on the comm.
uint8_t checksumCalc(uint8_t *buffer, int length) 
{
  uint8_t valu = 0;
  for (int c = 0; c < length; c++) {
    valu ^= buffer[c];
  }
  return valu;
}

void addFrame(CAN_FRAME &frame)
{
  // create new array
  dataArraySize++;
  CAN_FRAME *temp = new CAN_FRAME [dataArraySize];
  // copy data
  for(int i=0; i<dataArraySize-1; i++)
    temp[i]=dataArray[i];
  // delete old array
  if (dataArray != 0) {
      delete [] dataArray;
  }
  // switch reference    
  dataArray = temp;
  dataArray[dataArraySize-1]=frame;
}

int findFrameById(int id)
{
  for(int i=0; i<dataArraySize; i++)
  {
    if(dataArray[i].id==id) return i;
  }
  return -1;
}


void storeFrame(CAN_FRAME &frame)
{
  count++;
  totalRate = (count/(millis()/1000.));

  // only process frame if not filtered out or no active filter
  String idHex = String(frame.id,HEX);
  if(filter.length()==0 || filter.indexOf(idHex)!=-1)
  {
    filteredRate = (count/(millis()/1000.));

    // free data stream is < 0x700
    if(frame.id<0x700)
    {
      int index = findFrameById(frame.id);
      if(index==-1) addFrame(frame);
      else dataArray[index]=frame;
    }
    // if there is content and this is the frame we are waiting for
    else if(frame.length>0 && frame.id==isoMessage.id)
    {
      //Output.print(frameToBOB(frame));    
      //Output.println("Got ISO-TP frame ...");
      //frameToBOB(frame);
      //Output.print(frameToBOB(frame));
      
      // id = first nibble
      uint8_t type = frame.data.bytes[0] >> 4;
      //Output.println("Type = "+String(type));
      // single frame answer
      if(type==0x0) {
          //Output.print(String(millis()-T)+" > UNIQ: "+frameToBOB(frame));

          String dataString = frameToBOB(frame);
          dataString.trim();
          dataString+=",";
          for(int i = 0; i<isoMessage.replyLength; i++) 
          {
            dataString+=getHex(isoMessage.reply[i]);
          }  
          dataString+="\r\n";
          
          Output.println(dataString);
          #ifdef USE_BT
            Bluetooth.print(dataString);
          #endif       
      }
      // first frame of a multi-framed message
      else if(type==0x1) {
        Output.print(String(millis()-T)+" > FIRS: "+frameToBOB(frame));
  
        // length = second nibble + second byte
        uint16_t messageLength = frame.data.bytes[0] & 0x0f;
        messageLength <<= 8;
        messageLength |= frame.data.bytes[1];
        //Output.println("Length "+String(messageLength));
        // reply ID = third & fourth byte
        uint32_t frameReply = 0;
        uint32_t reply = 0;
        for(int i=0; i<isoMessage.replyLength; i++)
        {
          frameReply <<= 16;
          frameReply |=  frame.data.bytes[i+2];
          reply <<=16;
          reply |= isoMessage.reply[i];
        }
  
        // is this the message we are waiting for?
        if(frameReply == reply)
        {
          //sendFlowControlFrame(isoMessage);
          /**/     
          // request the remaining 0x2 frames
          CAN_FRAME flow;
          flow.id=isoMessage.requestId;
          flow.length=8;
          flow.data.bytes[0] = 0x30;  // type 3, clear to send
          flow.data.bytes[1] = 0x00;  // no flow control
          flow.data.bytes[2] = 0x00;  // no delay between frames
          flow.data.bytes[3] = 0; // fill-up 
          flow.data.bytes[4] = 0; // fill-up
          flow.data.bytes[5] = 0; // fill-up
          flow.data.bytes[6] = 0; // fill-up
          flow.data.bytes[7] = 0; // fill-up
      
          int OK = 0;
          do {
            OK=Can0.sendFrame(flow);
          } while(OK!=1);
      
          Output.print(String(millis()-T)+" > FLOW: "+frameToBOB(flow));
          /**/
      
                //Can0.sendFrame(flow);
          //Can1.sendFrame(flow);
          //Output.print("SEND: "+frameToBOB(flow));
  
          // build new iso message
          // set id
          isoMessage.id = frame.id;
          // set final length
          isoMessage.length = messageLength;
          // init data
          uint8_t* data = new uint8_t[messageLength];
          for(int i=0; i<messageLength; i++) data[i]=0;
          isoMessage.data = data;
          // init sequence
          isoMessage.next = 1;
          // fill up data
          isoMessage.index=0;
          for(int i=2; i<frame.length; i++)
          {
            if(isoMessage.index<isoMessage.length)
            {
              isoMessage.data[isoMessage.index]=frame.data.bytes[i];
              isoMessage.index++;
            }
          }
      
          //Output.println("DEBUG: "+messageToBOB(isoMessage));
          //Output.println("End of 0x1 case");
        }
     }
     // consecutive frames
     else if(type == 0x2)
     {
         Output.print(String(millis()-T)+" > NEXT: "+frameToBOB(frame));
          
        //Output.println("Start of 0x2 case");
        //Output.println(frameToBOB(frame));
  
        uint16_t sequence = frame.data.bytes[0] & 0x0f; 
        if(isoMessage.id == frame.id &&
           isoMessage.next == sequence)
        {
            for(int i=1; i<frame.length; i++)
            {
              if(isoMessage.index<isoMessage.length)
              {
                isoMessage.data[isoMessage.index]=frame.data.bytes[i];
                isoMessage.index++;
              }
            }
      
            //Output.println("Index: "+String(isoMessage.index));
            //Output.println("Length: "+String(isoMessage.length));
      
            // wait for next message
            isoMessage.next=(isoMessage.next+1)%16;
            //if(isoMessage.next==16) isoMessage.next=1;
  
            //Output.println(String(isoMessage.index)+" vs. "+String(isoMessage.length));
            // is this the last part?
            if(isoMessage.index+1>=isoMessage.length) // for some frames one byte is missing :-?
            {
              String dataString = messageToBOB(isoMessage);
              Output.print(dataString);
              Output.println("*************************************************************************************");
              #ifdef USE_BT
                Bluetooth.print
                (dataString);
              #endif
      
              // reset filters
              //Can0.watchFor();
              //Can1.watchFor();
      
              // cancel this message
              isoMessage.id=-1;
            }
            //Output.println("End of 0x2 case");
            //Output.println("DEBUG: "+messageToBOB(isoMessage));
          }
  
       }
    }  
  }
}

void sendFlowControlFrame(ISO_MESSAGE isoMessage)
{
    // request the remaining 0x2 frames
    CAN_FRAME flow;
    flow.id=isoMessage.requestId;
    flow.length=8;
    flow.data.bytes[0] = 0x30;  // type 3, clear to send
    flow.data.bytes[1] = 0x00;  // no flow control
    flow.data.bytes[2] = 0x00;  // no delay between frames
    flow.data.bytes[3] = 0; // fill-up 
    flow.data.bytes[4] = 0; // fill-up
    flow.data.bytes[5] = 0; // fill-up
    flow.data.bytes[6] = 0; // fill-up
    flow.data.bytes[7] = 0; // fill-up

    int OK = 0;
    do {
      OK=Can0.sendFrame(flow);
    } while(OK!=1);

    Output.print(String(millis()-T)+" > FLOW: "+frameToBOB(flow));
}

CAN_FRAME getFrameById(int id)
{
  int index = findFrameById(id);
  if (index==-1) return EMPTY;
  else return dataArray[index];
}

String frameToOutput(CAN_FRAME &frame)
{
  String dataString = String(frame.id,HEX)+",";
  for(int i = 0; i<frame.length; i++) 
  {
    dataString+=getHex(frame.data.bytes[i]);
  }
  dataString+="\r\n";
  return dataString;
}

void processFrame(CAN_FRAME &frame, int whichBus) 
{
    count++;
    totalRate = (count/(millis()/1000.));

    String idHex = String(frame.id,HEX);

    // only process frame if not filtered out or no active filter
    if(filter.length()==0 || filter.indexOf(idHex)!=-1)
    {
      filteredRate = (count/(millis()/1000.));
      
      if(FORMAT=="gvret")  
      {
        uint8_t buff[40];
        uint8_t temp;
        uint32_t now = micros();

        if (frame.extended) frame.id |= 1 << 31;
        buff[0] = 0xF1;
        buff[1] = 0; //0 = canbus frame sending
        buff[2] = (uint8_t)(now & 0xFF);
        buff[3] = (uint8_t)(now >> 8);
        buff[4] = (uint8_t)(now >> 16);
        buff[5] = (uint8_t)(now >> 24);
        buff[6] = (uint8_t)(frame.id & 0xFF);
        buff[7] = (uint8_t)(frame.id >> 8);
        buff[8] = (uint8_t)(frame.id >> 16);
        buff[9] = (uint8_t)(frame.id >> 24);
        buff[10] = frame.length + (uint8_t)(whichBus << 4);
        for (int c = 0; c < frame.length; c++)
        {
          buff[11 + c] = frame.data.bytes[c];
        }
        temp = checksumCalc(buff, 11 + frame.length);
        buff[11 + frame.length] = temp;

        // send it to the serial line
        Output.write(buff, 12 + frame.length);          

        // send it to bluetooth
        #ifdef USE_BT
          //Bluetooth.write(buff, 12 + frame.length); 
        #endif
      }
      else
      {
        String dataString = "";
        // now let's build the output string
        if(FORMAT=="bob")  
          dataString+=idHex+",";
        else // CRTD
          dataString+=String(micros())+" R11 "+idHex+" ";
        
        // add the data
        //uint8_t chk = 0;
        
        for(int i = 0; i<frame.length; i++) 
        {
            if(FORMAT=="bob")  
              dataString+=getHex(frame.data.bytes[i]);
            else
              dataString+=getHex(frame.data.bytes[i])+" ";

            //chk ^= frame.data.bytes[i];
        }
        // append "rate"
        //dataString+=","+String(rate,2);

        //dataString+=","+getHex(chk)+"|";
        dataString+="\r\n";
        
        // send it to the serial line
        Output.print(dataString);
  
        // send it to bluetooth
        #ifdef USE_BT
          //Bluetooth.print(dataString);
        #endif
      }  
      
      
      // data logging to the SD card
      #ifdef USE_SD
        if(isCardPresent)
        {
          if (dataFile) 
          {
            dataFile.println(dataString);
          }
        }
      #endif
    }
}

uint16_t getRequestId(uint16_t responseId)
{                     //from        // to
  if     (responseId==0x7ec) return 0x7e4;  // EVC
  else if(responseId==0x7da) return 0x7ca;  // TCU
  else if(responseId==0x7bb) return 0x79b;  // LBC
  else if(responseId==0x77e) return 0x75a;  // PEB
  else if(responseId==0x772) return 0x752;  // Airbag
  else if(responseId==0x76d) return 0x74d;  // UDP
  else if(responseId==0x763) return 0x743;  // instrument panel
  else if(responseId==0x762) return 0x742;  // PAS
  else if(responseId==0x760) return 0x740;  // ABS
  else if(responseId==0x7bc) return 0x79c;  // UBP
  else if(responseId==0x765) return 0x745;  // BCM
  else if(responseId==0x764) return 0x744;  // CLIM
  else if(responseId==0x76e) return 0x74e;  // UPA
  else if(responseId==0x793) return 0x792;  // BCB
  else if(responseId==0x7b6) return 0x796;  // LBC2
  else if(responseId==0x722) return 0x702;  // LINCH
  else 
  {
    Output.println("NOT KNOW FROM: "+getHex(responseId));
    return -1;
  }
}



