//FRED Robot
//Langara Science Fair workshop project
//Description: This is the source code for the arduino based
//Lego robot for the Langara Science Fair
//Interested parties are free to modify and use as the code as they see fit.
//The original author does not guarantee the fitness of the code in any way.
//Copyright 2017
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <SPI.h>
#include <WiFi.h>
#include <string.h>
#include <stdlib.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

char  net[] = "Project";
char pass[] = "catbus12";  //sketchy, but what else are you going to do.
//WiFi related stuff
//char  net[] = "WiFiProject2";
//char pass[] = "FreeCoffee";  //sketchy, but what else are you going to do.
char botName[] = "Fred001"; // rebot name, different for each bot.
//char botName[] = "Giny";
//char botName[] = "George";
IPAddress ip(192, 168, 0, 150); 
// source code nightmare ahead.
char getCommand[] = "GET";
WiFiServer server(80);  //Use port 80, the parameter is the port number

//Pin related bumper emergency stop
int buttonPin = 3;

int incomingByte;
//Class representing a command
class command { //anatomy of a command
  public:
    char commandChar;
    word power; //between 0-255
    word duration; //upto 9999ms or more? 16,000 ms? max word?
    //Debug helper, prints the command to the serial port
    void print() {
      Serial.println("Command");
      Serial.println(commandChar);
      Serial.println("power");
      Serial.println(power);
      Serial.println("duration");
      Serial.println(duration);
    }
};

//List of commands
class commandBuffer {
  public:
    commandBuffer(int bufferSize);
    ~commandBuffer();
     int currentCommand;
    bool addCommand(char commandChar, word power, word duration) {
      Serial.println("Adding Command");
      if (maxCommands > numCommands) {
        buff[numCommands].commandChar = commandChar;
        buff[numCommands].power = power;
        buff[numCommands].duration = duration;
        numCommands += 1;
        return true;
      } else {
        return false; //buffer is full
      }
    }
    int length() {
      return numCommands;
    }
    void reset() {
      numCommands = 0;
      currentCommand = -1;
    }
    command & getCurCommand() {
      return buff[currentCommand];
    }
    bool endReached() {
      return currentCommand >= numCommands - 1;
    }
    bool advanceCommand() {
      if (currentCommand < numCommands - 1) {
        currentCommand += 1;
        return true;
      } else {
        return false;
      }
    }
  private:
    int numCommands;
    int maxCommands;
    command * buff;
   

};
commandBuffer::commandBuffer(int bufferSize) {
  buff =  new command[bufferSize];
  numCommands = 0;
  maxCommands = bufferSize;
  currentCommand = -1;
}
commandBuffer::~commandBuffer() {
  delete[] buff;
  buff = NULL;
}
//Fixed size buffer to hold the commands
//only one set of commands at a time for now.
commandBuffer comBuf(500);
//Helper Used to connect to the WiFi network
//returns true if success, returns false otherwise
//Will also output diagnostic messages to the serial port.
bool connectToNet() {
  WiFi.config(ip);
  int result = WiFi.begin(net, pass); //Assumes a WPA wifi network
  if (result == WL_CONNECTED) {
    Serial.println("Network connected.");
    IPAddress ipLocal = WiFi.localIP();
    Serial.println(ipLocal);
    return true;

  } else {
    Serial.println("Error Network not found");
    return false;
  }
}

//Helper
//End point detector, old school. Assuming 3 letter end points
//Each endpoint corresponds to a different type of request.
#define CMD 1 //Command string
#define STA 2 //Request status
#define UNK 0 //Unknown request
int detectEndPoint(char * endpointString){
  if(strncmp("/cmd", endpointString, 4)==0){
        return CMD; 
  } else if(strncmp("/sta", endpointString, 4)==0){
        return STA;
  }
  return UNK;
}

//Event handler for the emergency stop pin.
volatile boolean eStop = false;
void emergencyStop(){
   eStop = true;
}
//setup
void setup() {
  //Set the button to input mode as well as attach interrupt handler
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), emergencyStop, FALLING);

  // put your setup code here, to run once:
  Serial.begin(9600);
  AFMS.begin();

  Serial.print("Robot Fred Booting!");

  //Attempts to connect to network
  //If this does work the rebot needs to be reset?
  Serial.println("Attempting to connect to network.");
  connectToNet();
  server.begin();
  Serial.println("Boot sequence complete, awaiting orders!");
}

//Helper function to set motor speeds and directions.
void setMotors(int targetSpeed, int targetDirectionR, int targetDirectionL) {
  motor1->run(targetDirectionR);
  motor1->setSpeed(targetSpeed);
  motor2->run(targetDirectionL);
  motor2->setSpeed(targetSpeed);
}
//Help function to indentify the beginning of a string
inline boolean startsWith(char * buff, char *refWord) {

  int result = strncmp(buff, refWord, 3);
  return result == 0;
}
//Removes trailing HTTP/1.1
inline void stripHTTP(char * buff) {
  char * firstSpace = strrchr(buff, ' ');
  if (firstSpace != NULL) {
    *firstSpace = 0;
  }
}
//Determins if a Character represents a command.
inline boolean isCommand(char com){
  switch(com){
    case 'F':
    case 'B':
    case 'R':
    case 'L':
    case 'S':
              return true;
              break;
    default : return false;
  }
}
//Parses a commnad
//Format FP:D;
//Command letter
//Power level, between 1 and 255
//Duration in milliseconds.
inline void parseCommands(char * buff) {
  char command;
  word power; //In percent
  word duration; //In milliseconds
  int colonIndex;
  char * start = buff+3;
  //look for one command
  Serial.println("In Parse Command");
  Serial.println(*start);
  while (isCommand(*start)) {
    command = *start;
    Serial.println(command);
    start = start + 1;
    if (command != 'S') {
      int offset = 0;
      while (*(start + offset) != 0 && *(start + offset) != ':') {
        //find first colon
        offset += 1;
      }
      *(start + offset) = 0;

      power = atoi(start);
      start = start + offset + 1;
      offset = 0;
      while (*(start + offset) != 0 && *(start + offset) != ';') {
        //find first colon
        offset += 1;
      }
      *(start + offset) = 0;
      duration = atoi(start);
      start = start + offset + 1;
    } else {
      start = start + 2;
      duration = 1000;
    }
    comBuf.addCommand(command, power, duration);
  }
}
//Requires 1Kb
//parse a incoming http request. We only hand GET for now.
inline void parseClient( WiFiClient & client) {
  char buff[1024];
  int numChar = 0;
  while (client.available()) {
    char c = client.read();
    if (c == '\n') { //Fetched a complete line
      buff[numChar] = 0;
      Serial.println(buff);
      numChar = 0;
      if (startsWith(buff, getCommand)) {
        //GET COMMAND
        stripHTTP(buff + 4);
        //Assume /cmd for now
        //need to add to check if it's /cmd, if so reset command list?
        //Newest program takes precedence.
        int endPoint = detectEndPoint(buff+4);
        if(endPoint == CMD){
           comBuf.reset(); //Erase any existing commands, only 1 program at a time.
           setMotors(0,RELEASE,RELEASE); //Stop the rover while receiving instructions.
           parseCommands(buff + 5);
           reportStatus(client);
        } else if(endPoint == STA){
            reportProgress(client);
        } else {
          reportNotFound(client);
        }
      }
    } else { //still reading in a line
      buff[numChar] = c;
      numChar += 1;
    }
  }
}
//reports the number of commands received from a command string
inline void reportStatus( WiFiClient & client) {
  //Response needed to be crafted 
  //To allow CORS support.
  client.println("HTTP/1.1 200 OK");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println("");
  client.print("Commands Received:");
  client.println(comBuf.length());
  return;
}
//Report what the robot is current doing.
inline void reportProgress( WiFiClient & client) {
  //Response needed to be crafted 
  //To allow CORS support.
  client.println("HTTP/1.1 200 OK");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println("");
  client.print("Commands Received:");
  client.println(comBuf.length());
  client.print("Current Command:");
  client.println(comBuf.currentCommand);
  client.print("Program Ended:");
  client.println(comBuf.endReached()==0 ? "false" : "true");
  client.print("Emergency Stop:");
  client.println(eStop==false ? "false" : "true");
  return;
}
//if the endpoint is unknown
inline void reportNotFound( WiFiClient & client) {
  //Response needed to be crafted 
  //To allow CORS support.
  client.println("HTTP/1.1 404 Not Found");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println("");
  client.println("endpoint not found");

  return;
}

unsigned long changeTime = 0;
void processProgram() {

  unsigned long currentTime = millis();
  if(eStop){
    eStop = false;
    comBuf.reset();
    setMotors(0,RELEASE,RELEASE);
    Serial.println("Emergency Stop hit, please send new program");
    return;
  }
  if (!comBuf.endReached()) {
    if (currentTime > changeTime) {
      comBuf.advanceCommand();
     
        command currCom = comBuf.getCurCommand();
        Serial.println("Printing Command!");
        currCom.print();
        changeTime = millis() + currCom.duration;
        switch (currCom.commandChar) {
          case 'F' : setMotors(currCom.power, FORWARD, FORWARD);
            break;
          case 'B' :
            setMotors(currCom.power, BACKWARD, BACKWARD);
            break;
          case 'R' :
            setMotors(currCom.power, FORWARD, BACKWARD);
            break;
          case 'L' :
            setMotors(currCom.power, BACKWARD, FORWARD);
            break;
          case 'S' :
            setMotors(0, RELEASE, RELEASE);
            break;
          default : 
            setMotors(0, RELEASE, RELEASE);
        }
    } 
  }else if (currentTime > changeTime){
    setMotors(0, RELEASE, RELEASE);
  } 
}
void loop() {
  // put your main code here, to run repeatedly:
  int wStatus = WiFi.status();  //get wifi status
  int incomingByte;
  
  //WiFi lost of signal warning message.
  if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi signal lost, attempting to reconnect.");
      WiFi.disconnect();
      connectToNet();
     
  } 
  //Command processing, with time slices
  processProgram();
  //Process WiFi messages if any!
  WiFiClient client = server.available(); //Get client, presumably there can be more than one
  if (client) {
    //If there is a client, service the request and close the connection.
    //This is going to be a REST API
    Serial.println("Responding to Client");
    parseClient(client);
    client.stop(); //We get here from the break statement in the HTTP response
    Serial.println("Client disconnected");
  }//end of if statement
}

