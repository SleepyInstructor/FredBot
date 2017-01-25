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

//WiFi related stuff
char  net[] = "WiFiProject2";
char pass[] = "FreeCoffee";  //sketchy, but what else are you going to do.
char botName[] = "Fred001"; // rebot name, different for each bot.
// source code nightmare ahead.
char getCommand[] = "GET";
IPAddress ip;
WiFiServer server(80);  //Use port 80, the parameter is the port number

//Pin related stuff
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
//Fixed buffer to hold the commands
//only one set of commands at a time for now.
commandBuffer comBuf(500);
//Used to connect to the WiFi network
//returns true if success, returns false otherwise
//Will also output diagnostic messages to the serial port.
bool connectToNet() {
  int result = WiFi.begin(net, pass); //Assumes a WPA wifi network
  if (result == WL_CONNECTED) {
    Serial.println("Network connected.");
    ip = WiFi.localIP();
    Serial.println(ip);
    return true;

  } else {
    Serial.println("Error Network not found");
    return false;
  }
}

//setup
volatile boolean eStop = false;
void emergencyStop(){
   eStop = true;
}
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
//Parse the individual commands to a command array
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
void parseClient( WiFiClient & client) {
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
        
        parseCommands(buff + 5);

      }
    } else { //still reading in a line
      buff[numChar] = c;
      numChar += 1;
    }
  }
}
void reportStatus( WiFiClient & client) {

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.println("Everything is okay!");
  client.println("</html>");
  return;
}
unsigned long changeTime = 0;
void processProgram() {

  unsigned long currentTime = millis();
  if(eStop && digitalRead(buttonPin)==LOW){
    eStop = false;
    comBuf.reset();
    Serial.println("Emergency Stop hit, please send new program");
    return;
  }
  if (!comBuf.endReached()) {
    Serial.println("Program is live");
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

int msgCount = 0;
void loop() {
  // put your main code here, to run repeatedly:
  int wStatus = WiFi.status();  //get wifi status
  int incomingByte;
  
  //WiFi lost of signal warning message.
  if (WiFi.status() != WL_CONNECTED && msgCount == 0) {
      Serial.println("WiFi signal lost, consider resetting.");
      msgCount += 1;  
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
    reportStatus(client);
    client.stop(); //We get here from the break statement in the HTTP response
    Serial.println("Client disconnected");
  }//end of if statement
}
