/*
 * Team Id:       <GG_3344>
 * Author List:   <Shiwam Kumar,Mohammad kaif, Abhay Agrawal>
 * Filename:      <Line_Following_Connection>
 * Theme:         <Geo Guide ->eyrc2023-24>
 * Functions:     <Setup,pidCalculations,moveForward,Uturn,stopMotors,readSensor,nodeUpdate,turnRight,turnLeft,LineFollowingcode,ServerReaderCode,loop>
 * Global Variables: <*ssid,*password,port,*host,pTerm, iTerm, dTerm,kp,ki,kd,output,integral,derivative,k,ir0,ir1,ir2,ir3,ir4,LMpin1,LMpin2,RMpin1,RMpin2,
 * buzzerPin,frequency, pwm_channel1, pwm_channel2, pwm_channel3, pwm_channel4,resolution,node_visited,LFir,node_value,no_of_node,error,startTime,endTime,number_of_path
 * n_path,path_array,incomingPacket,path_no_of_E,RUN_command,Event_Visit_command,speed>
 */

// Importing Modules
#include <WiFi.h>

// Multithreading
TaskHandle_t LineFollowing;
TaskHandle_t ServerReader;

// Global Variables
///////////////////////////////////////////////////////////////////////////
// WiFi client details
const char *ssid = "IQ";            // Enter your wifi hotspot ssid
const char *password = "qdvhuiop";  // Enter your wifi hotspot password
const uint16_t port = 8002;
const char *host = "192.168.50.21";
// Defining ESP32 connection pins
const int ir0 = 32;  // leftMost
const int ir1 = 13;  // center_left
const int ir2 = 12;  // center
const int ir3 = 34;  // center_right
const int ir4 = 15;  // rightMost
// Defining Motor pins
const int LMpin1 = 25;  // LeftMotorPin
const int LMpin2 = 33;
const int RMpin1 = 27;  // RightMotorPin
const int RMpin2 = 26;
// Defining buzzer pin
const int buzzerPin = 18;
// Defining frequency
const int frequency = 500;
// Defining pwm chanell
const int pwm_channel1 = 0;
const int pwm_channel2 = 1;
const int pwm_channel3 = 2;
const int pwm_channel4 = 3;
const int resolution = 8;
// Defining node variable which is storing number of node visited.
int node_visited = 0;
// Defining LFir array which stores ir Sensor readings
int LFir[] = { 0, 0, 0, 0, 0 };

// Defining Global variables
float pTerm, iTerm, dTerm;
float previousError;
float kp = 10;
float ki = 0;
float kd = 11;
float output;
float integral, derivative;
int node_value = 0;
int no_of_node = 0;
float error;
long startTime;
long endTime;
String number_of_path;
int n_path;
String path_array[6] = { "", "", "", "", "", "" };
char incomingPacket[80];
int path_no_of_E;

WiFiClient client;
String RUN_command = "";
String Event_Visit_command = "";
int speed = 110;

/*
 * Function Name:<setup>
 * Input:     <None>
 * Output:    <None>
 * Logic:     <This function setup pinmode of motors and ir sensors, setup frequency and resolution of pwm channel and
               attach motor pins to pwm chanel for pwm output of motors and define the Multithreading >
 * Example Call:   <None>
  */
void setup() {
  Serial.begin(9600);
  // setup pinMode of motor pins
  pinMode(LMpin1, OUTPUT);
  pinMode(LMpin2, OUTPUT);
  pinMode(RMpin1, OUTPUT);
  pinMode(RMpin2, OUTPUT);
  // setup pinMode of buzzerPin
  pinMode(buzzerPin, OUTPUT);
  // setup pinMode of ir pins
  pinMode(ir0, INPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  // setup frequncy of pwm channels
  ledcSetup(pwm_channel1, frequency, resolution);
  ledcSetup(pwm_channel2, frequency, resolution);
  ledcSetup(pwm_channel3, frequency, resolution);
  ledcSetup(pwm_channel4, frequency, resolution);
  // Attaching motorpins to pwm channels
  ledcAttachPin(LMpin1, pwm_channel1);
  ledcAttachPin(LMpin2, pwm_channel2);
  ledcAttachPin(RMpin1, pwm_channel3);
  ledcAttachPin(RMpin2, pwm_channel4);
  // Initially call the stopMotor
  stopMotors();
  WiFi.begin(ssid, password);
  // Waiting until Wifi is connected successfully
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  digitalWrite(buzzerPin, HIGH);
  // The First Priority is given to line following and then to Connection.
  // Assigning Line Following Algorithm to Thread 1.
  xTaskCreatePinnedToCore(
    LineFollowingCode, /* Task function. */
    "LineFollowing",   /* name of task. */
    10000,             /* Stack size of task */
    NULL,              /* parameter of the task */
    1,                 /* priority of the task */
    &LineFollowing,    /* Task handle to keep track of created task */
    1);                /* pin task to core 0 */
  delay(500);
  // Assigning Connection to server code to thread zero.
  xTaskCreatePinnedToCore(
    ServerReaderCode, /* Task function. */
    "ServerReader",   /* name of task. */
    10000,            /* Stack size of task */
    NULL,             /* parameter of the task */
    0,                /* priority of the task */
    &ServerReader,    /* Task handle to keep track of created task */
    0);
  delay(500);
}

/*
 * Function Name:<pidCalculations>
 * Input:     <error -->A floating number parameter which is passed in pidCalculations function which store the error according to Sensor reading.>
 * Output:    <None>
 * Logic:     <Calculating PTerm,iTerm,dTerm according to error and then Calculating output by adding these pTerm,iTerm and dTerm>
 * Example Call:   <pidCalculations(4)-->Calling the pidCalculations function with error value 4>
 */
void pidCalculations(float error) {
  /*pTerm is propotional term in pid and this term is based on the current error*/
  pTerm = kp * error;
  integral += error;
  /* iTerm is integral term in pid and this term is based on the accumulation of past error over time */
  iTerm = ki * integral;
  derivative = error - previousError;
  /* dTerm is derivative term in pid and this term is based on the rate of change of error */
  dTerm = kd * derivative;
  output = pTerm + iTerm + dTerm;
  previousError = error;
}
/*
 * Function Name:<readSensor>
 * Input:     <None>
 * Output:    <None>
 * Logic:     <Digital reading of all the ir sensors and storing them in an array
 *              Updation of LFir Array>
 * Example Call:   readSensor()
 */
void readSensor() {
  LFir[0] = digitalRead(ir0);  // leftMost ir
  LFir[1] = digitalRead(ir1);  // center_left ir
  LFir[2] = digitalRead(ir2);  // center ir
  LFir[3] = digitalRead(ir3);  // center_right ir
  LFir[4] = digitalRead(ir4);  // rightMost ir
}
/*
 * Function Name:<nodeUpdate>
 * Input:     <None>
 * Output:    <None>
 * Logic:     <If any node is detected this function is called. It updates the number of nodes visited by adding 1.>
 * Example Call:   <nodeUpdate()>
 */
void nodeUpdate() {
  node_visited = node_visited + 1;
}
/*
 * Function Name:<moveForward>
 * Input:     <error -->A floating number parameter which is passed in moveForward function which store the error according to Sensor reading.>
 * Output:    <None>
 * Logic:     < 1. We calculate the difference in speed of the bot using the pidCalculations.
 *              2. We then write the speed to respective pins of the motor.>
 * Example Call:   moveForward(-1.5) where -1.5 is error
 */
void moveForward(float error) {
  float pid_error = error;
  pidCalculations(pid_error);     // To calculate difference in speed to be made in motors to bring bot to center.
  int LM_Speed = speed + output;  // output is calculated by PID calculation and updated globally
  int RM_Speed = speed - output;
  ledcWrite(pwm_channel1, LM_Speed);
  ledcWrite(pwm_channel2, 0);
  ledcWrite(pwm_channel3, RM_Speed);
  ledcWrite(pwm_channel4, 0);
}

/*
 * Function Name:<stopMotors>
 * Input:     <None>
 * Output:    <None>
 * Logic:     <Define Speed of all the motors pins to zero>
 * Example Call:   stopMotors()
 */
void stopMotors() {
  ledcWrite(pwm_channel1, 0);
  ledcWrite(pwm_channel2, 0);
  ledcWrite(pwm_channel3, 0);
  ledcWrite(pwm_channel4, 0);
}

/*
 * Function Name:<turnRight>
 * Input:     <None>
 * Output:    <None>
 * Logic:     <1.First we need to make the bot move forward so that its center of mass is aligned with the center of node so that turn could be easy.
 *             2.We then  make left motor move in forward direction and right motor in reverse direction to make bot start turning right.
 *             3.Then continously try to detect the black line from the center ir. As soon as the Central ir is high (detects black line) Stop Motors is Called and bot has turned successfully.
 * Example Call:   <turnRight()>
 */
void turnRight() {
  moveForward(1.6);
  delay(600);
  ledcWrite(pwm_channel1, 85);
  ledcWrite(pwm_channel2, 0);
  ledcWrite(pwm_channel3, 0);
  ledcWrite(pwm_channel4, 85);
  delay(400);
  int i = 1;
  while (i == 1)  // Running till black line is detected by central ir.
  {
    readSensor();
    if (LFir[2] == 1) {
      i++;
      delay(65);
    }
  }
  stopMotors();
}
/*
 * Function Name:<turnLeft>
 * Input:     <None>
 * Output:    <None>
 * Logic:     <1.First we need to make the bot move forward so that its center of mass is aligned with the center of node so that turn could be easy.
 *             2.We then  make left motor move in backward direction and right motor in forward direction to make bot start turning left.
 *             3.Then continously try to detect the black line from the center ir. As soon as the Central ir is high (detects black line) Stop Motors is Called and bot has turned successfully.
 * Example Call:   <turnLeft()>
 */
void turnLeft() {
  moveForward(-0.5);
  delay(500);
  ledcWrite(pwm_channel1, 0);
  ledcWrite(pwm_channel2, 88);
  ledcWrite(pwm_channel3, 88);
  ledcWrite(pwm_channel4, 0);
  delay(350);
  int i = 1;
  while (i == 1)  // Running till black line is detected by central ir.
  {
    readSensor();
    if (LFir[2] == 1) {
      i++;
      delay(65);
    }
  }
  stopMotors();
}
/*
 * Function Name:<Uturn>
 * Input:     <None>
 * Output:    <None >
 * Logic:     <1.We then  make left motor move in forward direction and right motor in backward direction to make bot start turning right.
 *             2.Then we use delay so that bot could rotate at specific location. At most place the Uturn is between side roads.
 *             3.Then continously try to detect the black line from the rightmost ir. As soon as the rightmost ir is high (detects black line) Stop Motors is Called and bot has turned successfully.
 *             4.Rightmost ir has detected the black line of its very own side.
 * Example Call:   <Uturn()>
 */
void Uturn() {
  ledcWrite(pwm_channel1, 120);
  ledcWrite(pwm_channel2, 0);
  ledcWrite(pwm_channel3, 0);
  ledcWrite(pwm_channel4, 120);
  delay(1100);
  int i = 0;
  while (i < 1) {
    readSensor();
    if (LFir[0] == 1 || LFir[4] == 1) {
      i++;
    }
  }
  stopMotors();
}
/*
 * Function Name:<LineFollowingCode>
 * Input:     <Inputs (or Parameters) list with description if any>
 * Output:    <Return value with description if any>
 * Logic:     <1.Firstly Check whether the "RUN" command is initiated and sent by server.
 *             2.Once the Run command is initiated bot traverses the paths sent by python one by one.
 *             3.For the path_no fetch the corresponding path string.
 *             4.For the new path define number of node visited as zero.
 *             5.Find the Node_value correspoing to node_visited from pathstring and convert it to integer type.
 *             6.If at node zero uturn is needed take otherwise move forward .Node =0 is at events or at start position.
 *             7.Define different speed for special cases of turn or E_path.
 *             8.Start a Line Following algorithm to traverse the corresponding path until event is reached.
 *                Line Following Algorithm- CenterLine - 3 ir for center.
 *                                          SIDELINE - Detected as walls and bot tries to remain inside it - 2 side irs.
 *             9.Once the event is reached stop for 1 sec and reinitiate the new path to all variables.
 * >
 * Example Call:   LineFollowingCode()
 */
void LineFollowingCode(void *pvParameters) {
  for (;;) {
    if (RUN_command != "RUN") {
      delay(200);
      continue;
    }
    for (int path_no = 0; path_no < n_path; path_no++) {
      String path = path_array[path_no];
      node_visited = 0;
      node_value = path[node_visited] - '0';
      no_of_node = path.length() - 1;  // Number of nodes to be visited by bot before event is reached is calculated.
      // For the path returned by server 4 means Uturn so checking whether first node value is required u turn or not
      if (node_value == 4) {

        Uturn();
      }
      if (path_no == 0)  // This is to ensure bot moves smoothly for the initial turn and curve of the path. This is required when path_no == 0 i.e. first path
      {
        speed = 80;
      }
      // When the bot has visited the Event E it has to relatively move at slower spped till first node of the next path as steep curve.
      if (path_no == path_no_of_E + 1) {
        speed = 80;
      }
      // INITIATING THE LineFollowing Algorithm For the current path.
      for (;;) {
        readSensor();

        if (node_visited == 1 && (path_no == path_no_of_E + 1 || path_no == 0))  // If the node 1 is reached for the first path and the path after E make the bot speed to normal as curve has been completed.
        {
          speed = 110;
        }
        if (node_visited >= no_of_node)  // To check Whether the last node has been crossed of the path to check for stop command.
        {
          if (path_no == path_no_of_E)  // After the last node towards the E make the bot slower to let bot smoothly traverse curve path.
          {
            speed = 90;
          }

          if (Event_Visit_command == "Yes")  // If the event is reached the connection thread updates the Event_Visit_command to STOP and hence bot stop and beeps
          {
            stopMotors();
            digitalWrite(buzzerPin, LOW);
            delay(1500);
            if (path_no + 1 == n_path)  // In addition check whether the Path traversed is last path or not
            {                           // If Yes Buzzer has to additionally beep for 3.5s i.e total 5 secs.Initiate the RUN_command to Stop so that bot stops permanently indicating "END OF RUN".
              delay(3500);
              RUN_command = "STOP";
            }
            digitalWrite(buzzerPin, HIGH);
            Event_Visit_command = "NO";  // Initialize the command to NO so that bot just simply dont stop next to last node for next path and traverse till the till event location.

            break;  // break LineFollowing and reupdate the next path.
          }
        }
        readSensor();
        // Negative Error means that Right motor is turning fast.
        // Positive Error means that Left motor is turning fast.
        if (LFir[2] == 1 && LFir[1] == 1 && LFir[3] == 1)  // Condition to Check whether node is detected or not.
        // These three central ir becomes high on node and indicate detection of node.
        {
          // stopMotors for some time .
          stopMotors();
          delay(500);
          nodeUpdate();
          node_value = path[node_visited] - '0';  // Calculate the Node Value at this particular node to turn right or left or move forward.
          // If node value is 2 bot is moving forward
          if (node_value == 2) {
            // For the first path bot comes from initial location to follow the center line in straight manner bot has to move slightly in right direction.
            if (node_visited == 1 && path_no == 0) {
              moveForward(-1.5);
              delay(400);
            }

            else if (node_visited == 1 && path_no == path_no_of_E + 1) {  // First node after E path make speed = normal and let the bot cross the node area.
              speed = 110;
              moveForward(0);
              delay(300);
            }

            else {  // To cross the bot from the node area in all other forward cases.
              moveForward(0.4);
              delay(300);
            }
          } else if (node_value == 1)  // if node_value of the present node is 1 turn left by calling turnLeft()
          {
            turnLeft();
          } else if (node_value == 3)  // if node_value of the present node is 3 turn right by calling turnRight()
          {
            turnRight();
          }

          error = 0;  // Once the node area is cleared continue LineFollowing code with the zero error.
        }
        //***************************************************************************************************************************
        // CENTER LINE ALGORITHM
        /*Leftmost IR- 0
          centerLeft IR - 1
          center IR- 2
          centerRightIR - 3
          rightMost - 4
        */
        //***************************************************************************************************************************
        // If only ir 2 detects black line then move the bot forward with same speed of both the motors on centerline
        else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 1) && (LFir[3] == 0) && (LFir[4] == 0)) {
          error = 0;
        }
        // If  ir 1,2  is high then move the bot forward with negative error i.e bot has deviated towards the right so let it turn slightly to left
        else if ((LFir[0] == 0) && (LFir[1] == 1) && (LFir[2] == 1) && (LFir[3] == 0) && (LFir[4] == 0)) {
          error = -0.5;
        }
        // If  ir 2,3  is high then move the bot forward with positive error i.e bot has deviated towards the left so let it turn slightly to right
        else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 1) && (LFir[3] == 1) && (LFir[4] == 0)) {
          error = 2;
        }
        //If ir 1 is high ,high negative error is passed to take the bot move more towards center more
        else if ((LFir[0] == 0) && (LFir[1] == 1) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 0)) {
          error = -1.5;
        }
        //If ir 3 is high ,high positive error is passed to take the bot move more towards center more by right
        else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 1) && (LFir[4] == 0)) {
          error = 2;
        }
        //If none of the sensor is high bot is perfectly between the two side black lines and has to move perfect straight so error is zero
        else if (LFir[0] == 0 && LFir[1] == 0 && LFir[2] == 0 && (LFir[3] == 0) && (LFir[4] == 0)) {
          error = 0;
        }
        //*************************************************************************************************************
        //SIDE WALL Following
        //*************************************************************************************************************
        //If IR 0 is high which means bot has deviated towards left to side wall and has to move right so positive error.
        else if ((LFir[0] == 1) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 0)) {
          error = 3;
        }
        //If IR 4 is high which means bot has deviated towards right to side wall and has to move left so negative error.
        else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 1)) {
          error = -2;
        }
        /* Move the bot forward with error got from above conditions*/
        moveForward(error);
      }
    }
  }
}
/*
 * Function Name:ServerReaderCode
 * Input:     *pvParameters
 * Output:    <None>
 * Logic:     <1.Try to Connect To The server until connected. 
 *             2.Read total number of path.
 *             3.For n_path read n path string.
 *             4.Read priority path number of E.
 *             5.Read the command until "RUN" is read.
 *             6.Read the Stop command until the connection from the server is closed.>
 * Example Call:   <Example of how to call this function>
 */
void ServerReaderCode(void *pvParameters) {
  
    for (;;) {
      if (!client.connect(host, port))  //Connect to Server
      {
        delay(200);
        continue;
      }
      break;
    }
    delay(1000);
    number_of_path = client.readStringUntil(',');  //Read number of path string
    n_path = number_of_path.toInt();
    for (int i = 0; i < n_path; i++)  //Read n paths string
    {
      path_array[i] = client.readStringUntil(',');
    }
    delay(200);
    String E = client.readStringUntil(',');
    path_no_of_E = E.toInt();  //integer of E path number
    delay(1000);
    while (1) {  //Read the command until RUN is read.
      RUN_command = client.readStringUntil('\n');
      if (RUN_command == "RUN") {
        Serial.println("Running");
        break;
      }
    }
    while (1) {  //Continously read for STOP command from server if server is available and connected.
      if (client.readStringUntil('\n') == "Yes") {
        Event_Visit_command = "Yes";
      }
    }
  }

//This keeps the esp code running in  loop.
void loop() {
}
