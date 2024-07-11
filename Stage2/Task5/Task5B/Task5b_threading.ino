#include <WiFi.h>

// WiFi credentials
const char *ssid = "IQ"; // Enter your wifi hotspot ssid
const char *password = "qdvhuiop";  // Enter your wifi hotspot password
const uint16_t port = 8004;
const char *host = "192.168.131.144";

TaskHandle_t LineFollowing;
TaskHandle_t ServerReader;

float pTerm, iTerm, dTerm;
int previousError;
float kp = 10; // 10
float ki = 0;
float kd = 11; // 11
float output;
int integral, derivative;
int k = 0;

// defining ir sensor pin
const int ir0 = 32;
const int ir1 = 13; // Corrected pin number
const int ir2 = 12;
const int ir3 = 34; // Corrected pin number
const int ir4 = 15;

// defining motor pin
const int LMpin1 = 25;
const int LMpin2 = 33;
const int RMpin1 = 27;
const int RMpin2 = 26;

// defining the buzzer and led pin
const int buzzerPin = 18;
const int greenledPin = 19;
const int redledPin = 23;

const int frequency = 500;
const int pwm_channel1 = 0;
const int pwm_channel2 = 1;
const int pwm_channel3 = 2;
const int pwm_channel4 = 3;
const int resolution = 8;

int node = 0;
int LFir[] = {0, 0, 0, 0, 0};

int node_value = 0;
int no_of_node = 0;
float error;
long startTime;
long endTime;
String number_of_path;
int n_path;
String path_array[6] = {"", "", "", "", "", ""};
char incomingPacket[80];
WiFiClient client;
String RUN_command = "";
String Event_Visit_command = "";
int speed=110;

void setup()
{
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(LMpin1, OUTPUT);
  pinMode(LMpin2, OUTPUT);
  pinMode(RMpin1, OUTPUT);
  pinMode(RMpin2, OUTPUT);

  pinMode(buzzerPin, OUTPUT);
  pinMode(greenledPin, OUTPUT);
  pinMode(redledPin, OUTPUT);
  pinMode(ir0, INPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);

  ledcSetup(pwm_channel1, frequency, resolution);
  ledcSetup(pwm_channel2, frequency, resolution);
  ledcSetup(pwm_channel3, frequency, resolution);
  ledcSetup(pwm_channel4, frequency, resolution);
  ledcAttachPin(LMpin1, pwm_channel1);
  ledcAttachPin(LMpin2, pwm_channel2);
  ledcAttachPin(RMpin1, pwm_channel3);
  ledcAttachPin(RMpin2, pwm_channel4);
  stopMotors();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("...");
  }
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(redledPin, HIGH);

  xTaskCreatePinnedToCore(
    LineFollowingCode, /* Task function. */
    "LineFollowing",   /* name of task. */
    10000,             /* Stack size of task */
    NULL,              /* parameter of the task */
    1,                 /* priority of the task */
    &LineFollowing,    /* Task handle to keep track of created task */
    1);                /* pin task to core 0 */
  delay(500);

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

void pidCalculations(int error)
{
  pTerm = kp * error;
  integral += error;
  iTerm = ki * integral;
  derivative = error - previousError;
  dTerm = kd * derivative;
  output = pTerm + iTerm + dTerm;
  previousError = error;
}

void moveForward(int error)
{
  int pid_error = error;
  pidCalculations(pid_error);
  int LM_Speed = speed + output; // 150 //100
  int RM_Speed = speed - output; // 180 //142
  ledcWrite(pwm_channel1, LM_Speed);
  ledcWrite(pwm_channel2, 0);
  ledcWrite(pwm_channel3, RM_Speed);
  ledcWrite(pwm_channel4, 0);
}

void Uturn()
{
  //  moveForward(2);
  //  delay(50);
  ledcWrite(pwm_channel1, 100); // 200  // Set the left motor speed
  ledcWrite(pwm_channel2, 0);   // Stop the left motor
  ledcWrite(pwm_channel3, 0);   // Stop the right motor
  ledcWrite(pwm_channel4, 100); // 200
  //
  //  readSensor();
  //  int i = 0;
  //  while (i <= 1) {
  //    readSensor();
  //    delay(100);
  //    if (LFir[2] == 1) {
  //      i++;
  //      delay(65);
  //    }
  //  }

  //  startTime = millis();
  //  endTime = startTime;
  //  while ((endTime - startTime) <= 1500) {
  //    readSensor();
  //    if (LFir[2] == 1) {
  //      delay(50);
  //      break;
  //    }
  //    endTime = millis();
  //  }
}

void stopMotors()
{
  ledcWrite(pwm_channel1, 0);
  ledcWrite(pwm_channel2, 0);
  ledcWrite(pwm_channel3, 0);
  ledcWrite(pwm_channel4, 0);
}

void readSensor()
{
  LFir[0] = digitalRead(ir0);
  LFir[1] = digitalRead(ir1);
  LFir[2] = digitalRead(ir2);
  LFir[3] = digitalRead(ir3);
  LFir[4] = digitalRead(ir4);
}
void nodeUpdate()
{
  node = node + 1;
}
void turnRight()
{

  ledcWrite(pwm_channel1, 85); // 200  // Set the left motor speed
  ledcWrite(pwm_channel2, 0);   // Stop the left motor
  ledcWrite(pwm_channel3, 0);   // Stop the right motor
  ledcWrite(pwm_channel4, 85); // 200
}
void turnLeft()
{
  ledcWrite(pwm_channel1, 0);   // Set the left motor speed
  ledcWrite(pwm_channel2, 88); // 200   // Stop the left motor
  ledcWrite(pwm_channel3, 88); // 200  // Stop the right motor
  ledcWrite(pwm_channel4, 0);
}

void LineFollowingCode(void *pvParameters)
{
  Serial.print("Task1 running on core ");
  // Serial.println(xPortGetCoreID());
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    if (RUN_command != "RUN")
    {
      delay(200);
      continue;
    }
    Serial.println("StartingLineFollowing");
    for (int path_no = 0; path_no < n_path; path_no++)
    {
      String path = path_array[path_no];
      node = 0;
      Serial.println(path);
      node_value = path[node] - '0';
      if (node_value == 4)
      {

        Uturn();
        startTime = millis();
        endTime = startTime;
        while ((endTime - startTime) <= 1550) {
          endTime = millis();
        }
          stopMotors();
      }
        no_of_node = path.length() - 1;
        if (path_no == 0) {

          moveForward(-1);
          delay(500);
          

        }
        if(path_no==1){
          speed=80;
        }
        
        for (;;)
        {
          readSensor();

          Serial.print(node);
          Serial.println(no_of_node);
          if(node==2 && path_no==1){
            speed=110;}
          if (node >= no_of_node)
          {
            //Case for E as first event            
            if( path_no==0){
//              moveForward(1.4);
//              delay(1800);
//              moveForward(1.5);
//              delay(2000);
//              stopMotors();
//              digitalWrite(buzzerPin, LOW);
//              digitalWrite(redledPin, HIGH);
//              delay(3000);
//              digitalWrite(redledPin, LOW);
//              digitalWrite(buzzerPin, HIGH);
//              Event_Visit_command = "NO";
              speed=70;
//              break;
              }
            if (path_no + 1 == n_path)
            {
              Serial.println('stop');
              moveForward(1.5);
              delay(1600);
              stopMotors();
              digitalWrite(buzzerPin, LOW);
              digitalWrite(redledPin, HIGH);
              delay(5000);
              digitalWrite(redledPin, LOW);
              digitalWrite(buzzerPin, HIGH);
              RUN_command = "STOP";
              break;
            }
            if (Event_Visit_command == "Yes")
            {
              Serial.println();
              Serial.println(Event_Visit_command);
              stopMotors();
              digitalWrite(buzzerPin, LOW);
              digitalWrite(redledPin, HIGH);
              delay(1500);
              digitalWrite(redledPin, LOW);
              digitalWrite(buzzerPin, HIGH);
              Event_Visit_command = "NO";
              break;
            }
          }
          // node detection
          readSensor();
          if (LFir[2] == 1 && LFir[1] == 1 && LFir[3] == 1)
          {
            stopMotors();
            // digitalWrite(buzzerPin, LOW);
            // digitalWrite(greenledPin, HIGH);
            delay(1000);
            // digitalWrite(greenledPin, LOW);
            // digitalWrite(buzzerPin, HIGH);
            nodeUpdate();
            Serial.print("Node: ");
            Serial.println(node);
            // String path = path_array[0];
            node_value = path[node] - '0';
            Serial.println(node_value);
            if (node_value == 2)
            {
              if (node == 1 && path_no == 0)
              {
                moveForward(-1.5);  //-0.2----------
                delay(300);
              }
              else
              {
                moveForward(0);
                delay(300);
              }
            }
            else if (node_value == 1)
            {
              moveForward(-1.3);
              delay(550);
              turnLeft();
              delay(400);
              turnLeft();
              int i = 1;
              while (i == 1)
              {
                readSensor();
                if (LFir[2] == 1)
                {
                  i++;
                  delay(65);
                }
              }
              stopMotors();
            }
            else if (node_value == 3)
            {
              moveForward(1);
              delay(550);
              turnRight(); //3
              delay(400);
              turnRight();
              // Detecting whether it has achieved center line again or not while turning.
              int i = 1;
              while (i == 1)
              {
                readSensor();
                if (LFir[2] == 1)
                {
                  i++;
                  delay(65);
                }
              }
              stopMotors();
            }

            error = 0;
            //    Serial.println("hello : ");
          }

          // determine the error
          // center line detection
          else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 1) && (LFir[3] == 0) && (LFir[4] == 0))
          {
            error = 0;
          }
          else if ((LFir[0] == 0) && (LFir[1] == 1) && (LFir[2] == 1) && (LFir[3] == 0) && (LFir[4] == 0))
          {
            error = -0.5;
          }
          else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 1) && (LFir[3] == 1) && (LFir[4] == 0))
          {
            error = 2;
          }
          else if ((LFir[0] == 0) && (LFir[1] == 1) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 0))
          {
            error = -1.5;
          }
          else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 1) && (LFir[4] == 0))
          {
            error = 2;
          }
          else if (LFir[0] == 0 && LFir[1] == 0 && LFir[2] == 0 && (LFir[3] == 0) && (LFir[4] == 0))
          {
            error = 0;
          }
          else if ((LFir[0] == 1) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 0))
          {
            error = 2;
          }
          else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 1))
          {
            error = -2;
          }
          moveForward(error);
          //      Serial.println("RUNNING FORWARD");
        }
      }
    }
  }
  void ServerReaderCode(void *pvParameters)
  {
    Serial.print("Task1 running on core ");
    Serial.println(xPortGetCoreID());
    // Connecting the server.
    for (;;)
    {
      if (!client.connect(host, port))
      {
        Serial.println("Connection to host failed");
        delay(200);
        continue;
      }
      break;
    }
    delay(1000);
    number_of_path = client.readStringUntil('\n');
    Serial.println(number_of_path);
    n_path = number_of_path.toInt();
    delay(3000);
    for (int i = 0; i < n_path; i++)
    {
      path_array[i] = client.readStringUntil('\n');
      Serial.println(path_array[i]);
      delay(5000);
    }
    delay(1000);
    for (;;)
    {
      RUN_command = client.readStringUntil('\n');
      Serial.println(RUN_command);
      if (RUN_command == "RUN")
      {
        Serial.println("RUN");
        break;
      }
    }
    for (;;)
    {
      if ( client.readStringUntil('\n') == "Yes") {
        Event_Visit_command = "Yes";
      }
      Serial.println(Event_Visit_command);

    }
  }

  // Main Loop Function
  void loop()
  {
  }
