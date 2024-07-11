#include <WiFi.h>

// WiFi credentials
const char *ssid = "Abhay Agrawal"; // Enter your wifi hotspot ssid
const char *password = "kdrv12x7";  // Enter your wifi hotspot password
const uint16_t port = 8002;
const char *host = "192.168.143.21";
// defining pid terms
float pTerm, iTerm, dTerm;
int previousError;
float kp = 15; // 10
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
const int LMpin1 = 27;
const int LMpin2 = 26;
const int RMpin1 = 25;
const int RMpin2 = 33;

// defining the buzzer and led pin
const int buzzerPin = 21;
const int greenledPin = 19;
const int redledPin = 18;

const int frequency = 500;
const int pwm_channel1 = 0;
const int pwm_channel2 = 1;
const int pwm_channel3 = 2;
const int pwm_channel4 = 3;
const int resolution = 8;
int node = 0;
int LFir[] = {0, 0, 0, 0, 0};
float error;

char incomingPacket[80];
WiFiClient client;
String command = "";

void setup()
{
  // put your setup code here, to run once:
  pinMode(LMpin1, OUTPUT);
  pinMode(LMpin2, OUTPUT);
  pinMode(RMpin1, OUTPUT);
  pinMode(RMpin2, OUTPUT);

  pinMode(buzzerPin, OUTPUT);
  pinMode(greenledPin, OUTPUT);
  pinMode(redledPin, OUTPUT);

  Serial.begin(9600);

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

  //  digitalWrite(redledPin, HIGH);
  //  digitalWrite(buzzerPin, LOW);
  //  delay(1000);
  //  digitalWrite(redledPin, LOW);
  digitalWrite(buzzerPin,HIGH);
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
  digitalWrite(redledPin,HIGH);
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

// function for moving the motor forward
void moveForward(int error)
{
  int pid_error = error;
  pidCalculations(pid_error);
  int LM_Speed = 150 + output; // 150
  int RM_Speed = 180 - output; // 180
  // if (error == 0) {
  //   LM_Speed = 200;
  //   RM_Speed = 200;
  // } else if (error > 0) {  // line moves to the right
  //   LM_Speed=200+ (kp * error);
  //   RM_Speed=200- (kp * error);
  // } else if (error < 0) {  // line moves to the left
  //   LM_Speed=200+(kp * error);
  //   RM_Speed=200-(kp * error);
  // }
  ledcWrite(pwm_channel1, LM_Speed);
  ledcWrite(pwm_channel2, 0);
  ledcWrite(pwm_channel3, RM_Speed);
  ledcWrite(pwm_channel4, 0);
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
// void activateBuzzer() {
//   // Turn on the buzzer
//   digitalWrite(buzzerPin, LOW);
// }

// void deactivateBuzzer() {
//   // Turn off the buzzer
//   digitalWrite(buzzerPin, HIGH);
// }
void nodeUpdate()
{
  node = node + 1;
}
void turnRight()
{

  ledcWrite(pwm_channel1, 60); // 200  // Set the left motor speed
  ledcWrite(pwm_channel2, 0);  // Stop the left motor
  ledcWrite(pwm_channel3, 0);  // Stop the right motor
  ledcWrite(pwm_channel4, 0);  // 200
}
void turnLeft()
{
  ledcWrite(pwm_channel1, 0);  // Set the left motor speed
  ledcWrite(pwm_channel2, 60); // 200   // Stop the left motor
  ledcWrite(pwm_channel3, 60); // 200  // Stop the right motor
  ledcWrite(pwm_channel4, 0);
}

//}

void loop()
{
  // put your main code here, to run repeatedly:
  if (!client.connect(host, port))
  {
    Serial.println("Connection to host failed");
    delay(200);
    return;
  }
  while (1)
  {
    command = client.readStringUntil('\n');
    if (command == "RUN")
    {
      digitalWrite(redledPin, LOW);
      digitalWrite(buzzerPin, LOW);
      delay(1000);
      digitalWrite(buzzerPin, HIGH);
      while (1)
      {
        readSensor();
        // side line detection
        if (LFir[0] == 1 && LFir[4] == 1)
        {
          error = 0;
        }
        else if (LFir[0] == 1 && LFir[4] == 1 && LFir[2] == 1)
        {
          error = 0;
        }
        else if (LFir[2] == 1 && LFir[1] == 1 && LFir[3] == 1)
        {
          stopMotors();
          digitalWrite(buzzerPin, LOW);
          digitalWrite(greenledPin, HIGH);
          delay(2000);
          digitalWrite(greenledPin, LOW);
          digitalWrite(buzzerPin, HIGH);
          nodeUpdate();
          Serial.print("Node: ");
          Serial.println(node);
          if (node == 1)
          {

            turnLeft();
            delay(500);
            moveForward(0);
            Serial.println("delaying ");
            delay(1000);
          }
          else if (node == 2 || node == 7 || node == 9)
          {

            // delay(1000);
            moveForward(0.999);
            //       Serial.println("delaying ");
            delay(600);
          }
          //    if (node == 11) {
          //
          //      //delay(1000);
          //      moveForward(0.999);
          //      //       Serial.println("delaying ");
          //      delay(1000);
          //      stopMotors();
          //    }
          else if (node == 11)
          {
            if (k == 0)
            {
              moveForward(0.95);
              delay(400);
            }
          }
          else if (node == 3 || node == 5 || node == 6 || node == 8)
          {
            moveForward(1);
            delay(600); // 900
            turnRight();
            delay(300);
            //      stopMotors();
            if (LFir[2] == 1 || LFir[2] == 0)
            {
              int i = 1;
              turnRight();
              while (i == 1)
              {
                //          turnLeft();
                readSensor();
                if (LFir[2] == 1)
                {
                  i++;
                  delay(65);
                  Serial.println(i);
                }
                //          delay(100);
                Serial.println("turn left");
              }
            }
            stopMotors();
          }
          else if (node == 4 || node == 10)
          {
            moveForward(0.95);
            Serial.println("move");

            delay(900);
            turnLeft();
            delay(300);
            //      delay(650);
            //      stopMotors();
            if (LFir[2] == 1 || LFir[2] == 0)
            {
              int i = 1;
              turnLeft();
              while (i == 1)
              {
                //          turnLeft();
                readSensor();
                if (LFir[2] == 1)
                {
                  i++;
                  delay(65);
                  Serial.println(i);
                }
                //          delay(100);
                Serial.println("turn left");
              }
            }
            stopMotors();
          }
          error = 1;
          //    Serial.println("hello : ");
        }
        else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 1))
        {
          error = 3;
        }
        else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 1) && (LFir[4] == 1))
        {
          error = 3;
        }
        else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 1) && (LFir[4] == 0))
        {
          error = 3;
        }
        else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 1) && (LFir[3] == 1) && (LFir[4] == 0))
        {
          error = 1;
        }
        else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 1) && (LFir[3] == 0) && (LFir[4] == 0))
        {
          error = 1;
        }
        else if ((LFir[0] == 0) && (LFir[1] == 1) && (LFir[2] == 1) && (LFir[3] == 0) && (LFir[4] == 0))
        {
          error = -1;
        }
        else if ((LFir[0] == 0) && (LFir[1] == 1) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 0))
        {
          error = -2;
        }
        else if ((LFir[0] == 1) && (LFir[1] == 1) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 0))
        {
          error = -3;
        }
        else if ((LFir[0] == 1) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 0))
        {
          error = -0.5;
        }
        else if (LFir[0] == 0 && LFir[1] == 0 && LFir[2] == 0 && (LFir[3] == 0) && (LFir[4] == 0))
        {
          if (node == 11)
          {
            if (k == 0)
            {

              moveForward(1);
              delay(600); // 900
              turnRight();
              delay(450);
              //      stopMotors();
              if (LFir[2] == 1 || LFir[2] == 0)
              {
                int i = 1;
                turnRight();
                while (i == 1)
                {
                  //          turnLeft();
                  readSensor();
                  if (LFir[2] == 1)
                  {
                    i++;
                    delay(65);
                    Serial.println(i);
                  }
                  //          delay(100);
                  Serial.println("turn left");
                }
                k++;
              }
            }
            else
            {
              moveForward(1);
              delay(300);
              stopMotors();
              digitalWrite(buzzerPin, LOW);
              digitalWrite(redledPin, HIGH);
              delay(5000);
              digitalWrite(redledPin, LOW);
              digitalWrite(buzzerPin, HIGH);
              delay(500);

              return;
            }
          }
          else
          {
            turnLeft();
            delay(300);
            stopMotors();
            readSensor();
            if (LFir[0] == 1 || LFir[1] == 1 || LFir[2] == 1 || LFir[3] == 1 || LFir[4] == 1)
            {
              Serial.println("");
            }
            else
            {
              //      turnRight();
              while (LFir[2] == 0 && (LFir[0] == 0 || LFir[4] == 0))
              {
                turnRight();
                delay(200);
                stopMotors();
                readSensor();
              }
            }
          }
        }
        moveForward(error);
      }
      return;
    }
    delay(200);
  }
}
