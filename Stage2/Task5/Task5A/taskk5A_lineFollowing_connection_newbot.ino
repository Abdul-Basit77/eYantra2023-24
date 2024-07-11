#include <WiFi.h>

// WiFi credentials
const char *ssid = "Abhay Agrawal";  // Enter your wifi hotspot ssid
const char *password = "kdrv12x7";   // Enter your wifi hotspot password
const uint16_t port = 8004;
const char *host = "192.168.188.144";
float pTerm, iTerm, dTerm;
int previousError;
float kp = 10;  //10
float ki = 0;
float kd = 11;  //11
float output;
int integral, derivative;
int k = 0;

// defining ir sensor pin
const int ir0 = 32;
const int ir1 = 13;  // Corrected pin number
const int ir2 = 12;
const int ir3 = 34;  // Corrected pin number
const int ir4 = 15;

// defining motor pin
const int LMpin1 = 25;
const int LMpin2 = 33;
const int RMpin1 = 27;
const int RMpin2 = 26;

//defining the buzzer and led pin
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
int LFir[] = { 0, 0, 0, 0, 0 };
//int node_arr[] = { 2, 2, 2, 3, 1, 3, 3, 2, 3, 2, 1, 2 };
int node_value = 0;
int no_of_node = 0;
float error;
long startTime;
long endTime;

char incomingPacket[80];
WiFiClient client;
String command = "";

void setup() {
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
  stopMotors();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
  }
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(redledPin, HIGH);
}

void pidCalculations(int error) {
  pTerm = kp * error;
  integral += error;
  iTerm = ki * integral;
  derivative = error - previousError;
  dTerm = kd * derivative;
  output = pTerm + iTerm + dTerm;
  previousError = error;
}
void moveForward(int error) {
  int pid_error = error;
  pidCalculations(pid_error);
  int LM_Speed = 100 + output;  //150 //100
  int RM_Speed = 142 - output;  //180 //142
  ledcWrite(pwm_channel1, LM_Speed);
  ledcWrite(pwm_channel2, 0);
  ledcWrite(pwm_channel3, RM_Speed);
  ledcWrite(pwm_channel4, 0);
}

void Uturn() {
  moveForward(2);
  delay(50);
  ledcWrite(pwm_channel1, 100);  // 200  // Set the left motor speed
  ledcWrite(pwm_channel2, 0);    // Stop the left motor
  ledcWrite(pwm_channel3, 0);    // Stop the right motor
  ledcWrite(pwm_channel4, 100);  // 200

  readSensor();
  int i = 0;
  // Uturn();
  while (i <= 1) {
    if (LFir[2] == 1) {
      i++;
      delay(100);
    }
    readSensor();
  }
  startTime = millis();
  endTime = startTime;
  while ((endTime - startTime) <= 700) {
    if (LFir[2] == 1) {
      break;
    }
    endTime = millis();
    readSensor();
  }
}

void stopMotors() {
  ledcWrite(pwm_channel1, 0);
  ledcWrite(pwm_channel2, 0);
  ledcWrite(pwm_channel3, 0);
  ledcWrite(pwm_channel4, 0);
}

void readSensor() {
  LFir[0] = digitalRead(ir0);
  LFir[1] = digitalRead(ir1);
  LFir[2] = digitalRead(ir2);
  LFir[3] = digitalRead(ir3);
  LFir[4] = digitalRead(ir4);
}
void nodeUpdate() {
  node = node + 1;
}
void turnRight() {

  ledcWrite(pwm_channel1, 110);  //200  // Set the left motor speed
  ledcWrite(pwm_channel2, 0);    // Stop the left motor
  ledcWrite(pwm_channel3, 0);    // Stop the right motor
  ledcWrite(pwm_channel4, 110);  //200
}
void turnLeft() {
  ledcWrite(pwm_channel1, 0);    // Set the left motor speed
  ledcWrite(pwm_channel2, 110);  //200   // Stop the left motor
  ledcWrite(pwm_channel3, 110);  //200  // Stop the right motor
  ledcWrite(pwm_channel4, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connect(host, port)) {
    Serial.println("Connection to host failed");
    delay(200);
    return;
  }
  // delay(500);
  String number_of_path = client.readStringUntil('\n');
  Serial.println(number_of_path);
  int n = number_of_path.toInt();
  Serial.println(n);
  String path_array[n] = {};
  delay(3000);
  for (int i = 0; i < n; i++) {
    path_array[i] = client.readStringUntil('\n');
    Serial.println(path_array[i]);
    delay(5000);
  }

  delay(2000);

  while (1) {
    command = client.readStringUntil('\n');
    Serial.println(command);
    if (command == "RUN") {
      digitalWrite(redledPin, LOW);
      digitalWrite(buzzerPin, LOW);
      delay(1000);
      digitalWrite(buzzerPin, HIGH);
      // int path_number = 0;
      for (int path_no = 0; path_no < n; path_no++) {
        String path = path_array[path_no];
        node = 0;
        node_value = path[node] - '0';

        if (node_value == 4) {
          Uturn();
          stopMotors();
        }
        no_of_node = path.length() - 1;

        while (1) {
          readSensor();

          if (node == no_of_node) {
            if (path_no == n) {
              moveForward(4);
              delay(1000);
              stopMotors();
              digitalWrite(buzzerPin, LOW);
              digitalWrite(redledPin, HIGH);
              delay(5000);
              digitalWrite(redledPin, LOW);
              digitalWrite(buzzerPin, HIGH);
              break;
            }
            command = client.readStringUntil('\n');
            if (command == "yes") {
              stopMotors();
              digitalWrite(buzzerPin, LOW);
              digitalWrite(redledPin, HIGH);
              delay(3000);
              digitalWrite(redledPin, LOW);
              digitalWrite(buzzerPin, HIGH);
              break;
            }
          }
          //node detection
          if (LFir[2] == 1 && LFir[1] == 1 && LFir[3] == 1) {
            stopMotors();
            //digitalWrite(buzzerPin, LOW);
            //digitalWrite(greenledPin, HIGH);
            delay(500);
            //digitalWrite(greenledPin, LOW);
            //digitalWrite(buzzerPin, HIGH);
            nodeUpdate();
            Serial.print("Node: ");
            Serial.println(node);
            // String path = path_array[0];
            node_value = path[node] - '0';
            Serial.println(node_value);
            if (node_value == 2) {
              if (node == 1 && path_no == 0) {
                moveForward(0);
                delay(500);
              } else {
                moveForward(2);
                delay(1000);
              }
            } else if (node_value == 1) {
              moveForward(-1);
              delay(900);
              turnLeft();
              //delay(300);
              int i = 1;
              while (i == 1) {
                readSensor();
                if (LFir[2] == 1) {
                  i++;
                  delay(65);
                }
              }
              stopMotors();
            } else if (node_value == 3) {
              moveForward(3);
              delay(700);
              turnRight();
              //delay(300);
              //Detecting whether it has achieved center line again or not while turning.
              int i = 1;
              while (i == 1) {
                readSensor();
                if (LFir[2] == 1) {
                  i++;
                  delay(65);
                }
              }
              stopMotors();
            }

            error = 1;
            //    Serial.println("hello : ");
          }

          // determine the error
          //center line detection
          if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 1) && (LFir[3] == 0) && (LFir[4] == 0)) {
            error = 1;
          } else if ((LFir[0] == 0) && (LFir[1] == 1) && (LFir[2] == 1) && (LFir[3] == 0) && (LFir[4] == 0)) {
            error = -2;
          } else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 1) && (LFir[3] == 1) && (LFir[4] == 0)) {
            error = 1.5;
          } else if ((LFir[0] == 0) && (LFir[1] == 1) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 0)) {
            error = -1;
          } else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 1) && (LFir[4] == 0)) {
            error = 7;
          } else if (LFir[0] == 0 && LFir[1] == 0 && LFir[2] == 0 && (LFir[3] == 0) && (LFir[4] == 0)) {
            error = 1.5;
          } else if ((LFir[0] == 1) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 0)) {
            error = 4.5;
          } else if ((LFir[0] == 0) && (LFir[1] == 0) && (LFir[2] == 0) && (LFir[3] == 0) && (LFir[4] == 1)) {
            error = -4.5;
          }
          moveForward(error);
        }
      }
      return;
    }
    delay(200);
  }
}
