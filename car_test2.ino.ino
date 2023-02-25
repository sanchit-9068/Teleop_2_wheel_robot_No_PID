#include <math.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
//#include <Servo.h>
#include <std_msgs/UInt8.h>

void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
/*out3 and out4 connected to left motor
 * so in3 and in4 for left motor    
 * in3=0 && in4=1 left wheel clockwise || in3=1 && in4=0 left wheel anticlockwise
 * in1=0 && in2=1 right wheel clockwise || in1=1 && in2=0 right wheel anticlockwise
 * going straight -- left anti && right clock--(in3=1 && in4=0 && in1=0 && in2=1)
 * going back-- left clock && right anti--(in3=0 && in4=1 && in1=1 && in2=0)
 * turning right--left anti && right anti --(in3=1 && in4=0 && in1=1 && in2=0)
 * turning left-- left clock && right clock--(in3=0 && in4=1 && in1=0 && in2=1)
 */
const uint8_t EN_A = D5;
const uint8_t IN_1 = D7;
const uint8_t IN_2 = D8;
const uint8_t IN_3= D3;
const uint8_t IN_4 = D4;
const uint8_t EN_B = D6;

#ifndef ACCESS_POINT_SSID
ESP8266WiFiMulti wifi;
#endif

// ROS serial server
IPAddress server(192,168,110,114);
ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);
//ros::Subscriber<std_msgs::UInt8> sub2("/servo", &servo_cb);
bool _connected = false;

void setup() 
{
  setupWiFi();
  pinMode(EN_A,OUTPUT);
  pinMode(EN_B,OUTPUT);
  pinMode(IN_1,OUTPUT);
  pinMode(IN_2,OUTPUT);
  pinMode(IN_3,OUTPUT);
  pinMode(IN_4,OUTPUT);
  delay(2000);
  Serial.begin(9600);
  Serial.println();
  stop();
  // Connect to rosserial socket server and init node. (Using default port of 11411)
  Serial.printf("Connecting to ROS serial server at %s\n", server.toString().c_str());
  node.getHardware()->setConnection(server);
  node.initNode();
  node.subscribe(sub);
}

void stop()
{
  digitalWrite(IN_1,0);
  digitalWrite(IN_2,0);
  digitalWrite(IN_3,0);
  digitalWrite(IN_4,0);
  analogWrite(EN_A,0);
  analogWrite(EN_B,0);
}

void setupWiFi()
{
#ifdef ACCESS_POINT_SSID

  WiFi.disconnect();
  Serial.println("Creating Wifi network");
  if (WiFi.softAP(ACCESS_POINT_SSID))
  {
    Serial.println("Wifi network created");
    Serial.print("SSID: ");
    Serial.println(WiFi.softAPSSID());
    Serial.print("IP:   ");
    Serial.println(WiFi.softAPIP());
  }

#else

  WiFi.softAPdisconnect();
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  wifi.addAP("Sanchit's Galaxy S20 FE 5G", "my tenor");
//  wifi.addAP("--wifi_2--", "--password_2--");
//  wifi.addAP("--wifi_3--", "--password_3--");

  Serial.println("Connecting to Wifi");
  while (wifi.run() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   ");
  Serial.println(WiFi.localIP());

#endif
}


void onTwist(const geometry_msgs::Twist &msg)
{
    if (!_connected)
  {
    stop();
    return;
  }
  analogWrite(EN_A,255);
  analogWrite(EN_B,255);
  float straight=msg.linear.x;
  float turn=msg.angular.z; 
  if(straight>0.0)
  {
    //(in3=1 && in4=0 && in1=0 && in2=1)
//    IN_3=1;
//    IN_4=0;
//    IN_1=0;
//    IN_2=1;
    digitalWrite(IN_3,1);
    digitalWrite(IN_4,0);
    digitalWrite(IN_1,0);
    digitalWrite(IN_2,1);
  }
  else if(straight<0.0)
  {
//    IN_3=0;
//    IN_4=1;
//    IN_1=1;
//    IN_2=0;
    digitalWrite(IN_3,0);
    digitalWrite(IN_4,1);
    digitalWrite(IN_1,1);
    digitalWrite(IN_2,0);
  }
  else if(turn>0.0)
  {
    //(in3=1 && in4=0 && in1=1 && in2=0)
//    IN_3=1;
//    IN_4=0;
//    IN_1=1;
//    IN_2=0;
    digitalWrite(IN_3,1);
    digitalWrite(IN_4,0);
    digitalWrite(IN_1,1);
    digitalWrite(IN_2,0);
  }
  else if(turn<0.0)
  {
    //(in3=0 && in4=1 && in1=0 && in2=1)
//    IN_3=0;
//    IN_4=1;
//    IN_1=0;
//    IN_2=1;
    digitalWrite(IN_3,0);
    digitalWrite(IN_4,1);
    digitalWrite(IN_1,0);
    digitalWrite(IN_2,1);
  }
  else
  {
//    IN_3=0;
//    IN_4=0;
//    IN_1=0;
//    IN_2=0;
    digitalWrite(IN_3,0);
    digitalWrite(IN_4,0);
    digitalWrite(IN_1,0);
    digitalWrite(IN_2,0);
  }

}


void loop() {
  // put your main code here, to run repeatedly:
  if (!rosConnected())
    stop();
  node.spinOnce();

}
bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool connected = node.connected();
  if (_connected != connected)
  {
    _connected = connected;
    digitalWrite(LED_BUILTIN, !connected); // false -> on, true -> off
    Serial.println(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}
