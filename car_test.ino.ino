#include <math.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>
#include <std_msgs/UInt8.h>
Servo servo;
#define LED_BUILTIN 2 // Remapping the built-in LED since the NodeMcu apparently uses a different one.
#define LED_BUILTIN_RED 16 // If using a NodeMcu v1, then there's another red onboard led.
// The min amount of PWM the motors need to move. Depends on the battery, motors and controller.
//The max amount is defined by PWMRANGE in Arduino.h
#define PWM_MIN 220

// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
void servo_cb(const std_msgs::UInt8 &cmd_msg);

// Pins
const uint8_t R_PWM = D5;
const uint8_t R_BACK = D7;
const uint8_t R_FORW = D8;
const uint8_t L_BACK = D3;
const uint8_t L_FORW = D4;
const uint8_t L_PWM = D6;
//const uint8_t tray=D2;
// Wifi
// If access point is defined, a Wifi network with this name will be created.
// Remove if you want to connect to an existing network.
// #define ACCESS_POINT_SSID "SMARTCAR"

#ifndef ACCESS_POINT_SSID
ESP8266WiFiMulti wifi;
#endif

// ROS serial server
IPAddress server(192,168,57,114 );
ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);
//ros::Subscriber<std_msgs::UInt8> sub2("/servo", &servo_cb);
bool _connected = false;

void setup()
{
  setupPins();
  setupSerial();
  setupWiFi();

  // Connect to rosserial socket server and init node. (Using default port of 11411)
  Serial.printf("Connecting to ROS serial server at %s\n", server.toString().c_str());
  node.getHardware()->setConnection(server);
  node.initNode();
  node.subscribe(sub);
//  node.subscribe(sub2);
}

void setupPins()
{
  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH);

  pinMode(L_PWM, OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  analogWrite(L_PWM,220);
  analogWrite(R_PWM,220);
//  servo.attach(tray);/
  stop();
//  servo.write(90);/

delay(2000);
}

void setupSerial()
{
  Serial.begin(9600);
  Serial.println();
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

void stop()
{
  digitalWrite(L_FORW, 0);
  digitalWrite(L_BACK, 0);
  digitalWrite(R_FORW, 0);
  digitalWrite(R_BACK, 0);
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}
//void servo_cb( const std_msgs::UInt8& cmd_msg){
//  Serial.println(cmd_msg.data);
//  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
//}
void onTwist(const geometry_msgs::Twist &msg)
{
  if (!_connected)
  {
    stop();
    return;
  }

  float l = msg.linear.x;
  float r = msg.linear.y;
  uint16_t lPwm = msg.angular.x;
  uint16_t rPwm = msg.angular.y;

  Serial.println(l);
  Serial.println(r);
  //lPwm=min(uint16_t(550),lPwm);
  //rPwm=min(uint16_t(550),rPwm);
  // Set direction pins and PWM
  digitalWrite(L_FORW, l < 0);
  digitalWrite(L_BACK, l > 0);
  digitalWrite(R_FORW, r > 0);
  digitalWrite(R_BACK, r < 0);
//  analogWrite(L_PWM, lPwm);
//  analogWrite(R_PWM, rPwm);
}

void loop()
{
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
