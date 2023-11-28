#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//LIBRERIAS

//LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

//ENCODER
#define ENCA 2

//MOTOR
int RPWM = 5;
int LPWM = 6; 
int L_EN = 8;
int R_EN = 8;

//LECTURA ENCODER
volatile int pulsos = 0;
unsigned long timeold;
float resolution = 64;
float ms = 0;
unsigned long prevEncoderValue = 0;
int encoderValue = 0;
unsigned long prevTime = 0;

//CONTROL
float kp = 2.70168;
float ki = 26132.9;
long currT;
float deltaT;
float e;
float eprev;
float integral;
float u;
float uprev;
float power;
int pwm;

float altosetpoint = 0.20;
float mediosetpoint = 0.10;
float bajosetpoint = 0.05;

//BOTONES Y
int botonbajaY = 29;
int botonmediaY = 25;
int botonaltaY = 30;

//DIRECCION
int joystickY = A0;

//LIMITSWITCH
int limitswitchY = 46; 

//VOIDS
void velocidadBaja();
void velocidadMedia();
void velocidadAlta();
void direccionY();
void encoderReading();
void fincarreraY();


void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(botonbajaY, INPUT);
  pinMode(botonmediaY, INPUT);
  pinMode(botonaltaY, INPUT);
  pinMode(joystickY, INPUT);
  pinMode(limitswitchY, INPUT);

  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENCA), encoderReading, RISING);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("GripperGrabber");
}

void loop() {
  currT = millis();
  deltaT = (float(currT - prevTime))/(1.0e3);
  prevTime = currT;

  //LECTURA RPM
  if ((millis() - prevEncoderValue) >= 10) {
    unsigned long elapsedTime = millis() - prevTime;
    ms = float(pulsos * 0.04915469/4);
    Serial.println(ms);
    pulsos = 0;
    prevEncoderValue = elapsedTime;
    }

  //VELOCIDADES
  if(digitalRead(botonbajaY) == HIGH && digitalRead(botonmediaY) == LOW && digitalRead(botonaltaY) == LOW){
    velocidadBaja();
  }else if (digitalRead(botonbajaY) == LOW && digitalRead(botonmediaY) == HIGH && digitalRead(botonaltaY) == LOW){
    velocidadMedia();
  }else if (digitalRead(botonbajaY) == LOW && digitalRead(botonmediaY) == LOW && digitalRead(botonaltaY) == HIGH){
    velocidadAlta();
  } else if(digitalRead(botonbajaY) == LOW && digitalRead(botonmediaY) == LOW && digitalRead(botonaltaY) == LOW){
    lcd.setCursor(0,3);
    lcd.print("                    ");
  }else if(digitalRead(botonbajaY) == LOW && digitalRead(botonmediaY) == LOW && digitalRead(botonaltaY) == LOW){
    lcd.setCursor(0,3);
    lcd.print("                    ");
  }else{
  }
}

//VOID VELOCIDAD BAJA
void velocidadBaja(){
  lcd.setCursor(1,2);
  lcd.print("Velocidad media");

  e = bajosetpoint - ms;
  u = kp*e + 0.01*ki*e;

  power = fabs(u);

  pwm = map(power, 0, 18, 0, 254);

  eprev = e;  
  uprev = u;
  direccionY();
}

//VOID VELOCDIAD MEDIA
void velocidadMedia(){
  lcd.setCursor(1,2);
  lcd.print("Velocidad baja");
  e = mediosetpoint - ms;
  u = kp*e + 0.01*ki*e;

  power = fabs(u);

  pwm = map(power, 0, 18, 0, 254);

  eprev = e;  
  uprev = u;
  direccionY();
}
    
//VOID VELOCDIAD ALTA
void velocidadAlta(){
  lcd.setCursor(1,2);
  lcd.print("Velocidad alta ");
  e = altosetpoint - ms;
  u = kp*e + 0.01*ki*e;

  power = fabs(u);
  pwm = map(power, 0, 18, 0, 254);

  eprev = e;  
  uprev = u;
  direccionY();
}

void direccionY(){
  int joystickYValue = analogRead(joystickY);
  if (joystickYValue < 400) {
    lcd.setCursor(1,3);
    lcd.print("Subiendo...      ");
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  } else if (joystickYValue > 600) {
      lcd.setCursor(1,3);
      lcd.print("Bajando...      ");
      analogWrite(RPWM, 0);
      analogWrite(LPWM, pwm);
  } else {
      lcd.setCursor(1,3);
      lcd.print("                 ");;
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 0);
  }
}


void encoderReading(){
  pulsos++;
  }