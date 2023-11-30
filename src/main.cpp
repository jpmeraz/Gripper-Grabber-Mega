#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// LIBRERIAS

// LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ENCODER
#define ENCA 2
// MOTOR REDUCTOR
#define RPWM 5
#define LPWM 6
#define L_EN 8
#define R_EN 8
// STEPPER MOTOR
#define direccion_stepper 10
#define pulsos_stepper 9
// BOTONES Y
#define botonbajaY 25
#define botonmediaY 29
#define botonaltaY 30
// BOTONES X
#define botonbajaX 22
#define botonlaltaX 26
// BOTON PARO
#define paro 18
// BOTON RESET
#define reset 2
// JOYSTICKS
#define joystickY A0
#define joystickX A1
// LIMITSWITCH
#define limitswitchY 46
#define limitInicio 19
#define limitFin 44

// LECTURA ENCODER
volatile int pulsos = 0;
unsigned long timeold;
float resolution = 64;
// float ms = 0;
unsigned long prevEncoderValue = 0;
int encoderValue = 0;
unsigned long prevTime = 0;

// CONTROL
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

// SETPOINTS VELOCIDADES
float altosetpoint = 0.12;
float mediosetpoint = 0.04;
float bajosetpoint = 0.02;
const int velbaja = 60;
const int PPRbaja = 150;
const int velalta = 20;
const int PPRalta = 200;

// FLAGS INTERNAS
bool reset_needed = false;

// VOIDS
void direccionY(int pwm);
void direccionX(int velocidad, int PPR);
void control_velocidades(float setpoint, String lcd_print);
float rpm();
void encoderReading();
void emergencia();
void reseteo();

void setup()
{
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
  pinMode(limitInicio, INPUT);
  pinMode(limitFin, INPUT);
  pinMode(reset, INPUT);
  pinMode(paro, INPUT);
  pinMode(direccion_stepper, OUTPUT);
  pinMode(pulsos_stepper, OUTPUT);
  pinMode(botonbajaX, INPUT);
  pinMode(botonlaltaX, INPUT);

  attachInterrupt(digitalPinToInterrupt(paro), emergencia, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCA), encoderReading, RISING);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("GripperGrabber");
}

void loop()
{
  lcd.setCursor(3, 0);
  lcd.print("GripperGrabber");

  while (!digitalRead(paro))
  {
    lcd.setCursor(1, 2);
    lcd.print("Emergencia activa");
    lcd.setCursor(0, 1);
    lcd.print("                    ");
  }

  if (reset_needed)
  {
    lcd.setCursor(0, 2);
    lcd.print("                   ");
    lcd.setCursor(0, 3);
    lcd.print("  Reset necesario");
  }

  // Al oprimir el botón de reset se inicia la secuencia correspondiente.
  if (digitalRead(reset) && digitalRead(paro))
  {
    reseteo();
  }

  if (!reset_needed && digitalRead(paro))
  {
    // SE ACTIVAN LOS ENABLES Y SE OBTIENEN LAS RPMs
    digitalWrite(L_EN, HIGH);
    digitalWrite(R_EN, HIGH);

    // VELOCIDADES
    // Para seleccionar la velocidad a utilizar se verifica el estado de todos los botones y solamente se activa un estado al tener un solo botón presionado.
    if (digitalRead(botonbajaY) && !digitalRead(botonmediaY) && !digitalRead(botonaltaY) && !digitalRead(botonlaltaX) && !digitalRead(botonbajaX))
    {
      // Velocidad baja del eje Y
      control_velocidades(bajosetpoint, "Velocidad baja");
      Serial.println("Velocidad baja");
    }
    else if (!digitalRead(botonbajaY) && digitalRead(botonmediaY) && !digitalRead(botonaltaY) && !digitalRead(botonlaltaX) && !digitalRead(botonbajaX))
    {
      // Velocidad media del eje Y
      control_velocidades(mediosetpoint, "Velocidad media");
    }
    else if (!digitalRead(botonbajaY) && !digitalRead(botonmediaY) && digitalRead(botonaltaY) && !digitalRead(botonlaltaX) && !digitalRead(botonbajaX))
    {
      // Velocidad alta del eje Y
      control_velocidades(altosetpoint, "Velocidad alta");
    }
    else if (!digitalRead(botonbajaY) && !digitalRead(botonmediaY) && !digitalRead(botonaltaY) && digitalRead(botonlaltaX) && !digitalRead(botonbajaX))
    {
      // Velocidad alta del eje X
      lcd.setCursor(1, 2);
      lcd.print("Velocidad alta");
      direccionX(velalta, PPRalta);
    }
    else if (!digitalRead(botonbajaY) && !digitalRead(botonmediaY) && !digitalRead(botonaltaY) && !digitalRead(botonlaltaX) && digitalRead(botonbajaX))
    {
      // Velocidad alta del eje X
      lcd.setCursor(1, 2);
      lcd.print("Velocidad baja");
      direccionX(velbaja, PPRbaja);
    }
    else
    {
      lcd.setCursor(0, 1);
      lcd.print("                    ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("                    ");
    }
  }
}

// VOID VELOCIDAD BAJA
void control_velocidades(float setpoint, String lcd_print)
{
  lcd.setCursor(1, 2);
  lcd.print(lcd_print);

  e = setpoint - rpm();
  u = kp * e + 0.01 * ki * e;
  power = fabs(u);
  int pwm = map(power, 0, 18, 0, 254);

  eprev = e;
  uprev = u;
  direccionY(pwm);
}

void direccionY(int pwm)
{
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
  int joystickYValue = analogRead(joystickY);
  if (joystickYValue < 400 && digitalRead(limitswitchY) && !reset_needed)
  {
    lcd.setCursor(1, 3);
    lcd.print("Subiendo...      ");
    Serial.println(pwm);
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  }
  else if (joystickYValue > 600 && !reset_needed)
  {
    lcd.setCursor(1, 3);
    lcd.print("Bajando...      ");
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
  }
  else if (!limitswitchY)
  {
    lcd.setCursor(0, 3);
    lcd.print("Limite alcanzado...");
  }
  else if (reset_needed && digitalRead(limitswitchY))
  {
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  }
  else
  {
    lcd.setCursor(1, 3);
    lcd.print("                   ");
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}

void direccionX(int velocidad, int PPR)
{
  bool contador = true;
  while (reset_needed && !digitalRead(limitInicio)) {
    digitalWrite(direccion_stepper, true);
    for (int i = 0; i < PPR; i++)
    {
      digitalWrite(pulsos_stepper, HIGH);
      delayMicroseconds(velocidad);        
      digitalWrite(pulsos_stepper, LOW);
      delayMicroseconds(velocidad);
    }
  }
  while (analogRead(joystickX) < 400 && !digitalRead(limitInicio) && !reset_needed)
  {
    if (contador)
    {
      lcd.setCursor(1, 3);
      lcd.print("Moviendo a la izq.");
    }
    contador = false;
    digitalWrite(direccion_stepper, true);
    for (int i = 0; i < PPR; i++)
    {
      digitalWrite(pulsos_stepper, HIGH);
      delayMicroseconds(velocidad);
      digitalWrite(pulsos_stepper, LOW);
      delayMicroseconds(velocidad);
    }
  }
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  contador = true;
  while (analogRead(joystickX) > 600 && !digitalRead(limitFin) && !reset_needed)
  {
    if (contador)
    {
      lcd.setCursor(1, 3);
      lcd.print("Moviendo a la der.");
    }
    contador = false;
    // stepper(true, velocidad, PPR);
    digitalWrite(direccion_stepper, false);
    for (int i = 0; i < PPR; i++)
    {
      digitalWrite(pulsos_stepper, HIGH);
      delayMicroseconds(velocidad);
      digitalWrite(pulsos_stepper, LOW);
      delayMicroseconds(velocidad);
    }
  }
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  
}

void encoderReading()
{
  pulsos++;
}

void emergencia()
{
  {
    digitalWrite(L_EN, LOW);
    digitalWrite(R_EN, LOW);
    digitalWrite(pulsos_stepper, LOW);
    reset_needed = true;
  }
}

void reseteo()
{ 
  reset_needed = true;
  lcd.setCursor(0, 3);
  lcd.print("  Reset necesario");
  while (digitalRead(limitswitchY))
  {
    control_velocidades(bajosetpoint, "Reiniciando Eje Y...");
  }
  while(!digitalRead(limitInicio)){
      lcd.setCursor(0, 2);
      lcd.print("Reiniciando Eje X...");
      direccionX(velbaja, PPRbaja);
  }
  while (digitalRead(reset))
  {
    lcd.setCursor(0, 2);
    lcd.print(" Reinicio completo  ");
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
  reset_needed = false;
  lcd.clear();
}

float rpm()
{
  float ms = 0.0;
  currT = millis();
  deltaT = (float(currT - prevTime)) / (1.0e3);
  prevTime = currT;

  // LECTURA RPM
  if ((millis() - prevEncoderValue) >= 10)
  {
    unsigned long elapsedTime = millis() - prevTime;
    ms = float(pulsos * 0.04915469 / 4);
    pulsos = 0;
    prevEncoderValue = elapsedTime;
  }
  return ms;
}
