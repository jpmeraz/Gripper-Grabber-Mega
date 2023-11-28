#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//PANTALLA
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
// SWITCHES
int limit1 = 46;
int limit2 = 44;
int limit3 = 19;
int b1 = 30;
int b2 = 29;
int b3 = 25;
int b4 = 26;
int b5 = 22;
//MANEJO      
int reset = 2;
int paro = 18;
int joyX = A0;
int joyY = A1;


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.println("Inicializando LCD");
  lcd.init();
  lcd.backlight();
  lcd.clear();
  Serial.begin(9600);

  pinMode(limit1, INPUT);
  pinMode(limit2, INPUT);
  pinMode(limit3, INPUT);
  pinMode(b1, INPUT);
  pinMode(b2, INPUT);
  pinMode(b3, INPUT);
  pinMode(b4, INPUT);
  pinMode(b5, INPUT);
  pinMode(reset, INPUT);
  pinMode(paro, INPUT);
  pinMode(joyX, INPUT);
  pinMode(joyY, INPUT);


}

// the loop function runs over and over again forever
void loop() {
    //PANTALLA LED
  lcd.setCursor(0, 1);
  lcd.print("Buen dia :)");
  // LIMIT SWITCHES
  int lestado1 = digitalRead(limit1);
  int lestado2 = digitalRead(limit2);
  int lestado3 = digitalRead(limit3);
  Serial.print("Limit1: ");
  Serial.println(lestado1);
  Serial.print("Limit2: ");
  Serial.println(lestado2);
  Serial.print("Limit3: ");
  Serial.println(lestado3);
  //BOTONES
  /*/ int bestado1 = digitalRead(b1);
  int bestado2 = digitalRead(b2);
  int bestado3 = digitalRead(b3);
  int bestado4 = digitalRead(b4);
  int bestado5 = digitalRead(b5);
  Serial.print("Boton1: ");
  Serial.println(bestado1);
  Serial.print("Boton2: ");
  Serial.println(bestado2);
  Serial.print("Boton3: ");
  Serial.println(bestado3);
  Serial.print("Boton4: ");
  Serial.println(bestado4);
  Serial.print("Boton5: ");
  Serial.println(bestado5);  
  //CONTROLADORES
  int restado = digitalRead(reset);
  int pestado = digitalRead(paro);
  int jxestado = digitalRead(joyX);
  int jyestado = digitalRead(joyY);
  Serial.print("Reset: ");
  Serial.println(restado);
  Serial.print("Paro: ");
  Serial.println(pestado);
  Serial.print("JoyX: ");
  Serial.println(jxestado);
  Serial.print("JoyY: ");
  Serial.println(jyestado);


  delay(4000);
  /*/
  delay(2000);
}


