/*////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////
////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////*/

#include <Arduino.h> //biblioteki
#include <RC5.h>
#include <Servo.h>

#define L_PWM 5 //piny na stałe
#define L_DIR 4
#define R_PWM 6
#define R_DIR 9
#define PWM_MAX 165
Servo serwo;

#define SERWO_PIN 11
#define GRANICA 850
#define R_LINE_SENSOR A0
#define L_LINE_SENSOR A1
#define BUZZER 10
#define LED 13
#define TSOP_PIN 3
#define trigPin 7 //piny od czujnika odleglosci
#define echoPin 8
//define linijka 12!

Adafruit_NeoPixel linijka = Adafruit_NeoPixel(8, 12, NEO_GRB + NEO_KHZ800); //konfiguracja linijki PIN 12!!!!!
Servo serwo;                                                                //serwo 11
RC5 rc5(TSOP_PIN);                                                          //Informacja o podłączeniu odbiornika TSOP 3

byte address; //byte zmienna od 0-255
byte command;
byte toggle;
byte togglePoprzedni = 0;

int zmierzOdleglosc();
int predkoscObrotu = 30;

//Funkcje
void offLed();          // wyłączenie linijki
void speed();           //przyśpieszanie pracą silnika
void automat();         // automatyczna jazda
void square();          // automatyczna jazda kwadrat
void horn();            //klakson
void lineFollow();      //śledzenie linii
void leftMotor(int V);  //lewy silnik
void rightMotor(int V); //prawy
void stopMotors();      //wyłączenie silników
boolean leftSensor();   //do linefollowera fotorezystor
boolean rightSensor();  //j.w.

volatile int stanRobota = 1; //do maszyny stanów

/*////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP
/////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP*/

void setup()
{
  //Konfiguracja pinow od mostka H
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, 0); //Wylaczenie buzzera
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 0); //Wylaczenie diody

  //Czujnik odleglosci
  pinMode(trigPin, OUTPUT); //Pin, do którego podłączymy trig jako wyjście
  pinMode(echoPin, INPUT);  //a echo, jako wejście

  serwo.attach(SERWO_PIN); //Inicjalizacja
  linijka.begin();         //Inicjalizacja LED
  Serial.begin(9600);      // inicjalizacja monitor szeregowy

  linijka.setPixelColor(0, linijka.Color(0, 1, 0)); //Dioda nr 1 świeci na zielono r,g,b 0-255
  linijka.show();

  delay(1000);
  offLed();        // Wyłączenie linijki LED
  serwo.write(90); //Serwo na pozycje srodkowa 90 (bo zakres 0-180)
}

/*////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////
/////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////*/

void loop()
{
  switch (stanRobota)
  {
  case 1:
    if (command == 1)
    {
      lineFollow();
    }
  }

  if (rc5.read(&toggle, &address, &command))
  {
    switch (command)
    {
    case 13:
      horn();
      break;

    case 80:
      offLed();
      linijka.setPixelColor(0, linijka.Color(0, 0, 1));
      linijka.show();
      leftMotor(predkoscObrotu); //Do przodu
      rightMotor(predkoscObrotu);
      speed();
      break;

    case 81:
      offLed();
      linijka.setPixelColor(0, linijka.Color(0, 0, 1));
      linijka.show();
      leftMotor(-predkoscObrotu); //Do tyłu
      rightMotor(-predkoscObrotu);
      speed();
      break;

    case 85:
      offLed();
      linijka.setPixelColor(0, linijka.Color(0, 0, 1));
      linijka.show();
      leftMotor(-predkoscObrotu); //Obrót w lewo
      rightMotor(predkoscObrotu);
      speed();
      break;

    case 86:
      offLed();
      linijka.setPixelColor(0, linijka.Color(0, 0, 1));
      linijka.show();
      leftMotor(predkoscObrotu); //Obrót w prawo
      rightMotor(-predkoscObrotu);
      speed();
      break;

    case 87:
      offLed();
      stopMotors(); //Stop
      predkoscObrotu = 30;
      digitalWrite(BUZZER, 1);
      delay(20);
      digitalWrite(BUZZER, 0);
      break;

    case 2:
      square(); //rysuje kwadrat
      digitalWrite(BUZZER, 1);
      delay(500);
      digitalWrite(BUZZER, 0);
      break;

    case 3:
      for (int i = 0; i < 150; i++)
      {
        offLed();
        automat(); //automatyczna jazda
      }
      stopMotors();
      digitalWrite(BUZZER, 1);
      delay(500);
      digitalWrite(BUZZER, 0);
      break;

    case 1: //zapala linijkę, w switch (stanRobota) pod 1 działa funkcja bez przerwy śledzenie linii
      for (int i = 0; i < 8; i++)
      {
        linijka.setPixelColor(i, linijka.Color(0, 0, 1)); //Dioda nr 1 świeci na zielono
        linijka.show();
      }
      break;
    }
  }

  ///////

  ///////

  //Jeśli odebrano komendę pokaż w monitorze szeregowym, do sprawdzania case na pilocie
  if (rc5.read(&toggle, &address, &command))
  {
    Serial.print("A:");
    Serial.print(address);
    Serial.print(" K:");
    Serial.print(command);
    Serial.print(" T:");
    Serial.println(toggle);
  }
}

/*///FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE///
////FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE//////FUNKCJE///*/

boolean leftSensor()
{
  if (analogRead(L_LINE_SENSOR) > GRANICA)
  {           //Jesli czujnik widzi linie, to
    return 1; //Zwroc 1
  }
  else
  {           //Jesli czujnik nie jest nad linią, to
    return 0; //Zwroc 0
  }
}

boolean rightSensor()
{
  if (analogRead(R_LINE_SENSOR) > GRANICA)
  {           //Jesli czujnik widzi linie, to
    return 1; //Zwroc 1
  }
  else
  {           //Jesli czujnik nie jest nad linią, to
    return 0; //Zwroc 0
  }
}

void leftMotor(int V)
{
  if (V > 0)
  { //Jesli predkosc jest wieksza od 0 (dodatnia)
    V = map(V, 0, 100, 0, PWM_MAX);
    digitalWrite(L_DIR, 0); //Kierunek: do przodu
    analogWrite(L_PWM, V);  //Ustawienie predkosci
  }
  else
  {
    V = abs(V); //Funkcja abs() zwroci wartosc V  bez znaku
    V = map(V, 0, 100, 0, PWM_MAX);
    digitalWrite(L_DIR, 1); //Kierunek: do tyłu
    analogWrite(L_PWM, V);  //Ustawienie predkosci
  }
}

void rightMotor(int V)
{
  if (V > 0)
  { //Jesli predkosc jest wieksza od 0 (dodatnia)
    V = map(V, 0, 100, 0, PWM_MAX);
    digitalWrite(R_DIR, 0); //Kierunek: do przodu
    analogWrite(R_PWM, V);  //Ustawienie predkosci
  }
  else
  {
    V = abs(V); //Funkcja abs() zwroci wartosc V  bez znaku
    V = map(V, 0, 100, 0, PWM_MAX);
    digitalWrite(R_DIR, 1); //Kierunek: do tyłu
    analogWrite(R_PWM, V);  //Ustawienie predkosci
  }
}

void stopMotors()
{
  analogWrite(L_PWM, 0); //Wylaczenie silnika lewego
  analogWrite(R_PWM, 0); //Wylaczenie silnika prawego
}

void lineFollow()
{

  if (leftSensor() == 0 && rightSensor() == 0)
  {                //Jesli czujniki nie widza linii
    leftMotor(50); //Jazda prosto
    rightMotor(50);
  }
  else if (leftSensor() == 1)
  {               //Jesli lewy czujnik widzi linie
    leftMotor(0); //Jazda po łuku w lewo
    rightMotor(60);
  }
  else if (rightSensor() == 1)
  {                //Jesli prawy czujnik widzi linie
    leftMotor(60); //Jazda po łuku w prawo
    rightMotor(0);
  }
}

void horn()
{
  digitalWrite(BUZZER, HIGH);
  delay(500);
  digitalWrite(BUZZER, LOW);
  delay(500);
}

void square()
{
  offLed();
  for (int i = 0; i < 4; i++)
  {
    for (int i = 0; i < 8; i++)
    {
      linijka.setPixelColor(i, linijka.Color(0, 1, 1)); //rgb
      linijka.show();
    }
    leftMotor(50);
    rightMotor(46);
    delay(1000);
    offLed();
    leftMotor(-50);
    rightMotor(50);
    delay(310);
  }
  stopMotors();
  offLed();
}

int zmierzOdleglosc()
{
  long czas, dystans;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  czas = pulseIn(echoPin, HIGH);
  dystans = czas / 58;

  return dystans;
}

void automat()
{
  //Czy wykryto przeszkode w zakresie 0-40 cm
  if (zmierzOdleglosc() > 40)
  {
    leftMotor(39); //Jesli nie, to jedz prosto
    rightMotor(43);
  }

  else
  {
    //Jesli przeszkoda
    stopMotors();    //Zatrzymaj robota
    serwo.write(20); //Skrec czujnikiem w prawo
    delay(800);      //Poczekaj 800ms dla ustabilizowania konstrukcji

    //Sprawdz, czy po prawej stronie jest przeszkoda
    if (zmierzOdleglosc() > 40)
    {
      //Jesli jest pusto
      leftMotor(40);
      rightMotor(-40);
      delay(300); //Obracaj w prawo przez 400 ms
    }
    else
    {
      //Jeśli po prawej jest przeszkoda
      serwo.write(160); //Obroc czujnik w lewo
      delay(800);       //Poczekaj 800ms dla ustabilizowania konstrukcji

      //Sprawdz, czy po lewej stronie jest przeszkoda
      if (zmierzOdleglosc() > 40)
      {
        //Jesli jest pusto
        leftMotor(-40);
        rightMotor(40);
        delay(300); //Obracaj w lewo przez 400 ms
      }

      else
      {

        //Jesli z przodu, z lewej i prawej jest przeszkoda
        digitalWrite(BUZZER, 1);
        delay(500);
        digitalWrite(BUZZER, 0);
        //Daj sygnal buzzerem
      }
    }
    //Po sprawdzeniu przeszkod po bokach
    //Ustaw czujnik prosto
    serwo.write(90);
  }
  for (int i = 0; i < 8; i++)
  {
    linijka.setPixelColor(i, linijka.Color(0, 0, 1));
    linijka.show();
  }
  //Opoznienie 100ms, ponieważ nie ma potrzeby sprawdzać przeszkod czesciej
  delay(80);
  digitalWrite(BUZZER, 1);
  offLed();
  delay(20);
  digitalWrite(BUZZER, 0);
}

void speed()
{
  //Jeśli bit toggle jest taki sam jak poprzednio
  if (toggle == togglePoprzedni)
  {
    predkoscObrotu = predkoscObrotu + 6; //Zwieksz predkosc obrotu o 3

    //Jeśli wartość prędkości przekroczy 90
    if (predkoscObrotu >= 90)
    {
      predkoscObrotu = 90; //To trzymaj na 90
    }
  }
  else
  { //Jeśli bit toggle jest różny
    //Ustaw predkosc na standardową
    predkoscObrotu = 30;
  }
  //Zapamiętanie poprzedniej wartości toggle
  togglePoprzedni = toggle;
}

void offLed()
{
  for (int i = 0; i < 8; i++)
  {
    linijka.setPixelColor(i, linijka.Color(0, 0, 0)); //Dioda nr 1 świeci na zielono
    linijka.show();
  }
}
