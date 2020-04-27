/*////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////
////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////////ROBOT/STEROWANIE/IR////*/

//PROBLEM
//Aktualnie robot jeździ sterowanie pilotem,
// podstawowe sterowanie działa, line follower po wciśnięciu 1 też, 2 kwadrat też
//PROBLEM kiedy wciskam 3, robot przechodzi w tryb automatycznej jazdy ale, NIE MOGĘ PILOTEM TEGO WYŁĄCZYĆ, tak jak w przypadku jazdy line follower

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

RC5 rc5(TSOP_PIN); //Informacja o podłączeniu odbiornika TSOP
byte address;      //byte zmienna od 0-255
byte command;
byte toggle;

int zmierzOdleglosc();
void automat();
void square();
void horn();
void lineFollow();
void leftMotor(int V);
void rightMotor(int V);
void stopMotors();
boolean leftSensor();
boolean rightSensor();

//Piny od czujnika odleglosci
#define trigPin 7
#define echoPin 8

volatile int stanRobota_A = 1;
volatile int stanRobota_B = 1;

//1000=1sekunda (1000UL)
unsigned long aktualnyCzas = 0;
unsigned long zapamietanyCzas = 0;
unsigned long roznicaCzasu = 0;

/*////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP
/////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP////FUNKCJA//SETUP*/

void setup()
{
  //Konfiguracja pinow od mostka H
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  //Konfiguracja pozostalych elementow
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, 0); //Wylaczenie buzzera
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 0); //Wylaczenie diody

  //Czujnik odleglosci
  pinMode(trigPin, OUTPUT); //Pin, do którego podłączymy trig jako wyjście
  pinMode(echoPin, INPUT);  //a echo, jako wejście

  //Serwo do pinu 11
  serwo.attach(SERWO_PIN);
  //Serwo na pozycje srodkowa 90 (bo zakres 0-180)
  serwo.write(90);

  Serial.begin(9600);
}

/*////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////
/////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////////LOOP//DZIAŁA ZAWSZE////*/

void loop()
{

  aktualnyCzas = millis();                       // Pobieraj liczbę milisekund od startu
  roznicaCzasu = aktualnyCzas - zapamietanyCzas; // Różnica czasu

  switch (stanRobota_A)
  {
  case 1: //Line follower
    if (command == 1)
    {
      lineFollow();
    }
    else if (command == 87)
    {
      stopMotors();
    }
    else if (command == 3)
    {
      automat();
    }
    break;
  }

  if (rc5.read(&toggle, &address, &command))
  {
    switch (command) // łatwo sprawdzić na pilocie, każdy przycisk ma swój command np. jeden to command 1
    {
    case 13:
      horn();
      break;
    case 80:
      leftMotor(70); //Do przodu
      rightMotor(70);
      break;
    case 81:
      leftMotor(-50); //Do tyłu
      rightMotor(-50);
      break;
    case 85:
      leftMotor(-40); //Obrót w lewo
      rightMotor(40);
      break;
    case 86:
      leftMotor(40); //Obrót w prawo
      rightMotor(-40);
      break;
    case 87: //Stop
      stopMotors();
      break;
    case 2:
      square();
      break;
    }
  }
  ///////

  ///////
  //Jeśli odebrano komendę pokaż w monitorze szeregowym
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

//Do fotorezystorów jeżeli pod progiem lewy zwróć 1
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
//c.d. prawy
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

//Obrót w prawo
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

//Obrót w prawo
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

//Stop silników
void stopMotors()
{
  analogWrite(L_PWM, 0); //Wylaczenie silnika lewego
  analogWrite(R_PWM, 0); //Wylaczenie silnika prawego
}

//Śledzenie linii
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

//Klakson
void horn()
{
  digitalWrite(BUZZER, HIGH);
  delay(500);
  digitalWrite(BUZZER, LOW);
  delay(500);
}

//Rysuj linię
void square()
{
  for (int i = 0; i < 4; i++)
  {
    leftMotor(50);
    rightMotor(46);
    delay(1000);
    leftMotor(-50);
    rightMotor(50);
    delay(310);
  }
  stopMotors();
}

//Standard do podawania odleglosci w cm
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

//Funkcja odpwiedzialna za wykrywanie przeszkód serwo + Ultradzwiek///////Funkcja odpwiedzialna za wykrywanie przeszkód serwo + Ultradzwiek/////
//Funkcja odpwiedzialna za wykrywanie przeszkód serwo + Ultradzwiek///////Funkcja odpwiedzialna za wykrywanie przeszkód serwo + Ultradzwiek/////

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

      //Sprawdz, czy po lowej stronie jest przeszkoda
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

  //Opoznienie 100ms, ponieważ nie ma potrzeby sprawdzać przeszkod czesciej
  delay(100);
}