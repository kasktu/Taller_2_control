// Librerías
#include <OneWire.h>
//as5600 - sensor

// Pines del Arduino
const int pinPWM = 9;      // Pin de entrada PWM para controlar la velocidad del motor
const int pinIN1 = 8;      // Pin para controlar la dirección del motor
const int pinManual = A1;
const int pinSensor = A0;  // Pin del sensor

// Variables
#define pwmRes 12   
#define pwmMax 3300 
#define minimo 0
#define Uunits 200

long previousMillis = 0; 

//const int anguloMinimo = 10;    // Ángulo mínimo
//const int anguloMaximo = 130;   // Ángulo máximo
const int anguloReferencia = 90; // Ángulo de referencia
int errorAngulo = 0;
int anguloActual = 0;
int velocidadPWM = 60;// Variable para controlar la velocidad del motor
float lecturaManual=0;
long Ts = 100;   // Sample time in ms
long TsPWM=50;
long TsPWMtest=5000;
long time = 0;  // For auxiliary functions (squarewaves)

void setup() {
  pinMode(A0, INPUT);
  pinMode(pinIN1, OUTPUT);
  pinMode(A1,INPUT);
  pinMode(pinPWM,OUTPUT);
  analogWrite(pinPWM, 0); // Velocidad inicial del motor en 0
  Serial.begin(115200);

    delay(1000);
}

void leersensor() {
  int lecturaSensor = analogRead(pinSensor);
  int lecturaManual = analogRead(pinManual);
  int lecturaPWM = analogRead(pinManual);
  anguloActual = map(lecturaSensor, 0, 1023,-115, 245);
  if (anguloActual>180){
    anguloActual=anguloActual-366;
  }
  lecturaManual = map(lecturaManual,0,1023,0,5);

 // if(pinManual>minimo){ 
          //float U_tl = min(max(pinManual, 0), Uunits); // Saturated Control Output
          //float pwmDuty = int((U_tl/Uunits)*20*pwmMax);
          //analogWrite (pinPWM, pwmDuty);
//}

  //errorAngulo = anguloReferencia - anguloActual;

  // Ajusta la velocidad del motor en función del error del ángulo
  //velocidad = map(abs(errorAngulo), 0, 180, 0, 255);
  //analogWrite(pinPWM, velocidad);
  
}
void PWM(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TsPWM) {
    previousMillis = currentMillis;
    if(anguloActual==anguloReferencia){
      velocidadPWM=velocidadPWM;
    } else if (anguloActual<anguloReferencia and velocidadPWM<90) {
      delay(490);
      velocidadPWM=velocidadPWM+1;
    } else if (anguloActual>anguloReferencia){
      if (velocidadPWM<0){
        velocidadPWM=0;
      }else if (anguloActual-anguloReferencia>10) {
      velocidadPWM=velocidadPWM-3;
      }else{
        velocidadPWM=velocidadPWM-1;
        
      }
    }
  }
}


void PWMTest(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TsPWMtest) {
    previousMillis = currentMillis;
    velocidadPWM=velocidadPWM+1;
  }
}

void loop() {
  leersensor();
  PWM();
  //Serial.print("Ángulo medido: ");
  Serial.print(anguloActual);
  Serial.print(",");
  analogWrite(pinPWM, velocidadPWM);
  //Serial.print("PWM: ");
  Serial.println(velocidadPWM);
  //Serial.print("Lectura manual ");
  //Serial.println(lecturaManual);


  delay(Ts); // Espera durante el tiempo de muestreo
}
