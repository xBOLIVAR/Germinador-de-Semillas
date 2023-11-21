#include <DHT.h>
#include <Stepper.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define sensor A0
#define sensorSharp A1
#define DHTPIN 4  // Pin donde está conectado el sensor
#define motoBomba 12
#define DHTTYPE DHT11  // Descomentar si se usa el DHT 11
DHT dht(DHTPIN, DHTTYPE);


LiquidCrystal_I2C lcd(0x27, 16, 2);

int lista_humedad[2];
int valor_humedad = 0;
int contador = 0;
bool delayServo, inicio = false;

// Define el número de pasos y los pines del motor
const int stepsPerRevolution = 50;                    // Cambia esto según las especificaciones de tu motor
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);  // Pines de control del motor
// Configura el pin del sensor IR
const int sensorInfra = 2;  // Cambia el número de pin según tu conexión

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(9600);  //iniciailzamos la comunicación
  pinMode(sensorInfra, INPUT);
  pinMode(8, OUTPUT);
  pinMode(motoBomba, OUTPUT);

  digitalWrite(motoBomba, HIGH);

  // Establece la velocidad del motor en pasos por segundo
  myStepper.setSpeed(100);  // Puedes ajustar la velocidad según tus necesidades

  servo1.attach(3);  // Conectar servo1 al pin 3
  // servo2.attach(5);

  dht.begin();
  lcd.init();
  lcd.backlight();
}

void loop() {
  if (!inicio) {
    lcd.setCursor(0, 0);
    lcd.print("Germinador de");
    lcd.setCursor(1, 1);
    lcd.print("Semillas");
    delay(3000);
    lcd.clear();
    inicio = true;
  }
  lcd.setCursor(0, 0);
  lcd.print(dht.readTemperature());
  lcd.print(" C ");
  lcd.print(dht.readHumidity());
  lcd.print("%t");
  Serial.print(dht.readTemperature());
  Serial.print(",");
  Serial.print(dht.readHumidity());
  Serial.println();


  int value_sharp = analogRead(sensorSharp);                // Lee el valor analógico del sensor
  float distance_sharp = 10650.08 / (value_sharp - 27.61);  // Fórmula de conversión, puede variar según el modelo

  Serial.print("Distancia: ");
  Serial.print(distance_sharp);
  Serial.println(" cm");

  //delay(500); // Espera 0.5 segundos antes de realizar la próxima lectura

  // Lee el estado del sensor IR
  servo1.write(170);
  servo2.write(130);

  int infraRojo = digitalRead(sensorInfra);

  // Si el sensor detecta un obstáculo (estado bajo)
  if (infraRojo == LOW) {
    lcd.setCursor(1, 1);
    lcd.print("Detectado           ");
    abajo();
    arriba();
    unsigned long tiempoInicial = millis();
    while (millis() - tiempoInicial < 3000) {  // Espera 3 segundos (3000 ms)
      myStepper.step(stepsPerRevolution);
    }
    lcd.clear();
  } else {
    myStepper.step(stepsPerRevolution);
    // Si no detecta un obstáculo (estado alto)
    digitalWrite(13, LOW);  // Apaga el LED incorporado en el pin 13
    lcd.setCursor(1, 1);
    lcd.print("No detectado");
  }


  if (distance_sharp <= 40) {
    Serial.println("ARRAY!!!");
    Serial.println(lista_humedad[0]);
    Serial.println(lista_humedad[1]);
    regadera();
  }
}

void arriba() {
  for (int ang = 170; ang >= 170; ang--) {
    servo1.write(ang);
    delay(20);
  }
}

void abajo() {
  for (int ang = 170; ang >= 100; ang--) {
    servo1.write(ang);
    delay(100);
  }
  unsigned long tiempoInicial = millis();
  while (millis() - tiempoInicial < 5000) {  // Espera 3 segundos (3000 ms)
    valor_humedad = map(analogRead(sensor), 0, 1023, 100, 0);
    //se hace un mapeo de la lectura del sensor a porcentual
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Humedad: ");
    lcd.print(valor_humedad);
    lcd.print("%");
    delay(100);
  }
  lista_humedad[contador % 2] = valor_humedad;
}

void regadera() {
  if (lista_humedad[0] <= 50 && lista_humedad[0] >= 1) {
    myStepper.step(stepsPerRevolution);
    lcd.setCursor(1, 1);
    lcd.print("Riego             ");
    unsigned long tiempoInicial = millis();
    while (millis() - tiempoInicial < 3000) {  // Espera 3 segundos (3000 ms)
      digitalWrite(motoBomba, LOW);
    }
    while (millis() - tiempoInicial < 3000) {  // Espera 3 segundos (3000 ms)
      digitalWrite(motoBomba, HIGH);
      myStepper.step(stepsPerRevolution);
    }
  }
  // Elimina el primer dato y mueve el segundo dato a la primera posición
  lista_humedad[0] = lista_humedad[1];
  lista_humedad[1] = 0;  // O cualquier valor predeterminado, según tu lógica
}