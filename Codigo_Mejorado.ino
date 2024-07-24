#include <Arduino.h>

// Pines de control para los motores
#define MotFwd1  19  // Motor 1 hacia adelante
#define MotRev1  21  // Motor 1 hacia atrás
#define MotFwd2  25  // Motor 2 hacia adelante
#define MotRev2  26  // Motor 2 hacia atrás

// Pines de los encoders
int encoderPin1A = 4;  // Encoder 1 canal A
int encoderPin1B = 15; // Encoder 1 canal B
int encoderPin2A = 23; // Encoder 2 canal A
int encoderPin2B = 22; // Encoder 2 canal B

// Variables del PID para el motor 1
unsigned long lastTime1 = 0, SampleTime1 = 0;
double Input1 = 0.0, Setpoint1 = 0.0;
double ITerm1 = 0.0, dInput1 = 0.0, lastInput1 = 0.0;
double kp1 = 0.0, ki1 = 0.0, kd1 = 0.0;
double outMin1 = 0.0, outMax1 = 0.0;
double error1 = 0.0;

volatile int lastEncoded1 = 0;
volatile long encoderValue1 = 0;

// Variables del PID para el motor 2
unsigned long lastTime2 = 0, SampleTime2 = 0;
double Input2 = 0.0, Setpoint2 = 0.0;
double ITerm2 = 0.0, dInput2 = 0.0, lastInput2 = 0.0;
double kp2 = 0.0, ki2 = 0.0, kd2 = 0.0;
double outMin2 = 0.0, outMax2 = 0.0;
double error2 = 0.0;

volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;

// Variables adicionales
byte cmd = 0;    // Comando recibido por comunicación serie
byte pwm1 = 0;   // Valor PWM para el motor 1
byte pwm2 = 0;   // Valor PWM para el motor 2
int ledok = 2;   // LED indicador para el motor 1
int ledok2 = 27; // LED indicador para el motor 2

void setup() {
  // Configuración de pines de salida
  pinMode(MotFwd1, OUTPUT);
  pinMode(MotRev1, OUTPUT);
  pinMode(MotFwd2, OUTPUT);
  pinMode(MotRev2, OUTPUT);
  pinMode(ledok, OUTPUT);
  pinMode(ledok2, OUTPUT);

  // Inicializar comunicación serie
  Serial.begin(115200);

  // Configuración de pines de entrada para los encoders con resistencias pull-up
  pinMode(encoderPin1A, INPUT_PULLUP);
  pinMode(encoderPin1B, INPUT_PULLUP);
  pinMode(encoderPin2A, INPUT_PULLUP);
  pinMode(encoderPin2B, INPUT_PULLUP);

  // Configuración de interrupciones para los encoders
  attachInterrupt(digitalPinToInterrupt(encoderPin1A), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin1B), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2B), updateEncoder2, CHANGE);

  // Configuración inicial de los límites y constantes del PID para el motor 1
  outMax1 = 255.0;
  outMin1 = -outMax1;
  SampleTime1 = 50;
  kp1 = 10.0;
  ki1 = 0.00;
  kd1 = 4.00;
  Setpoint1 = (double)encoderValue1;

  // Configuración inicial de los límites y constantes del PID para el motor 2
  outMax2 = 255.0;
  outMin2 = -outMax2;
  SampleTime2 = 50;
  kp2 = 10.0;
  ki2 = 0.00;
  kd2 = 4.00;
  Setpoint2 = (double)encoderValue2;

  imprimir(3); // Imprime la configuración inicial
}

void loop() {
  // Cálculo de salida del PID para cada motor
  double Out1 = Compute();
  double Out2 = Compute2();

  // Control del motor 1
  if (error1 == 0.0) {
    digitalWrite(MotFwd1, LOW);
    digitalWrite(MotRev1, LOW);
    digitalWrite(ledok, HIGH);
  } else {
    pwm1 = abs(Out1);
    if (Out1 > 0.0) {
      digitalWrite(MotRev1, LOW);
      analogWrite(MotFwd1, pwm1);
    } else {
      digitalWrite(MotFwd1, LOW);
      analogWrite(MotRev1, pwm1);
    }
  }

  // Control del motor 2
  if (error2 == 0.0) {
    digitalWrite(MotFwd2, LOW);
    digitalWrite(MotRev2, LOW);
    digitalWrite(ledok2, HIGH);
  } else {
    pwm2 = abs(Out2);
    if (Out2 > 0.0) {
      digitalWrite(MotRev2, LOW);
      analogWrite(MotFwd2, pwm2);
    } else {
      digitalWrite(MotFwd2, LOW);
      analogWrite(MotRev2, pwm2);
    }
  }

  // Recepción de comandos por comunicación serie
  if (Serial.available() > 0) {
    cmd = 0;
    cmd = Serial.read();
    if (cmd > 31) {
      byte flags = 0;
      if (cmd > 'Z') cmd -= 32;
      if (cmd == 'W') { Setpoint1 += 5.0; flags = 2; }
      if (cmd == 'Q') { Setpoint1 -= 5.0; flags = 2; }
      if (cmd == 'S') { Setpoint1 += 400.0; flags = 2; }
      if (cmd == 'A') { Setpoint1 -= 400.0; flags = 2; }
      if (cmd == 'X') { Setpoint1 += 5000.0; flags = 2; }
      if (cmd == 'Z') { Setpoint1 -= 5000.0; flags = 2; }
      if (cmd == '2') { Setpoint1 += 12000.0; flags = 2; }
      if (cmd == '1') { Setpoint1 -= 12000.0; flags = 2; }
      if (cmd == '0') { Setpoint1 = 0.0; flags = 2; }

      // Modificación de las constantes PID
      switch(cmd) {
        case 'P': kp1 = Serial.parseFloat(); flags = 1; break;
        case 'I': ki1 = Serial.parseFloat(); flags = 1; break;
        case 'D': kd1 = Serial.parseFloat(); flags = 1; break;
        case 'T': SampleTime1 = Serial.parseInt(); flags = 1; break;
        case 'G': Setpoint1 = Serial.parseFloat(); flags = 2; break;
        case 'U': Setpoint2 = Serial.parseFloat(); flags = 2; break;
        case 'K': flags = 3; break;
      }
      if (flags == 2) digitalWrite(ledok, LOW);
      
      imprimir(flags);
    }
  }
}

void updateEncoder1() {
  int MSB = digitalRead(encoderPin1A);
  int LSB = digitalRead(encoderPin1B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded1 << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue1--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue1++;
  lastEncoded1 = encoded;
}

void updateEncoder2() {
  int MSB2 = digitalRead(encoderPin2A);
  int LSB2 = digitalRead(encoderPin2B);
  int encoded2 = (MSB2 << 1) | LSB2;
  int sum2 = (lastEncoded2 << 2) | encoded2;
  if (sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011) encoderValue2--;
  if (sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000) encoderValue2++;
  lastEncoded2 = encoded2;
}

// Cálculo del control PID para el motor 1
double Compute() {
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime1);

   if (timeChange >= SampleTime1) {
      Input1 = (double)encoderValue1;
      error1 = Setpoint1 - Input1;
      ITerm1 += (ki1 * error1);

      if (ITerm1 > outMax1) ITerm1 = outMax1;
      else if (ITerm1 < outMin1) ITerm1 = outMin1;

      dInput1 = (Input1 - lastInput1);
      double output = kp1 * error1 + ITerm1 - kd1 * dInput1;

      if (output > outMax1) output = outMax1;
      else if (output < outMin1) output = outMin1;

      lastInput1 = Input1;
      lastTime1 = now;
      return output;
   }
   return 0.0;
}

// Cálculo del control PID para el motor 2
double Compute2() {
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime2);

   if (timeChange >= SampleTime2) {
      Input2 = (double)encoderValue2;
      error2 = Setpoint2 - Input2;
      ITerm2 += (ki2 * error2);

      if (ITerm2 > outMax2) ITerm2 = outMax2;
      else if (ITerm2 < outMin2) ITerm2 = outMin2;

      dInput2 = (Input2 - lastInput2);
      double output = kp2 * error2 + ITerm2 - kd2 * dInput2;

      if (output > outMax2) output = outMax2;
      else if (output < outMin2) output = outMin2;

      lastInput2 = Input2;
      lastTime2 = now;
      return output;
   }
   return 0.0;
}

// Función para imprimir los valores actuales
void imprimir(byte x) {
  if (x == 1) {
    Serial.print("Kp1 ");
    Serial.println(kp1);
    Serial.print("Ki1 ");
    Serial.println(ki1);
    Serial.print("Kd1 ");
    Serial.println(kd1);
    Serial.print("T ");
    Serial.println(SampleTime1);
  }
  if (x == 2) {
    Serial.print("Encoder1 ");
    Serial.println(encoderValue1);
    Serial.print("Setpoint1 ");
    Serial.println(Setpoint1);
    Serial.print("Error1 ");
    Serial.println(error1);
    Serial.print("Encoder2 ");
    Serial.println(encoderValue2);
    Serial.print("Setpoint2 ");
    Serial.println(Setpoint2);
    Serial.print("Error2 ");
    Serial.println(error2);
  }
  if (x == 3) {
    Serial.println("Inicializando...");
  }
}
