#include <Arduino.h>

#define MotFwd1  19
#define MotRev1  21
#define MotFwd2  25
#define MotRev2  26

int encoderPin1A = 4;
int encoderPin1B = 15;
int encoderPin2A = 23;
int encoderPin2B = 22;

// *********************************************** Variables Globales PID *****************************************************************************************
unsigned long lastTime1 = 0,   SampleTime1 = 0;                    // Variables de tiempo discreto.
double        Input1    = 0.0, Setpoint1   = 0.0;                  //     "     de posición del motor y posición a la que queremos llevar el motor (posición designada).
double        ITerm1    = 0.0, dInput1     = 0.0, lastInput1 = 0.0; //     "     de error1 integral, error1 derivativo y posición anterior del motor
double        kp1       = 0.0, ki1         = 0.0, kd1        = 0.0; // Constantes: proprocional, integral y derivativa.
double        outMin1   = 0.0, outMax1     = 0.0;                  // Límites para no sobrepasar la resolución del pwm1.
double        error1    = 0.0;                                    // Desviación o error1 entre la posición real del motor y la posición designada.


volatile int lastEncoded1 = 0; // Last encoded value for encoder 1
volatile long encoderValue1 = 0; // Raw encoder value for encoder 1

byte          cmd      =  0;
byte          pwm1      =  0;
int           ledok    = 2;
// ****************************************************************************************************************************************************************


volatile int lastEncoded2 = 0; // Last encoded value for encoder 2
volatile long encoderValue2 = 0; // Raw encoder value for encoder 2

unsigned long lastTime2 = 0,   SampleTime2 = 0;                    // Variables de tiempo discreto.
double        Input2    = 0.0, Setpoint2   = 0.0;                  //     "     de posición del motor y posición a la que queremos llevar el motor (posición designada).
double        ITerm2    = 0.0, dInput2     = 0.0, lastInput2 = 0.0; //     "     de error1 integral, error1 derivativo y posición anterior del motor
double        kp2       = 0.0, ki2         = 0.0, kd2        = 0.0; // Constantes: proprocional, integral y derivativa.
double        outMin2   = 0.0, outMax2     = 0.0;
double        error2    = 0.0;
byte          pwm2      =  0;
int           ledok2    = 27;  



void setup() {
  pinMode(MotFwd1, OUTPUT); 
  pinMode(MotRev1, OUTPUT);
  pinMode(MotFwd2, OUTPUT); 
  pinMode(MotRev2, OUTPUT);
  pinMode(ledok  , OUTPUT);
  pinMode(ledok2 , OUTPUT);
  Serial.begin(115200); // Initialize serial communication

  // Encoder 1 pins
  pinMode(encoderPin1A, INPUT_PULLUP); 
  pinMode(encoderPin1B, INPUT_PULLUP);
  digitalWrite(encoderPin1A, HIGH); // Turn pullup resistor on
  digitalWrite(encoderPin1B, HIGH); // Turn pullup resistor on
  // Encoder 2 pins
  pinMode(encoderPin2A, INPUT_PULLUP); 
  pinMode(encoderPin2B, INPUT_PULLUP);
  digitalWrite(encoderPin2A, HIGH); // Turn pullup resistor on
  digitalWrite(encoderPin2B, HIGH); // Turn pullup resistor on

  // Attach interrupts for both encoders
  attachInterrupt(digitalPinToInterrupt(encoderPin1A), updateEncoder1, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderPin1B), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), updateEncoder2, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderPin2B), updateEncoder2, CHANGE);
// Acotación máxima y mínima; corresponde a Max.: 0=0V hasta 255=5V (MotFwd1), y Min.: 0=0V hasta -255=5V (MotRev1). El pwm1 se convertirá a la salida en un valor absoluto, nunca negativo.
  outMax1 =  255.0;                      // Límite máximo del controlador PID.
  outMin1 = -outMax1;                     // Límite mínimo del controlador PID.
  
  SampleTime1 = 50;                      // Se le asigna el tiempo de muestreo en milisegundos.
  
  kp1 = 10.0;                             // Constantes PID iniciales. Los valores son los adecuados para un encoder de 334 ppr (con un motor de 12V),
  ki1 = 0.00;                            // pero como el lector de encoder está diseñado como x4, entonces equivale a uno de 1336 ppr. (ppr = pulsos por revolución.)
  kd1 = 4.00;
  
  Setpoint1 = (double)encoderValue1;          // Para evitar que haga cosas extrañas al ponerse en marcha o después de resetear, igualamos los dos valores para que comience estando quieto el motor.


  
  outMax2 =  255.0;                      // Límite máximo del controlador PID.
  outMin2 = -outMax2;  
  SampleTime2 = 50;                      // Se le asigna el tiempo de muestreo en milisegundos.

  kp2 = 10.0;                             // Constantes PID iniciales. Los valores son los adecuados para un encoder de 334 ppr (con un motor de 12V),
  ki2 = 0.00;                            // pero como el lector de encoder está diseñado como x4, entonces equivale a uno de 1336 ppr. (ppr = pulsos por revolución.)
  kd2 = 4.00;
  
  Setpoint2 = (double)encoderValue2;          // Para evitar que haga cosas extrañas al ponerse en marcha o después de resetear, igualamos los dos valores para que comience estando quieto el motor.
  
  
  imprimir(3);

}

void loop() {

  double Out1 = Compute();               // Llama a la función "Compute()" para calcular la desviación y el resultado lo carga en la variable 'Out'.
  double Out2 = Compute2();
  // *********************************************** Control del Motor *************************************************
  if (error1 == 0.0)                     // Cuando está en el punto designado, parar el motor.
  {
    digitalWrite(MotFwd1, LOW);            // Pone a 0 los dos pines del puente en H.
    digitalWrite(MotRev1, LOW);
    digitalWrite(ledok, HIGH);          // Se enciende el led (pin 13) porque ya está en la posición designada.
  }
  else                                  // De no ser igual, significa que el motor ha de girar en un sentido o al contrario; esto lo determina el signo que contiene "Out".
  {
    pwm1 = abs(Out1);                     // Transfiere a la variable pwm1 el valor absoluto de Out.
    // if (pwm1 < 50) pwm1 = 50;          // Línea experimental. Se trata de hacer que el motor tenga una voltaje mínimo para comenzar a girar, aunque esto no es necesario.
    
    if (Out1 > 0.0)                      // Gira el motor en un sentido con el pwm1 correspondiente a su posición.
    {
      digitalWrite(MotRev1, LOW);          // Pone a 0 el segundo pin del puente en H.
      analogWrite(MotFwd1, pwm1);           // Por el primer pin sale la señal pwm1.
    }
    else                                // Gira el motor en sentido contrario con el pwm1 correspondiente a su posición.
    {
      digitalWrite(MotFwd1, LOW);          // Pone a 0 el primer pin del puente en H.
      analogWrite(MotRev1, pwm1);           // Por el segundo pin sale la señal pwm1.
    }
  }


  if (error2 == 0.0)                     // Cuando está en el punto designado, parar el motor.
  {
    digitalWrite(MotFwd2, LOW);            // Pone a 0 los dos pines del puente en H.
    digitalWrite(MotRev2, LOW);
    digitalWrite(ledok2, HIGH);          // Se enciende el led (pin 13) porque ya está en la posición designada.
  }
  else                                  // De no ser igual, significa que el motor ha de girar en un sentido o al contrario; esto lo determina el signo que contiene "Out".
  {
    pwm2 = abs(Out2);                     // Transfiere a la variable pwm1 el valor absoluto de Out.
    // if (pwm1 < 50) pwm1 = 50;          // Línea experimental. Se trata de hacer que el motor tenga una voltaje mínimo para comenzar a girar, aunque esto no es necesario.
    
    if (Out2 > 0.0)                      // Gira el motor en un sentido con el pwm1 correspondiente a su posición.
    {
      digitalWrite(MotRev2, LOW);          // Pone a 0 el segundo pin del puente en H.
      analogWrite(MotFwd2, pwm2);           // Por el primer pin sale la señal pwm1.
    }
    else                                // Gira el motor en sentido contrario con el pwm1 correspondiente a su posición.
    {
      digitalWrite(MotFwd2, LOW);          // Pone a 0 el primer pin del puente en H.
      analogWrite(MotRev2, pwm2);           // Por el segundo pin sale la señal pwm1.
    }
  }
  
  // Recepción de datos para posicionar el motor, o modificar las constantes PID, o el tiempo de muestreo. Admite posiciones relativas y absolutas.
  if (Serial.available() > 0)           // Comprueba si ha recibido algún dato por el terminal serie.
  {
    cmd = 0;                            // Por seguridad "limpiamos" cmd.
    cmd = Serial.read();                // "cmd" guarda el byte recibido.
    if (cmd > 31)
    {
      byte flags = 0;                                      // Borramos la bandera que decide lo que hay que imprimir.
      if (cmd >  'Z') cmd -= 32;                           // Si una letra entra en minúscula la covierte en mayúscula.
      if (cmd == 'W') { Setpoint1 += 5.0;     flags = 2; }  // Si (por ejemplo) es la letra 'W' mueve 5 pasos hacia delante. Estos son movimientos relativos.
      if (cmd == 'Q') { Setpoint1 -= 5.0;     flags = 2; }  // Aquí son esos 5 pasos pero hacia atrás si se pulsa la letra 'Q'.
      if (cmd == 'S') { Setpoint1 += 400.0;   flags = 2; }  // Se repite lo mismo en el resto de las teclas.
      if (cmd == 'A') { Setpoint1 -= 400.0;   flags = 2; }
      if (cmd == 'X') { Setpoint1 += 5000.0;  flags = 2; }
      if (cmd == 'Z') { Setpoint1 -= 5000.0;  flags = 2; }
      if (cmd == '2') { Setpoint1 += 12000.0; flags = 2; }
      if (cmd == '1') { Setpoint1 -= 12000.0; flags = 2; }
      if (cmd == '0') { Setpoint1 = 0.0;      flags = 2; }  // Ir a Inicio.
      
      // Decodificador para modificar las constantes PID.
      switch(cmd)                                          // Si ponemos en el terminal serie, por ejemplo "P2.5 I0.5 D40" y pulsas enter  tomará esos valores y los cargará en kp1, ki1 y kd1.
      {                                                    // También se puede poner individualmente, por ejemplo "P5.5", sólo cambiará el parámetro kp1, los mismo si son de dos en dos.
        case 'P': kp1  = Serial.parseFloat();        flags = 1; break; // Carga las constantes y presenta en el terminal serie los valores de las variables que hayan sido modificadas.
        case 'I': ki1  = Serial.parseFloat();        flags = 1; break;
        case 'D': kd1  = Serial.parseFloat();        flags = 1; break;
        case 'T': SampleTime1 = Serial.parseInt();   flags = 1; break;
        case 'G': Setpoint1   = Serial.parseFloat(); flags = 2; break; // Esta línea permite introducir una posición absoluta. Ex: G23000 (y luego enter) e irá a esa posición.
        case 'U': Setpoint2   = Serial.parseFloat(); flags = 2; break;
        case 'K':                                   flags = 3; break;
      }
      if (flags == 2) digitalWrite(ledok, LOW); // Cuando entra una posición nueva se apaga el led y no se volverá a encender hasta que el motor llegue a la posición que le hayamos designado.
      
      imprimir(flags);
    }
  }
  

}

void updateEncoder1() {
  int MSB = digitalRead(encoderPin1A); // MSB = most significant bit
  int LSB = digitalRead(encoderPin1B); // LSB = least significant bit
  int encoded = (MSB << 1) | LSB; // Converting the 2 pin value to single number
  int sum = (lastEncoded1 << 2) | encoded; // Adding it to the previous encoded value
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue1--;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue1++;
  lastEncoded1 = encoded; // Store this value for next time
}

void updateEncoder2() {
  int MSB2 = digitalRead(encoderPin2A); // MSB = most significant bit
  int LSB2 = digitalRead(encoderPin2B); // LSB = least significant bit
  int encoded2 = (MSB2 << 1) | LSB2; // Converting the 2 pin value to single number
  int sum2 = (lastEncoded2 << 2) | encoded2; // Adding it to the previous encoded value
  if(sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011) encoderValue2--;
  if(sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000) encoderValue2++;
  lastEncoded2 = encoded2; // Store this value for next time
}

// Cálculo PID.
double Compute()
{
   unsigned long now = millis();                  // Toma el número total de milisegundos que hay en ese instante.
   unsigned long timeChange = (now - lastTime1);   // Resta el tiempo actual con el último tiempo que se guardó (esto último se hace al final de esta función).
   
   if(timeChange >= SampleTime1)                   // Si se cumple el tiempo de muestreo entonces calcula la salida.
   {
     Input1  = (double)encoderValue1;                   // Lee el valor del encoder óptico. El valor del contador se incrementa/decrementa a través de las interrupciones externas (pines 2 y 3).
     
     error1  = (Setpoint1 - Input1)  * kp1;           // Calcula el error1 proporcional.
     dInput1 = (Input1 - lastInput1) * kd1;           // Calcula el error1 derivativo.
     
     // Esta línea permite dos cosas: 1) Suaviza la llegada a la meta. 2) El error1 integral se auto-ajusta a las circunstancias del motor.
     if (dInput1 == 0.0)  ITerm1 += (error1 * ki1); else ITerm1 -= (dInput1 * ki1);
     // Acota el error1 integral para eliminar el "efecto windup".
     if (ITerm1 > outMax1) ITerm1 = outMax1; else if (ITerm1 < outMin1) ITerm1 = outMin1;
     
     double Output1 = error1 + ITerm1 - dInput1;      // Suma todos los error1es, es la salida del control PID.
     if (Output1 > outMax1) Output1 = outMax1; else if (Output1 < outMin1) Output1 = outMin1; // Acota la salida para que el pwm1 pueda estar entre outMin1 y outMax1.
     
     lastInput1 = Input1;                           // Se guarda la posición para convertirla en pasado.
     lastTime1  = now;                             // Se guarda el tiempo   para convertirlo en pasado.
     
     return Output1;                               // Devuelve el valor de salida PID.
   }
}

double Compute2()
{
   unsigned long now2 = millis();                  // Toma el número total de milisegundos que hay en ese instante.
   unsigned long timeChange2 = (now2 - lastTime2);   // Resta el tiempo actual con el último tiempo que se guardó (esto último se hace al final de esta función).
   
   if(timeChange2 >= SampleTime2)                   // Si se cumple el tiempo de muestreo entonces calcula la salida.
   {
     Input2  = (double)encoderValue2;                   // Lee el valor del encoder óptico. El valor del contador se incrementa/decrementa a través de las interrupciones externas (pines 2 y 3).
     
     error2  = (Setpoint2 - Input2)  * kp2;           // Calcula el error1 proporcional.
     dInput2 = (Input2 - lastInput2) * kd2;           // Calcula el error1 derivativo.
     
     // Esta línea permite dos cosas: 1) Suaviza la llegada a la meta. 2) El error1 integral se auto-ajusta a las circunstancias del motor.
     if (dInput2 == 0.0)  ITerm2 += (error2 * ki2); else ITerm2 -= (dInput2 * ki2);
     // Acota el error1 integral para eliminar el "efecto windup".
     if (ITerm2 > outMax2) ITerm2 = outMax2; else if (ITerm2 < outMin2) ITerm2 = outMin2;
     
     double Output2 = error2 + ITerm2 - dInput2;      // Suma todos los error1es, es la salida del control PID.
     if (Output2 > outMax2) Output2 = outMax2; else if (Output2 < outMin2) Output2 = outMin2; // Acota la salida para que el pwm1 pueda estar entre outMin1 y outMax1.
     
     lastInput2 = Input2;                           // Se guarda la posición para convertirla en pasado.
     lastTime2  = now2;                             // Se guarda el tiempo   para convertirlo en pasado.
     
     return Output2;                               // Devuelve el valor de salida PID.
   }
}


void imprimir(byte flag) // Imprime en el terminal serie los datos de las contantes PID, tiempo de muestreo y posición. Según las necesidades se muestran algunos datos o todos ellos.
{
  if ((flag == 1) || (flag == 3))
  {
    Serial.print("kp1=");     Serial.print(kp1);
    Serial.print(" ki1=");    Serial.print(ki1);
    Serial.print(" kd1=");    Serial.print(kd1);
    Serial.print(" Time=");  Serial.println(SampleTime1);
  }
  if ((flag == 2) || (flag == 3))
  {
    Serial.print("Posicion1:");
    Serial.println((long)Setpoint1);
    Serial.print("Posicion2:");
    Serial.println((long)Setpoint2);
  }
}
