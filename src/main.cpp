#include <Encoder.h>
#include <SPI.h>
#include <avr/interrupt.h>

/*
BALL & PLATE V2: Ahora se limita la posición de los motores y hay posibilidad de mirar latencias dentro del código

DESIRED POS: X = 650; Y = 650 aprox
Tanto como la pantalla como los encoders retornan valores enteros, uno long y otro int
*/

volatile bool stepEnableX = false;
volatile bool stepEnableY = false;
volatile bool stepEnableZ = false; 

const float TOLX = 0.5;
const float TOLY = 0.5;
const float TOLZ = 0.5;

const int Max_Vel = 8000; // Velocidad máxima en pasos por segundo (aprox 8000)

// --- PID ---
float Kp = 200;
float Ki = 10;
float Kd = 2;

float errorPrevX = 0, errorPrevY = 0, errorPrevZ = 0;
float integralX = 0, integralY = 0, integralZ = 0;


#define MOTORES_HABILITADOS 1
#define TIMER2ACT 0
#define USINGMACROS 1

// === SPI Estructuras ===
struct Angulos {
    float angX;
    float angY;
    float angZ;
    int16_t apagado;  
    int16_t var_c; 
};

struct Respuesta {
    int16_t touchX;
    int16_t touchY;
    float encoderX;
    float encoderY;
    float encoderZ;
  };
volatile int apagado = 1;
volatile int var_control = 0;

volatile bool datoCompleto = false;
volatile Angulos datosRecibidos;
volatile byte* datos_ptr = (byte*)&datosRecibidos;
volatile size_t indice = 0;

volatile Respuesta respuesta;
volatile byte* respuesta_ptr = (byte*)&respuesta;
volatile size_t respuesta_indice = 0;

const byte START_BYTE = 0xA5;
const byte READY_BYTE = 0xAA;
volatile bool esperandoInicio = true;
volatile bool ready_to_send = false;

//Angulos recibidos de la ESP
volatile float targetX = 0;
volatile float targetY = 0;
volatile float targetZ = 0;


// =================== Pines motores ===================
#define dirPinX     55
#define stepPinX    54
#define enablePinX  38

#define dirPinY     61
#define stepPinY    60
#define enablePinY  56

#define dirPinZ     48
#define stepPinZ    46
#define enablePinZ  62

//MACROS --------------------------
// === MOTOR X ===
#define DIRX_PORT  PORTF
#define DIRX_BIT   PF1    // pin 55
#define STEPX_PORT PORTF
#define STEPX_BIT  PF0    // pin 54
#define ENX_PORT   PORTD
#define ENX_BIT    PD7    // pin 38

// === MOTOR Y ===
#define DIRY_PORT  PORTF
#define DIRY_BIT   PF7    // pin 61
#define STEPY_PORT PORTF
#define STEPY_BIT  PF6    // pin 60
#define ENY_PORT   PORTF
#define ENY_BIT    PF2    // pin 56

// === MOTOR Z ===
#define DIRZ_PORT  PORTL
#define DIRZ_BIT   PL1    // pin 48
#define STEPZ_PORT PORTL
#define STEPZ_BIT  PL3    // pin 46
#define ENZ_PORT   PORTK
#define ENZ_BIT    PK0    // pin 62


// =================== Variables SPI ===================

volatile byte bufferSPI[sizeof(Angulos)];
volatile size_t idxSPI = 0;
volatile bool spi_interrupted = false;


// === Encoders ===
// Motor X: D2 y D3
Encoder encoderX(2, 3);

// Motor Y: D18 y D19
Encoder encoderY(18, 19);

// Motor Z: D20 y D21 (NO mover estos pines)
Encoder encoderZ(20, 21);

// =================== Touchscreen (4 hilos) =================== (a10a11 no, a5a12no) 10c12 5c11 (11,12 lorh)
// #define XP A10 //A10, A11, A12, A5 
// #define XM A12
// #define YP A5 //No importa
// #define YM A11 //No importa
/*
Blanco: A11
Gris:   A12
Morado: A10
Azul:   A5
Pares:
A5,A12
A10,A11
*/

#define XP A5 
#define XM A12
#define YP A10 
#define YM A11 

// =================== Variables ===================
int touchX = 0, touchY = 0;

volatile float posX = 0, posY = 0, posZ = 0;
volatile bool dirX,dirY,dirZ; //true sube, false baja
volatile bool touch = false;


void configurarTimersMotores() {


    //OCR1A = 15999;          // (16 MHz) / (1 prescaler) * tiempo → 16k = 1ms
  //OCR1A = 7999; // 0.5ms
  //OCR1A = 3999; // 0.25ms

  cli(); // Desactivar interrupciones globales mientras se configuran

  // === Timer1 - Motor X ===
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 3500; // 0.375 ms aprox
  TCCR1B |= (1 << WGM12); // Modo CTC
  TCCR1B |= (1 << CS10);  // Prescaler 1
  TIMSK1 |= (1 << OCIE1A); // Habilita interrupción

  // === Timer3 - Motor Y ===
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 3500;
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS30);
  TIMSK3 |= (1 << OCIE3A);

  // === Timer4 - Motor Z ===
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;
  OCR4A = 3500;
  TCCR4B |= (1 << WGM42);
  TCCR4B |= (1 << CS40);
  TIMSK4 |= (1 << OCIE4A);

  sei(); // Reactivar interrupciones globales
}



void setup() {

  Serial.begin(115200);


  pinMode(dirPinX, OUTPUT);
  pinMode(stepPinX, OUTPUT);
  pinMode(enablePinX, OUTPUT);

  pinMode(dirPinY, OUTPUT);
  pinMode(stepPinY, OUTPUT);
  pinMode(enablePinY, OUTPUT);

  pinMode(dirPinZ, OUTPUT);
  pinMode(stepPinZ, OUTPUT);
  pinMode(enablePinZ, OUTPUT);
  

  #if MOTORES_HABILITADOS
  digitalWrite(enablePinX, LOW); // Habilitar motor (LOW normalmente lo habilita)
  digitalWrite(enablePinY, LOW);
  digitalWrite(enablePinZ, LOW);
  #else
  digitalWrite(enablePinX, HIGH); // Habilitar motor (LOW normalmente lo habilita)
  digitalWrite(enablePinY, HIGH);
  digitalWrite(enablePinZ, HIGH);
  #endif

  digitalWrite(dirPinX, HIGH); // o LOW según dirección deseada
  digitalWrite(dirPinY, HIGH);
  digitalWrite(dirPinZ, HIGH);
  
  dirX = true, dirY = true, dirZ = true;


  #if TIMER2ACT
  // Timer2: interrupción cada 5 ms (aprox 200 Hz)
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  // Modo CTC (Clear Timer on Compare Match)
  OCR2A = 124; // (16 MHz / (64 * 200)) - 1 = 124
  
  TCCR2A |= (1 << WGM21);
  // Prescaler 64
  TCCR2B |= (1 << CS22);  // 64 prescaler
  TIMSK2 |= (1 << OCIE2A);  // Habilita interrupción
  #endif


  configurarTimersMotores();

  

  //-------- PROTOCOLO SPI ----------------
  // SPI esclavo
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();
  SPDR = 0x00;



}


float encoderToDegrees(long counts) {
  // Ticks por revolucion = 300 * 4 = 1200
  // grados = encoder * (360/1200) = encoder * 0.3
  return counts * 0.3;
}


// ==================================================== M O T O R E S ===========================================000



// ---------- MOTOR X --------------

#if USINGMACROS

// --- ISR para motor X (Timer1) ---
ISR(TIMER1_COMPA_vect) {
  static bool flancoX = false;

  if (!stepEnableX) {
    // Si no hay movimiento deseado, aseguramos STEP en LOW
    STEPX_PORT &= ~(1 << STEPX_BIT);   // LOW
    flancoX = false;
    return;
  }

  if (flancoX) {
    // Flanco bajo STEP
    STEPX_PORT &= ~(1 << STEPX_BIT);   // LOW
  } else {
    // Antes del flanco alto fijamos la dirección
    if (dirX) DIRX_PORT |= (1 << DIRX_BIT);   // HIGH
    else      DIRX_PORT &= ~(1 << DIRX_BIT);  // LOW

    STEPX_PORT |= (1 << STEPX_BIT);           // HIGH
  }

  flancoX = !flancoX;
}

#else

ISR(TIMER1_COMPA_vect) {
  static bool flancoX = false;

  if (!stepEnableX) {   // si no hay que mover
    digitalWrite(stepPinX, LOW);
    flancoX = false;
    return;
  }

  if (flancoX) {
    digitalWrite(stepPinX, LOW);
  } else {
    // fijar dirección según bandera global
    digitalWrite(dirPinX, dirX ? HIGH : LOW);
    digitalWrite(stepPinX, HIGH);
  }
  flancoX = !flancoX;
}

#endif

// ---------- MOTOR Y --------------

#if USINGMACROS


// --- ISR para motor Y (Timer3) ---
ISR(TIMER3_COMPA_vect) {
  static bool flancoY = false;

  if (!stepEnableY) {
    // Si no hay movimiento deseado, aseguramos STEP en LOW
    STEPY_PORT &= ~(1 << STEPY_BIT);   // LOW
    flancoY = false;
    return;
  }

  if (flancoY) {
    // Flanco bajo STEP
    STEPY_PORT &= ~(1 << STEPY_BIT);   // LOW
  } else {
    // Antes del flanco alto fijamos la dirección
    if (dirY) DIRY_PORT |= (1 << DIRY_BIT);   // HIGH
    else      DIRY_PORT &= ~(1 << DIRY_BIT);  // LOW

    STEPY_PORT |= (1 << STEPY_BIT);           // HIGH
  }

  flancoY = !flancoY;
}

#else

ISR(TIMER3_COMPA_vect) {
  static bool flancoY = false;

  if (!stepEnableY) {
    digitalWrite(stepPinY, LOW);
    flancoY = false;
    return;
  }

  if (flancoY) {
    digitalWrite(stepPinY, LOW);
  } else {
    digitalWrite(dirPinY, dirY ? HIGH : LOW);
    digitalWrite(stepPinY, HIGH);
  }
  flancoY = !flancoY;
}

#endif

// ---------- MOTOR Z --------------

#if USINGMACROS

ISR(TIMER4_COMPA_vect) {
  static bool flancoZ = false;

  if (!stepEnableZ) {
    // Si no hay movimiento deseado, aseguramos STEP en LOW
    STEPZ_PORT &= ~(1 << STEPZ_BIT);   // LOW
    flancoZ = false;
    return;
  }

  if (flancoZ) {
    // Flanco bajo STEP
    STEPZ_PORT &= ~(1 << STEPZ_BIT);   // LOW
  } else {
    // Antes del flanco alto fijamos la dirección
    if (dirZ) DIRZ_PORT |= (1 << DIRZ_BIT);   // HIGH
    else      DIRZ_PORT &= ~(1 << DIRZ_BIT);  // LOW

    STEPZ_PORT |= (1 << STEPZ_BIT);           // HIGH
  }

  flancoZ = !flancoZ;
}



#else
ISR(TIMER4_COMPA_vect) {
  static bool flancoZ = false;
  if (!stepEnableZ) {
    // si no hay movimiento deseado, asegúrate de poner STEP en LOW
    digitalWrite(stepPinZ, LOW);
    flancoZ = false;
    return;
  }
  if (flancoZ) {
    // Bajar STEP
    digitalWrite(stepPinZ, LOW);
  } else {
    // Antes del flanco alto fijamos la dirección
    digitalWrite(dirPinZ, dirZ ? HIGH : LOW);
    digitalWrite(stepPinZ, HIGH);
  }
  flancoZ = !flancoZ;
}

#endif

// =============================== SPI ISR ====================

ISR(SPI_STC_vect) {
  byte c = SPDR;
  if (esperandoInicio) {
    if (c == START_BYTE) {
      esperandoInicio = false;
      indice = 0;
    }
  } else if (indice < sizeof(Angulos)) {
    datos_ptr[indice++] = c;
    if (indice >= sizeof(Angulos)) {
      datoCompleto = true;
      esperandoInicio = true;
      ready_to_send = true;
      respuesta_indice = 0;
      SPDR = READY_BYTE;
      return;
    }
  }

  if (ready_to_send) {
    respuesta_indice++;
    if (respuesta_indice <= sizeof(Respuesta)) {
      SPDR = respuesta_ptr[respuesta_indice - 1];
    } else {
      ready_to_send = false;
      SPDR = 0x00;
    }
  } else {
    SPDR = 0x00;
  }
}



// =========================  T O U C H S C R E E N =================================================

const int N = 4; //Numero de lecturas anteriores
int lastX[N] = {0}, lastY[N] = {0};
int idx = 0;
bool historialInicializado = false;

void detect_touch() {

  // Configurar para detección de contacto
  pinMode(XP, INPUT_PULLUP); // Línea que leeremos
  pinMode(XM, INPUT_PULLUP);
  pinMode(YP, INPUT);        // Línea flotante
  pinMode(YM, OUTPUT);
  digitalWrite(YM, LOW);     // Poner YM a GND

  touch = digitalRead(XP) == LOW;

}

int media(int* arr) {
  long suma = 0;
  for (int i = 0; i < N; i++) suma += arr[i];
  return suma / N;
}

void touchscreen() {
  static uint8_t estado = 0;
  static unsigned long t_espera = 0;
  static int lecturaX = 0, lecturaY = 0;

  switch (estado) {
    case 0:
      detect_touch();

      if (!touch) {
        touchX = 500;
        touchY = 500;
        return;
      }

      // Preparar lectura eje Y
      pinMode(YP, INPUT);
      pinMode(YM, INPUT);
      pinMode(XP, OUTPUT); digitalWrite(XP, HIGH);
      pinMode(XM, OUTPUT); digitalWrite(XM, LOW);
      t_espera = millis();
      estado = 1;
      break;

    case 1:
      if (millis() - t_espera >= 20) {
        lecturaY = analogRead(YP);

        // Preparar lectura eje X
        pinMode(XP, INPUT);
        pinMode(XM, INPUT);
        pinMode(YP, OUTPUT); digitalWrite(YP, HIGH);
        pinMode(YM, OUTPUT); digitalWrite(YM, LOW);
        t_espera = millis();
        estado = 2;
      }
      break;

    case 2:
      if (millis() - t_espera >= 20) {
        lecturaX = analogRead(XP);
        estado = 0;  // Reiniciar para la siguiente lectura

        // -------------------- Filtro de lecturas atípicas -----------------------
        const int UMBRAL = 170; //AUMENTA --> Admite más lecturas 
        int promX = media(lastX);
        int promY = media(lastY);
        bool saltoDesdeCero = (promX < 10 && promY < 10 && lecturaX > 10 && lecturaY > 10);
        bool lecturaValida = 
          (abs(lecturaX - promX) < UMBRAL) && 
          (abs(lecturaY - promY) < UMBRAL);

        if (!historialInicializado) {
          for (int i = 0; i < N; i++) {
            lastX[i] = lecturaX;
            lastY[i] = lecturaY;
          }
          historialInicializado = true;
          touchX = lecturaX;
          touchY = lecturaY;
        } else if (lecturaValida || saltoDesdeCero) {
          touchX = lecturaX;
          touchY = lecturaY;
          lastX[idx] = lecturaX;
          lastY[idx] = lecturaY;
          idx = (idx + 1) % N;
        }
      }
      break;
  }
}


void loop() {

  touchscreen();
  
  //PRIMER FILTRO
  //static float targetFiltroX = 0, targetFiltroY = 0, targetFiltroZ = 0;
  const float alpha = 0.7;
  
  posX = encoderToDegrees(encoderX.read());
  posY = encoderToDegrees(encoderY.read());
  posZ = encoderToDegrees(encoderZ.read());


  if (datoCompleto) {
    
    respuesta.touchX = touchX;
    respuesta.touchY = touchY;
    respuesta.encoderX = posX;
    respuesta.encoderY = posY;
    respuesta.encoderZ = posZ;
    
    datoCompleto = false;
    
    apagado = datosRecibidos.apagado;
    var_control = datosRecibidos.var_c;
    
    targetX = datosRecibidos.angX;
    targetY = datosRecibidos.angY;
    targetZ = datosRecibidos.angZ;
    
  }





  //LOGICA DE APAGADO
  if (apagado) {
    digitalWrite(enablePinX, HIGH);  // deshabilitar motores
    digitalWrite(enablePinY, HIGH);
    digitalWrite(enablePinZ, HIGH);
  } else {
    #if MOTORES_HABILITADOS
    digitalWrite(enablePinX, LOW);   // habilitar motores
    digitalWrite(enablePinY, LOW);
    digitalWrite(enablePinZ, LOW);
    #else
    digitalWrite(enablePinX, HIGH);  // deshabilitar motores
    digitalWrite(enablePinY, HIGH);
    digitalWrite(enablePinZ, HIGH);
    #endif
  }


    // targetX = datosRecibidos.angX;
    // targetY = datosRecibidos.angY;
    // targetZ = datosRecibidos.angZ;


  //PRIMER FILTRO
  targetX = (1 - alpha) * targetX + alpha * targetX;
  targetY = (1 - alpha) * targetY + alpha * targetY;
  targetZ = (1 - alpha) * targetZ + alpha * targetZ;  
  


  static unsigned long last = 0;

  if (millis() - last >= 10) {  // cada 2 ms aprox
    last = millis();


    // ============================= Motor X ============================
    //posX = encoderToDegrees(encoderX.read());
    if (posX < 0) posX = 0;

    float errorX = targetX - posX;

    integralX += errorX * 0.002;  // dt=2ms
    float derivativeX = (errorX - errorPrevX)/0.002;
    float salidaX = Kp*errorX + Ki*integralX + Kd*derivativeX;
    errorPrevX = errorX;

    float velStepsX = fabs(salidaX);
    if (velStepsX < 1) velStepsX = 1;
    if (velStepsX > Max_Vel) velStepsX = Max_Vel;

    dirX = (salidaX > 0);
    stepEnableX = (abs(errorX) > TOLX);

    uint16_t ocrX = F_CPU / (2 * 1 * velStepsX);
    noInterrupts();
    OCR1A = ocrX;   // Timer1 controla motor X
    interrupts();


    // ================================== Motor Y ==============================
    //posY = encoderToDegrees(encoderY.read());
    if (posY < 0) posY = 0;

    float errorY = targetY - posY;

    integralY += errorY * 0.002;
    float derivativeY = (errorY - errorPrevY)/0.002;
    float salidaY = Kp*errorY + Ki*integralY + Kd*derivativeY;
    errorPrevY = errorY;

    float velStepsY = fabs(salidaY);
    if (velStepsY < 1) velStepsY = 1;
    if (velStepsY > Max_Vel) velStepsY = Max_Vel;

    dirY = (salidaY > 0);
    stepEnableY = (abs(errorY) > TOLY);

    uint16_t ocrY = F_CPU / (2 * 1 * velStepsY);
    noInterrupts();
    OCR3A = ocrY;   // Timer3 controla motor Y
    interrupts();


    // ============================ Motor Z =============================

    //posZ = encoderToDegrees(encoderZ.read());
    if (posZ < 0) posZ = 0;

    float errorZ = targetZ - posZ;

    integralZ += errorZ * 0.002;
    float derivativeZ = (errorZ - errorPrevZ)/0.002;
    float salidaZ = Kp*errorZ + Ki*integralZ + Kd*derivativeZ;
    errorPrevZ = errorZ;

    float velStepsZ = fabs(salidaZ);
    if (velStepsZ < 1) velStepsZ = 1;
    if (velStepsZ > Max_Vel) velStepsZ = Max_Vel;
    
    
    dirZ = (salidaZ > 0);
    stepEnableZ = (abs(errorZ) > TOLZ);
    
    uint16_t ocrZ = F_CPU / (2 * 1 * velStepsZ);
    noInterrupts();
    OCR4A = ocrZ;   // Timer4 controla motor Z
    interrupts();



  }


}



