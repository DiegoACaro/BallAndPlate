#include <Encoder.h>
#include <SPI.h>
#include <avr/interrupt.h>

/*
BALL & PLATE V2: Ahora se limita la posición de los motores y hay posibilidad de mirar latencias dentro del código

DESIRED POS: X = 650; Y = 650 aprox
Tanto como la pantalla como los encoders retornan valores enteros, uno long y otro int
*/

#define MOTORES_HABILITADOS 1
#define ACELERACION 0
#define TIMER2ACT 0

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

#if ACELERACION
const uint16_t MIN_OCRnA = 3999;   // velocidad máxima (250 µs)
const uint16_t MAX_OCRnA = 7999;  // velocidad mínima (500 µs)
#endif

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

#define XP A12 
#define XM A5
#define YP A11 
#define YM A10 

// =================== Variables ===================
volatile float posX = 0, posY = 0, posZ = 0;
int touchX = 0, touchY = 0;


volatile bool dirX,dirY,dirZ; //true sube, false baja

volatile bool touch = false;
int detect_touch_filter = 0;

unsigned long lastDebounceTime = 0;

void configurarTimersMotores() {


    //OCR1A = 15999;          // (16 MHz) / (1 prescaler) * tiempo → 16k = 1ms
  //OCR1A = 7999; // 0.5ms
  //OCR1A = 3999; // 0.25ms

  cli(); // Desactivar interrupciones globales mientras se configuran

  // === Timer1 - Motor X ===
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 3999; // 0.375 ms aprox
  TCCR1B |= (1 << WGM12); // Modo CTC
  TCCR1B |= (1 << CS10);  // Prescaler 1
  TIMSK1 |= (1 << OCIE1A); // Habilita interrupción

  // === Timer3 - Motor Y ===
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 3999;
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS30);
  TIMSK3 |= (1 << OCIE3A);

  // === Timer4 - Motor Z ===
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;
  OCR4A = 3999;
  TCCR4B |= (1 << WGM42);
  TCCR4B |= (1 << CS40);
  TIMSK4 |= (1 << OCIE4A);

  sei(); // Reactivar interrupciones globales
}



void setup() {


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


#if ACELERACION

const float DISTANCIA_MIN = 0.5;   // dentro de tolerancia, no moverse
const float DISTANCIA_MAX = 10.0;  // error grande = máxima velocidad

// Mapea distancia al OCRnA (velocidad inversa)
uint16_t calcularOCR(float distancia) {
  if (distancia <= DISTANCIA_MIN) return MAX_OCRnA;  // se mueve muy lento o se detiene
  if (distancia >= DISTANCIA_MAX) return MIN_OCRnA;  // velocidad máxima

  // Escalado inverso lineal entre MAX_OCR1A y MIN_OCR1A
  return map(distancia * 100, DISTANCIA_MIN * 100, DISTANCIA_MAX * 100, MAX_OCRnA, MIN_OCRnA);
}

#endif



// ---------- MOTOR X --------------


ISR(TIMER1_COMPA_vect) {
  static bool flancoX = false;

  posX = encoderToDegrees(encoderX.read());
  if (posX < 0) posX = 0;

  const float TOL = 2;
  const float HYST = 0.5;

  if (var_control == 0) {
    // ------ MODO POSICIÓN ------
    if (abs(posX - targetX) <= TOL) return;

    if (flancoX) {
      digitalWrite(stepPinX, LOW);
    } else {
      if (abs(posX - targetX) > (TOL + HYST)) {
        dirX = (targetX > posX);
      }
      digitalWrite(dirPinX, dirX ? HIGH : LOW);
      digitalWrite(stepPinX, HIGH);
    }

  } else {
    // ------ MODO VELOCIDAD ------
    // En este modo, targetX representa la velocidad deseada (°/s)
    // Puedes usar su signo para dirección
    if ((posX > 90 && targetX > 0) || (posX < 5 && targetX < 0)) return;
    if (targetX == 0) return;

    dirX = (targetX > 0);
    digitalWrite(dirPinX, dirX ? HIGH : LOW);

    if (flancoX) {
      digitalWrite(stepPinX, LOW);
    } else {
      digitalWrite(stepPinX, HIGH);
    }

    // Podrías ajustar la frecuencia del timer (OCR1A) en otro lugar del código
    // para que se relacione con la magnitud de targetX (velocidad deseada)
  }

  flancoX = !flancoX;
}


// ISR(TIMER1_COMPA_vect) {
//   static bool flancoX = false;
//   posX = encoderToDegrees(encoderX.read());
//   if (posX < 0) posX = 0;
  
//   const float TOL = 2;
//   const float HYST = 0.5;
  
//   if (abs(posX - targetX) <= TOL) return;

//   if (flancoX) {
//     digitalWrite(stepPinX, LOW);
//   } else {
//     if (abs(posX - targetX) > (TOL + HYST)) {
//       dirX = (targetX > posX);
//     }
//     digitalWrite(dirPinX, dirX ? HIGH : LOW);
//     digitalWrite(stepPinX, HIGH);

//   }
//   flancoX = !flancoX;
// }



// ---------- MOTOR Y --------------

ISR(TIMER3_COMPA_vect) {
  static bool flancoY = false;

  posY = encoderToDegrees(encoderY.read());
  if (posY < 0) posY = 0;

  const float TOL = 2;
  const float HYST = 0.3;

  if (var_control == 0) {
    // ------ MODO POSICIÓN ------
    if (abs(posY - targetY) <= TOL) return;

    if (flancoY) {
      digitalWrite(stepPinY, LOW);
    } else {
      if (abs(posY - targetY) > (TOL + HYST)) {
        dirY = (targetY > posY);
      }
      digitalWrite(dirPinY, dirY ? HIGH : LOW);
      digitalWrite(stepPinY, HIGH);
    }

  } else {
    // ------ MODO VELOCIDAD ------

    if ((posY > 90 && targetY > 0) || (posY < 5 && targetY < 0)) return;
    if (targetY == 0) return;

    dirY = (targetY > 0);
    digitalWrite(dirPinY, dirY ? HIGH : LOW);

    if (flancoY) {
      digitalWrite(stepPinY, LOW);
    } else {
      digitalWrite(stepPinY, HIGH);
    }
  }

  flancoY = !flancoY;
}


// ISR(TIMER3_COMPA_vect) {
//   static bool flancoY = false;
//   posY = encoderToDegrees(encoderY.read());
//   if (posY < 0) posY = 0;

//   const float TOL = 2;
//   const float HYST = 0.3;

//   if (abs(posY - targetY) <= TOL) return;

//   if (flancoY) {
//     digitalWrite(stepPinY, LOW);
//   } else {
//     if (abs(posY - targetY) > (TOL + HYST)) {
//       dirY = (targetY > posY);
//     }
//     digitalWrite(dirPinY, dirY ? HIGH : LOW);
//     digitalWrite(stepPinY, HIGH);

//   }
//   flancoY = !flancoY;
// }


// ---------- MOTOR Z --------------


ISR(TIMER4_COMPA_vect) {
  static bool flancoZ = false;

  posZ = encoderToDegrees(encoderZ.read());
  if (posZ < 0) posZ = 0;

  const float TOL = 2;
  const float HYST = 0.5;

  if (var_control == 0) {
    // ------ MODO POSICIÓN ------
    if (abs(posZ - targetZ) <= TOL) return;

    if (flancoZ) {
      digitalWrite(stepPinZ, LOW);
    } else {
      if (abs(posZ - targetZ) > (TOL + HYST)) {
        dirZ = (targetZ > posZ);
      }
      digitalWrite(dirPinZ, dirZ ? HIGH : LOW);
      digitalWrite(stepPinZ, HIGH);
    }

  } else {
    // ------ MODO VELOCIDAD ------

    if ((posZ > 90 && targetZ > 0) || (posZ < 5 && targetZ < 0)) return;
    if (targetZ == 0) return;

    dirZ = (targetZ > 0);
    digitalWrite(dirPinZ, dirZ ? HIGH : LOW);

    if (flancoZ) {
      digitalWrite(stepPinZ, LOW);
    } else {
      digitalWrite(stepPinZ, HIGH);
    }
  }

  flancoZ = !flancoZ;
}


// ISR(TIMER4_COMPA_vect) {
//   static bool flancoZ = false;
//   posZ = encoderToDegrees(encoderZ.read());
//   if (posZ < 0) posZ = 0;

//   const float TOL = 2;
//   const float HYST = 0.5;

//   if (abs(posZ - targetZ) <= TOL) return;

//   if (flancoZ) {
//     digitalWrite(stepPinZ, LOW);
//   } else {
//     if (abs(posZ - targetZ) > (TOL + HYST)) {
//       dirZ = (targetZ > posZ);
//     }
//     digitalWrite(dirPinZ, dirZ ? HIGH : LOW);
//     digitalWrite(stepPinZ, HIGH);

//   }
//   flancoZ = !flancoZ;
// }




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
  static float targetFiltroX = 0, targetFiltroY = 0, targetFiltroZ = 0;
  const float alpha = 0.7;


  if (datoCompleto) {

    respuesta.touchX = touchX;
    respuesta.touchY = touchY;
    respuesta.encoderX = (encoderToDegrees(encoderX.read()));
    respuesta.encoderY = (encoderToDegrees(encoderY.read()));
    respuesta.encoderZ = (encoderToDegrees(encoderZ.read()));
    datoCompleto = false;

    apagado = datosRecibidos.apagado;
    var_control = datosRecibidos.var_c;

    //PRIMER FILTRO
    targetFiltroX = (1 - alpha) * targetFiltroX + alpha * datosRecibidos.angX;
    targetFiltroY = (1 - alpha) * targetFiltroY + alpha * datosRecibidos.angY;
    targetFiltroZ = (1 - alpha) * targetFiltroZ + alpha * datosRecibidos.angZ;
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
  targetX = targetFiltroX;
  targetY = targetFiltroY;
  targetZ = targetFiltroZ;

  #if ACELERACION
  if (abs(posX - targetX) > 10) OCR1A = calcularOCR(abs(posX - targetX));
  else OCR1A = 5999;
  if (abs(posY - targetY) > 10) OCR3A = calcularOCR(abs(posY - targetY));
  else OCR3A = 5999;
  if (abs(posZ - targetZ) > 10) OCR4A = calcularOCR(abs(posZ - targetZ));
  else OCR4A = 5999;
  #endif




}



