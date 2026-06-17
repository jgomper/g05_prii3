#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// --- CONFIGURACIÓN BNO055 ---
#define IMU_ID 55
#define IMU_ADDR 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(IMU_ID, IMU_ADDR, &Wire);

unsigned long temporizador_imu = 0;

// --- PINES MOTORES (Configuración Yahboom) ---
const int M1INA = 2; 
const int M1INB = 4;
const int M1PWM = 9; 
const int M1ENA = 6; 

const int M2INA = 7; 
const int M2INB = 8;
const int M2PWM = 10; 
const int M2ENA = 12; 

// --- PIN VENTOSA ---
const int PIN_VENTOSA = 5;

void setup() {
  Serial.begin(115200); 
  Serial.setTimeout(10); 
  
  // --- INICIALIZAR BNO055 ---
  if (!bno.begin()) {
    while (1) {
      Serial.println("Ooops, no se detectó el BNO055. ¡Revisa los cables!");
      delay(1000);
    }
  }
  delay(1000); // Pausa de estabilización 
  
  // ¡CRÍTICO! Mantener en false para tu placa genérica
  bno.setExtCrystalUse(false);
  
  // --- INICIALIZAR PINES YAHBOOM ---
  pinMode(M1INA, OUTPUT); pinMode(M1INB, OUTPUT); pinMode(M1PWM, OUTPUT); pinMode(M1ENA, OUTPUT); 
  pinMode(M2INA, OUTPUT); pinMode(M2INB, OUTPUT); pinMode(M2PWM, OUTPUT); pinMode(M2ENA, OUTPUT); 
  pinMode(PIN_VENTOSA, OUTPUT);

  digitalWrite(M1ENA, HIGH); 
  digitalWrite(M2ENA, HIGH); 
  digitalWrite(PIN_VENTOSA, LOW);

  Serial.println("Arduino Listo con BNO055 y Motores preparados.");
}

void loop() {
  // 1. Enviar el Yaw a Python cada 20ms (50 Hz)
  if ((millis() - temporizador_imu) > 20) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float yaw = euler.x(); 
    Serial.print("Y:");
    Serial.println(yaw);
    temporizador_imu = millis();
  }

  // 2. Escuchar las órdenes de movimiento SIN BLOQUEOS
  if (Serial.available() > 0) {
    // Leemos toda la línea de golpe hasta el salto de línea (\n)
    String linea = Serial.readStringUntil('\n');
    linea.trim(); // Limpiamos espacios basura o saltos extra

    if (linea.length() > 0) {
      char comando = linea.charAt(0);

      if (comando == 'S') { 
        controlMotorIzquierdo(0);
        controlMotorDerecho(0);
      }
      else if (comando == 'V') digitalWrite(PIN_VENTOSA, HIGH);
      else if (comando == 'v') digitalWrite(PIN_VENTOSA, LOW);
      
      // LÓGICA ROBUSTA ANTI-RETRASOS
      else if (comando == 'M') {
        // Buscamos los espacios para separar los números "M 50 -30"
        int espacio1 = linea.indexOf(' ');
        int espacio2 = linea.indexOf(' ', espacio1 + 1);
        
        if (espacio1 != -1 && espacio2 != -1) {
          int vel_izq = linea.substring(espacio1 + 1, espacio2).toInt();
          int vel_der = linea.substring(espacio2 + 1).toInt();
          
          controlMotorIzquierdo(vel_izq);
          controlMotorDerecho(vel_der);
        }
      }
    }
  }
}

// ==========================================
// FUNCIONES AUXILIARES PARA MOTORES
// ==========================================

void controlMotorDerecho(int velocidad) {
  if (velocidad > 0) { 
    digitalWrite(M1INA, HIGH); 
    digitalWrite(M1INB, LOW);
    analogWrite(M1PWM, velocidad);
  } 
  else if (velocidad < 0) { 
    digitalWrite(M1INA, LOW); 
    digitalWrite(M1INB, HIGH);
    analogWrite(M1PWM, abs(velocidad)); 
  } 
  else { 
    digitalWrite(M1INA, LOW); 
    digitalWrite(M1INB, LOW);
    analogWrite(M1PWM, 0);
  }
}

void controlMotorIzquierdo(int velocidad) {
  if (velocidad > 0) {
    digitalWrite(M2INA, HIGH); 
    digitalWrite(M2INB, LOW);
    analogWrite(M2PWM, velocidad);
  } 
  else if (velocidad < 0) { 
    digitalWrite(M2INA, LOW); 
    digitalWrite(M2INB, HIGH);
    analogWrite(M2PWM, abs(velocidad)); 
  } 
  else { 
    digitalWrite(M2INA, LOW); 
    digitalWrite(M2INB, LOW);
    analogWrite(M2PWM, 0);
  }
}
