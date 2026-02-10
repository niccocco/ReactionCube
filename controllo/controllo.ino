#include <Arduino.h>

#include <math.h>

// Librerie per l'hardware
#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_NeoPixel.h>

// --- COSTANTI FISICHE ---
const double mw = 0.1;      // Massa Ruota [Kg]
const double mb = 0.4;      // Massa corpo [Kg]
const double l  = 0.086;    // Distanza CM [m]
const double g  = 9.81;     // Gravità
const double Ib = 0.57e-3;  // Inerzia corpo [Kg*m^2]
const double Iw = 3.34e-3;  // Inerzia ruota [Kg*m^2]
const double Cb = 1.02e-3;  // Attrito Corpo
const double Cw = 0.05e-3;  // Attrito Ruota

typedef struct vec3s16 {
  int16_t x;
  int16_t y;
  int16_t z;
};



// GPIO vari
#define PIN_SDA 12
#define PIN_SCL 13
#define LED_PIN    16

// -------- Parametri di soglia per la verifica --------
const float MAX_GYRO_OFFSET = 5.0; // Gradi al secondo (se l'offset è più alto, c'era movimento)
const float MAX_ACC_VARIANCE = 0.2; // G (tolleranza vibrazioni durante il calcolo)



// Configurazione NeoPixel
#define NUM_LEDS   1
Adafruit_NeoPixel pixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// -------- MOTOR --------
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(11,10,9,8);

// -------- SENSORI --------
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MPU6050 mpu(Wire);



void setup() {
  Serial.begin(115200);

  // 1. NeoPixel Giallo: Avvio
  pixels.begin();
  pixels.setBrightness(30); // Bassa luminosità per non disturbare
  pixels.setPixelColor(0, pixels.Color(255, 200, 0)); 
  pixels.show();

  // Configura I2C sui pin corretti
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.setClock(400000);
  Wire.begin();



  // 1. Controllo connessione (Rosso se fallisce)
  byte status = mpu.begin();
  if (status != 0) {
    mostraColore(255, 0, 0);
    Serial.println(F("Errore MPU6050!"));
    while (1);
  }

  // 2. Fase di attesa (Blu)
  // L'utente ha 3 secondi per posizionare il sensore in piano
  pixel.setPixelColor(0, pixel.Color(0, 0, 255)); // Blu
  pixel.show();
  Serial.println(F("Posiziona il sensore su una superficie piana..."));
  delay(3000);

}

void filtroComplementare (vec3s16 acc, vec3s16 gyro) {
  angleX = alpha * (angleX + gyroX * dt) + (1 - alpha) * accX;
  angleY = alpha * (alpha * (angleY + gyroY * dt) + (1 - alpha) * accY);
}

void mostraColore(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}


void eseguiCalibrazioneProtetta() {
  bool calibrazioneValida = false;

  while (!calibrazioneValida) {
    Serial.println(F("--- Inizio Calibrazione ---"));
    Serial.println(F("Posiziona il sensore immobile..."));
    
    // Attesa con luce Blu
    mostraColore(0, 0, 255);
    delay(3000);

    // Fase di calcolo (Giallo)
    mostraColore(255, 100, 0);
    Serial.println(F("Calcolo in corso..."));
    mpu.calcOffsets(); 

    // VERIFICA DEI CRITERI
    // Otteniamo i valori degli offset appena calcolati
    float gx = abs(mpu.getGyroXoffset());
    float gy = abs(mpu.getGyroYoffset());
    float gz = abs(mpu.getGyroZoffset());

    Serial.print(F("Offset Giroscopio rilevati: "));
    Serial.print(gx); Serial.print(", "); Serial.print(gy); Serial.print(", "); Serial.println(gz);

    // Criterio: se uno degli offset del giroscopio è troppo alto, il sensore si stava muovendo
    if (gx < MAX_GYRO_OFFSET && gy < MAX_GYRO_OFFSET && gz < MAX_GYRO_OFFSET) {
      calibrazioneValida = true;
      Serial.println(F("Calibrazione ACCETTATA!"));
      mostraColore(0, 255, 0); // Verde fisso
      delay(2000);
    } else {
      Serial.println(F("Calibrazione FALLITA (troppo movimento). Riprovo..."));
      // Errore: Rosso lampeggiante prima di riprovare
      for(int i=0; i<3; i++) {
        mostraColore(255, 0, 0); delay(200);
        mostraColore(0, 0, 0);   delay(200);
      }
    }
  }
}



void loop() {
  // put your main code here, to run repeatedly:

}
