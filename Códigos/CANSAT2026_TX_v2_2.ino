/*
  TELEMETRÍA DE PRUEBA
  Heltec LoRa 32 V3
  SIN BMP280

  Envía datos simulados de:
  - Temperatura
  - Presión
  - Altitud
*/

#include <RadioLib.h>

// PINES HELTEC LORA 32 V3
#define HELTEC_NSS   8
#define HELTEC_DIO1  14
#define HELTEC_RST   12
#define HELTEC_BUSY  13
#define LED          35
#define CHICHARRA    4

// CONFIGURACIÓN LORA
#define LORA_FREQUENCY     915.0
#define LORA_BANDWIDTH     125.0
#define LORA_SPREAD_FACTOR 7
#define LORA_CODING_RATE   5
#define LORA_SYNC_WORD     0xF3
#define LORA_POWER         17

#define SERIAL_BAUDRATE    115200
#define INTERVAL_TIME_TX   1000

// Umbral para activar la chicharra (metros)
const float ALTITUD_UMBRAL = 105.0;

// RADIO
SX1262 radio = new Module(
  HELTEC_NSS,
  HELTEC_DIO1,
  HELTEC_RST,
  HELTEC_BUSY
);

// VARIABLES
unsigned long numeroPaquete = 0;
float temperatura = 25.0;
float presion = 1013.25;
float altitud = 0.0;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(CHICHARRA, OUTPUT);
  digitalWrite(CHICHARRA, LOW);

  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("     TELEMETRIA LORA - PRUEBA");
  Serial.println("     HELTEC LORA 32 V3");
  Serial.println("     SIN SENSOR");
  Serial.println("======================================");

  Serial.println("Inicializando LoRa...");

  int estado = radio.begin(
    LORA_FREQUENCY,
    LORA_BANDWIDTH,
    LORA_SPREAD_FACTOR,
    LORA_CODING_RATE,
    LORA_SYNC_WORD,
    LORA_POWER
  );

  if (estado == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa iniciado correctamente.");
  } else {
    Serial.print("ERROR LoRa. Codigo: ");
    Serial.println(estado);

    while (true) {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
    }
  }

  Serial.println();
  Serial.println("Sistema listo para transmitir.");
  Serial.println();

  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
}

void loop() {
  numeroPaquete++;

  // Generar datos simulados
  temperatura = 25.0 + (numeroPaquete % 10) * 0.1;
  presion = 1013.25 - (numeroPaquete % 20) * 0.5;
  altitud = 100.0 + numeroPaquete * 0.5;

  // Activar la chicharra al alcanzar el umbral
  if (altitud >= ALTITUD_UMBRAL) {
    digitalWrite(CHICHARRA, HIGH);
  } else {
    digitalWrite(CHICHARRA, LOW);
  }

  String paquete =
    String(numeroPaquete) + "," +
    String(temperatura, 2) + "," +
    String(presion, 2) + "," +
    String(altitud, 2);

  Serial.println("--------------------------------------");

  Serial.print("Paquete: ");
  Serial.println(numeroPaquete);

  Serial.print("Temperatura: ");
  Serial.print(temperatura, 2);
  Serial.println(" C");

  Serial.print("Presion: ");
  Serial.print(presion, 2);
  Serial.println(" hPa");

  Serial.print("Altitud: ");
  Serial.print(altitud, 2);
  Serial.println(" m");

  Serial.print("Enviando: ");
  Serial.println(paquete);

  digitalWrite(LED, HIGH);
  int estado = radio.transmit(paquete);
  digitalWrite(LED, LOW);

  if (estado == RADIOLIB_ERR_NONE) {
    Serial.println(">>> PAQUETE ENVIADO OK");
  } else {
    Serial.print(">>> ERROR DE TRANSMISION: ");
    Serial.println(estado);
  }

  Serial.println("--------------------------------------");
  delay(INTERVAL_TIME_TX);
}
