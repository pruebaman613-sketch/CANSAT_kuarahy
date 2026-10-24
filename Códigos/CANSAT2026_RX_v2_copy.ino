/*
  ============================================================
  CANSAT - ESTACION TERRENA RX + OLED
  HELTEC WiFi LoRa 32 V3
  ============================================================

  RECIBE POR LORA:

      NUMERO_PAQUETE,TEMPERATURA,PRESION,ALTITUD

  EJEMPLO:

      15,26.4,1010.45,15.0


  MUESTRA EN LA PANTALLA OLED:

      PROA & KUARAHY RX
      Temp: 26.4 C
      Pres: 1010.45 hPa
      Alt:  15.0 m
      PKT:  15


  ============================================================
*/


// ============================================================
// LIBRERIAS
// ============================================================

#include <RadioLib.h>
#include <Wire.h>
#include "HT_SSD1306Wire.h"


// ============================================================
// PINES OLED HELTEC V3
// ============================================================

#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21

#define VEXT 36


// ============================================================
// CREACION DEL OBJETO OLED
// ============================================================

SSD1306Wire oled(
  0x3C,
  500000,
  OLED_SDA,
  OLED_SCL,
  GEOMETRY_128_64,
  OLED_RST
);


// ============================================================
// PINES LORA HELTEC V3
// ============================================================

#define HELTEC_NSS   8
#define HELTEC_DIO1  14
#define HELTEC_RST   12
#define HELTEC_BUSY  13


// LED INTEGRADO
#define LED 35


// ============================================================
// CREACION DEL OBJETO LORA
// ============================================================

SX1262 radio = new Module(
  HELTEC_NSS,
  HELTEC_DIO1,
  HELTEC_RST,
  HELTEC_BUSY
);


// ============================================================
// CONFIGURACION SERIAL
// ============================================================

#define SERIAL_BAUDRATE 115200


// ============================================================
// CONFIGURACION LORA
//
// IMPORTANTE:
// Debe ser exactamente igual al transmisor.
// ============================================================

#define LORA_FREQUENCY     915.0
#define LORA_BANDWIDTH     125.0
#define LORA_SPREAD_FACTOR 7
#define LORA_CODING_RATE   5
#define LORA_SYNC_WORD     0xF3
#define LORA_POWER         17


// ============================================================
// VARIABLES PARA LOS DATOS RECIBIDOS
// ============================================================

unsigned int pktNumber = 0;

float temperatura = 0.0;
float presion = 0.0;
float altitud = 0.0;

float rssi = 0.0;


// ============================================================
// FUNCION: MOSTRAR DATOS EN OLED
// ============================================================

void mostrarOLED() {

  // Limpiamos la pantalla antes de escribir nuevos datos.
  oled.clear();


  // ----------------------------------------------------------
  // TITULO
  // ----------------------------------------------------------

  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  oled.setFont(ArialMT_Plain_10);

  oled.drawString(
    64,
    0,
    "PROA & KUARAHY RX"
  );


  // ----------------------------------------------------------
  // DATOS
  // ----------------------------------------------------------

  oled.setTextAlignment(TEXT_ALIGN_LEFT);


  // TEMPERATURA
  oled.drawString(
    0,
    14,
    "Temp:"
  );

  oled.drawString(
    45,
    14,
    String(temperatura, 1) + " C"
  );


  // PRESION
  oled.drawString(
    0,
    26,
    "Pres:"
  );

  oled.drawString(
    45,
    26,
    String(presion, 2) + " hPa"
  );


  // ALTITUD
  oled.drawString(
    0,
    38,
    "Alt:"
  );

  oled.drawString(
    45,
    38,
    String(altitud, 1) + " m"
  );


  // PAQUETE
  oled.drawString(
    0,
    50,
    "PKT: " + String(pktNumber)
  );


  // RSSI
  oled.setTextAlignment(TEXT_ALIGN_RIGHT);

  oled.drawString(
    128,
    50,
    String(rssi, 0) + " dBm"
  );


  // ----------------------------------------------------------
  // MOSTRAR EN PANTALLA
  // ----------------------------------------------------------

  oled.display();
}


// ============================================================
// FUNCION: MOSTRAR PANTALLA DE ESPERA
// ============================================================

void mostrarEsperandoOLED() {

  oled.clear();

  oled.setTextAlignment(TEXT_ALIGN_CENTER);

  oled.setFont(ArialMT_Plain_10);

  oled.drawString(
    64,
    5,
    "PROA & KUARAHY RX"
  );


  oled.setFont(ArialMT_Plain_16);

  oled.drawString(
    64,
    25,
    "ESPERANDO"
  );

  oled.drawString(
    64,
    45,
    "PAQUETE..."
  );


  oled.display();
}


// ============================================================
// FUNCION: MOSTRAR ERROR DE FORMATO
// ============================================================

void mostrarErrorFormatoOLED() {

  oled.clear();

  oled.setTextAlignment(TEXT_ALIGN_CENTER);

  oled.setFont(ArialMT_Plain_10);

  oled.drawString(
    64,
    10,
    "PAQUETE RECIBIDO"
  );

  oled.drawString(
    64,
    30,
    "FORMATO"
  );

  oled.drawString(
    64,
    45,
    "INCORRECTO"
  );


  oled.display();
}


// ============================================================
// SETUP
// ============================================================

void setup() {


  // ==========================================================
  // SERIAL
  // ==========================================================

  Serial.begin(SERIAL_BAUDRATE);

  delay(1000);


  Serial.println();
  Serial.println("================================");
  Serial.println(" CANSAT RX - HELTEC V3");
  Serial.println(" LORA + OLED");
  Serial.println("================================");


  // ==========================================================
  // ALIMENTACION OLED
  // ==========================================================

  pinMode(VEXT, OUTPUT);

  // En la Heltec V3, LOW enciende la alimentación del OLED.
  digitalWrite(VEXT, LOW);

  delay(100);


  // ==========================================================
  // INICIAR OLED
  // ==========================================================

  Serial.println("Inicializando OLED...");

  oled.init();

  delay(100);

  mostrarEsperandoOLED();

  Serial.println("OLED OK!");


  // ==========================================================
  // LED
  // ==========================================================

  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);


  // ==========================================================
  // INICIAR LORA
  // ==========================================================

  Serial.println("Inicializando LoRa...");


  int state = radio.begin(
    LORA_FREQUENCY,
    LORA_BANDWIDTH,
    LORA_SPREAD_FACTOR,
    LORA_CODING_RATE,
    LORA_SYNC_WORD,
    LORA_POWER
  );


  // ==========================================================
  // COMPROBAR SI LORA FUNCIONA
  // ==========================================================

  if (state == RADIOLIB_ERR_NONE) {

    Serial.println("LoRa RadioLib OK!");

  } else {

    Serial.print("ERROR LoRa: ");
    Serial.println(state);


    // Mostrar error en OLED.

    oled.clear();

    oled.setTextAlignment(TEXT_ALIGN_CENTER);

    oled.setFont(ArialMT_Plain_10);

    oled.drawString(
      64,
      20,
      "ERROR LORA"
    );

    oled.drawString(
      64,
      40,
      String(state)
    );

    oled.display();


    // Detener el programa si LoRa no inicia.

    while (true) {
      delay(1000);
    }
  }


  // ==========================================================
  // RX LISTO
  // ==========================================================

  Serial.println("RX listo.");
  Serial.println("Esperando paquetes...");


  // ==========================================================
  // INICIAR RECEPCION
  // ==========================================================

  radio.startReceive();
}


// ============================================================
// LOOP
// ============================================================

void loop() {


  // Variable donde se guardará el paquete recibido.

  String receivedData;


  // ==========================================================
  // RECIBIR PAQUETE LORA
  // ==========================================================

  int state = radio.receive(receivedData);


  // ==========================================================
  // PAQUETE RECIBIDO CORRECTAMENTE
  // ==========================================================

  if (state == RADIOLIB_ERR_NONE) {


    // Encendemos LED mientras procesamos el paquete.

    digitalWrite(LED, HIGH);


    Serial.println();
    Serial.println("--------------------------------");
    Serial.println("PAQUETE RECIBIDO");
    Serial.println("--------------------------------");


    Serial.print("Datos recibidos: ");
    Serial.println(receivedData);


    // ========================================================
    // BUSCAR LAS COMAS
    //
    // Formato esperado:
    //
    // PKT,TEMP,PRESION,ALTITUD
    //
    // Ejemplo:
    //
    // 15,26.4,1010.45,15.0
    // ========================================================

    int p1 = receivedData.indexOf(',');

    int p2 = receivedData.indexOf(',', p1 + 1);

    int p3 = receivedData.indexOf(',', p2 + 1);


    // ========================================================
    // COMPROBAR FORMATO
    // ========================================================

    if (
      p1 > 0 &&
      p2 > p1 &&
      p3 > p2
    ) {


      // ======================================================
      // NUMERO DE PAQUETE
      // ======================================================

      pktNumber = receivedData
                    .substring(0, p1)
                    .toInt();


      // ======================================================
      // TEMPERATURA
      // ======================================================

      temperatura = receivedData
                      .substring(p1 + 1, p2)
                      .toFloat();


      // ======================================================
      // PRESION
      // ======================================================

      presion = receivedData
                  .substring(p2 + 1, p3)
                  .toFloat();


      // ======================================================
      // ALTITUD
      // ======================================================

      altitud = receivedData
                  .substring(p3 + 1)
                  .toFloat();


      // ======================================================
      // RSSI
      // ======================================================

      rssi = radio.getRSSI();


      // ======================================================
      // MOSTRAR EN SERIAL
      // ======================================================

      Serial.println();

      Serial.print("Paquete: ");
      Serial.println(pktNumber);


      Serial.print("Temperatura: ");
      Serial.print(temperatura, 1);
      Serial.println(" C");


      Serial.print("Presion: ");
      Serial.print(presion, 2);
      Serial.println(" hPa");


      Serial.print("Altitud: ");
      Serial.print(altitud, 1);
      Serial.println(" m");


      Serial.print("RSSI: ");
      Serial.print(rssi);
      Serial.println(" dBm");


      // ======================================================
      // MOSTRAR DATOS EN OLED
      // ======================================================

      mostrarOLED();


      Serial.println("Datos mostrados en OLED.");


    } else {


      // ======================================================
      // ERROR EN EL FORMATO
      // ======================================================

      Serial.println("ERROR: formato de paquete incorrecto.");

      mostrarErrorFormatoOLED();
    }


    // Apagamos LED.

    digitalWrite(LED, LOW);


    // Volvemos a poner LoRa en modo recepción.

    radio.startReceive();
  }


  // ==========================================================
  // ERROR DE RECEPCION
  // ==========================================================

  else {

    /*
      Algunos estados pueden ocurrir simplemente porque todavía
      no llegó un paquete.

      No mostramos esto constantemente en el OLED porque
      produciría parpadeos.
    */

    radio.startReceive();
  }
}
