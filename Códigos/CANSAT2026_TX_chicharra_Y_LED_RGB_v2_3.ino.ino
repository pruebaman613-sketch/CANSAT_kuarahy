/*
  ============================================================
  CANSAT 2026 - SISTEMA DE TELEMETRÍA
  HELTEC WiFi LoRa 32 V3 + BMP280
  ============================================================

  FUNCIONES PRINCIPALES:
    - Medición de temperatura
    - Medición de presión
    - Cálculo de altitud absoluta
    - Cálculo de altura relativa al punto de lanzamiento
    - Transmisión mediante LoRa
    - Monitor Serie para diagnóstico

  ALTITUD:
    La primera lectura del BMP280 se utiliza como referencia.

    alturaRelativa = altitudActual - altitudInicial

    De esta manera, el punto de lanzamiento se considera:
    
                         0.00 m

    Ejemplo:
      Altitud inicial = 941.50 m
      Altitud actual  = 971.50 m

      Altura relativa = 971.50 - 941.50
                      = 30.00 m

  IMPORTANTE:
    1013.25 hPa es solamente la presión de referencia utilizada
    por readAltitude() para obtener una estimación de altitud.
    No se utiliza una altitud fija del lugar de lanzamiento.
  ============================================================
*/


// ============================================================
// LIBRERÍAS
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_BMP280.h>


// ============================================================
// CONFIGURACIÓN GENERAL
// ============================================================

constexpr uint32_t SERIAL_BAUDRATE  = 115200;
constexpr uint32_t TX_INTERVAL_MS   = 1000;


// ============================================================
// CONFIGURACIÓN BMP280
// ============================================================

constexpr uint8_t BMP_SDA = 41;
constexpr uint8_t BMP_SCL = 42;

constexpr uint8_t BMP_ADDRESS_1 = 0x76;
constexpr uint8_t BMP_ADDRESS_2 = 0x77;


// Presión estándar utilizada por readAltitude().
// NO representa la altitud del lugar de lanzamiento.

constexpr float SEA_LEVEL_PRESSURE_HPA = 1013.25f;


// ============================================================
// CONFIGURACIÓN LORA
// ============================================================

constexpr uint8_t LORA_NSS  = 8;
constexpr uint8_t LORA_DIO1 = 14;
constexpr uint8_t LORA_RST  = 12;
constexpr uint8_t LORA_BUSY = 13;

constexpr float LORA_FREQUENCY     = 915.0;
constexpr float LORA_BANDWIDTH     = 125.0;
constexpr uint8_t LORA_SPREAD_FACTOR = 7;
constexpr uint8_t LORA_CODING_RATE  = 5;
constexpr uint8_t LORA_SYNC_WORD    = 0xF3;
constexpr int8_t LORA_POWER         = 17;


// ============================================================
// LED
// ============================================================

// LED RGB externa, según el esquema (ánodo/cátodo común conectado a GND).
constexpr uint8_t LED_RED_PIN   = 2;
constexpr uint8_t LED_GREEN_PIN = 3;
constexpr uint8_t LED_BLUE_PIN  = 5;
constexpr uint32_t TX_RETRY_DELAY_MS = 250;

// Chicharra de recuperación: pita al descender a 5 m o menos.
constexpr uint8_t BUZZER_PIN = 4;
constexpr float BUZZER_ALTITUDE_M = 5.0f;
constexpr float BUZZER_ARM_ALTITUDE_M = 10.0f;
constexpr uint32_t BUZZER_PULSE_MS = 150;


// ============================================================
// OBJETOS DE HARDWARE
// ============================================================

// Bus I2C utilizado por el BMP280.
TwoWire bmpI2C = TwoWire(1);


// Sensor BMP280.
Adafruit_BMP280 bmp(&bmpI2C);


// Radio SX1262.
SX1262 radio = new Module(
  LORA_NSS,
  LORA_DIO1,
  LORA_RST,
  LORA_BUSY
);


// ============================================================
// ESTRUCTURA DE DATOS
// ============================================================
//
// Agrupar los datos en una estructura hace que el programa
// sea más fácil de mantener y ampliar.
//

struct TelemetryData {

  uint32_t packetNumber;

  float temperature;
  float pressure;

  float absoluteAltitude;
  float relativeAltitude;
};


// Instancia principal de telemetría.

TelemetryData telemetry = {
  0,
  0.0f,
  0.0f,
  0.0f,
  0.0f
};


// ============================================================
// VARIABLES DEL SISTEMA
// ============================================================

// Altitud registrada al iniciar el sistema.
//
// Esta variable representa el "piso" o punto 0.

float initialAltitude = 0.0f;


// Indica si ya se estableció correctamente
// la referencia inicial.

bool altitudeReferenceReady = false;


// Estado de los dispositivos.

bool bmpReady  = false;
bool loraReady = false;

// Se arma al superar 10 m y pita al volver a 5 m o menos.
bool buzzerArmed = false;
bool buzzerNearGround = false;


// ============================================================
// DECLARACIÓN DE FUNCIONES
// ============================================================

void initializeSerial();

void initializeLED();

void setLEDColor(bool redOn, bool greenOn, bool blueOn);

bool initializeBMP280();

bool initializeLoRa();

void establishAltitudeReference();

bool readSensors();

void calculateRelativeAltitude();

void updateBuzzer();

String createTelemetryPacket();

void printTelemetry();

void transmitTelemetry();

void showError(const String& title, const String& message);

void blinkError();


// ============================================================
// SETUP
// ============================================================

void setup() {

  initializeSerial();

  initializeLED();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);


  // ----------------------------------------------------------
  // BMP280
  // ----------------------------------------------------------

  bmpReady = initializeBMP280();

  if (!bmpReady) {

    showError(
      "ERROR BMP280",
      "Sensor no detectado"
    );

    while (true) {
      blinkError();
    }
  }


  // ----------------------------------------------------------
  // ESTABLECER PUNTO DE REFERENCIA
  // ----------------------------------------------------------

  establishAltitudeReference();


  // ----------------------------------------------------------
  // LORA
  // ----------------------------------------------------------

  loraReady = initializeLoRa();

  if (!loraReady) {

    showError(
      "ERROR LORA",
      "Radio no iniciado"
    );

    while (true) {
      blinkError();
    }
  }


  // ----------------------------------------------------------
  // SISTEMA LISTO
  // ----------------------------------------------------------


  Serial.println();
  Serial.println("==========================================");
  Serial.println("           SISTEMA LISTO");
  Serial.println("==========================================");

  Serial.print("Altitud inicial: ");
  Serial.print(initialAltitude, 2);
  Serial.println(" m");

  Serial.println("Referencia relativa: 0.00 m");
  Serial.println("==========================================");

  delay(2000);
}


// ============================================================
// LOOP PRINCIPAL
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // Incrementar número de paquete
  // ----------------------------------------------------------

  telemetry.packetNumber++;


  // ----------------------------------------------------------
  // Leer sensores
  // ----------------------------------------------------------

  if (!readSensors()) {

    // Lectura fallida del BMP280: LED rojo.
    setLEDColor(true, false, false);

    Serial.println(
      "ERROR: no se pudieron leer los sensores."
    );

    delay(TX_INTERVAL_MS);

    return;
  }


  // Lectura válida del BMP280: LED azul.
  setLEDColor(false, false, true);

  // ----------------------------------------------------------
  // Calcular altura relativa
  // ----------------------------------------------------------

  calculateRelativeAltitude();

  // Pitar al descender cerca del suelo, con pulsos para ahorrar batería.
  updateBuzzer();


  // ----------------------------------------------------------
  // Mostrar datos
  // ----------------------------------------------------------

  printTelemetry();


  // ----------------------------------------------------------
  // Transmitir
  // ----------------------------------------------------------

  transmitTelemetry();


  // ----------------------------------------------------------
  // Esperar siguiente medición
  // ----------------------------------------------------------

  delay(TX_INTERVAL_MS);
}


// ============================================================
// INICIALIZACIÓN SERIAL
// ============================================================

void initializeSerial() {

  Serial.begin(
    SERIAL_BAUDRATE
  );

  delay(1000);


  Serial.println();
  Serial.println("==========================================");
  Serial.println("          CANSAT 2026");
  Serial.println("       SISTEMA TELEMETRÍA");
  Serial.println("==========================================");

  Serial.println(
    "Heltec WiFi LoRa 32 V3"
  );

  Serial.println(
    "BMP280 + SX1262"
  );

  Serial.println("==========================================");
}


// ============================================================
// INICIALIZACIÓN Y CONTROL DEL LED RGB
// ============================================================

void initializeLED() {

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

  // Verde indica que el CanSat está encendido.
  setLEDColor(false, true, false);
}

void setLEDColor(bool redOn, bool greenOn, bool blueOn) {

  // El esquema conecta el terminal común del LED a GND:
  // HIGH enciende cada color.
  digitalWrite(LED_RED_PIN, redOn ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN, greenOn ? HIGH : LOW);
  digitalWrite(LED_BLUE_PIN, blueOn ? HIGH : LOW);
}

// ============================================================
// INICIALIZACIÓN BMP280
// ============================================================

bool initializeBMP280() {

  Serial.println();
  Serial.println(
    "Inicializando BMP280..."
  );


  // Iniciar bus I2C.

  bmpI2C.begin(
    BMP_SDA,
    BMP_SCL
  );


  Serial.print(
    "SDA: GPIO "
  );

  Serial.println(
    BMP_SDA
  );


  Serial.print(
    "SCL: GPIO "
  );

  Serial.println(
    BMP_SCL
  );


  delay(500);


  // ----------------------------------------------------------
  // Intentar dirección 0x76
  // ----------------------------------------------------------

  if (
    bmp.begin(BMP_ADDRESS_1)
  ) {

    Serial.println(
      "BMP280 detectado en 0x76."
    );

  }

  // ----------------------------------------------------------
  // Intentar dirección 0x77
  // ----------------------------------------------------------

  else if (
    bmp.begin(BMP_ADDRESS_2)
  ) {

    Serial.println(
      "BMP280 detectado en 0x77."
    );

  }

  else {

    Serial.println(
      "BMP280 NO detectado."
    );

    return false;
  }


  // ----------------------------------------------------------
  // Configuración del sensor
  // ----------------------------------------------------------

  bmp.setSampling(

    Adafruit_BMP280::MODE_NORMAL,

    Adafruit_BMP280::SAMPLING_X2,

    Adafruit_BMP280::SAMPLING_X16,

    Adafruit_BMP280::FILTER_X16,

    Adafruit_BMP280::STANDBY_MS_125
  );


  Serial.println(
    "BMP280 configurado correctamente."
  );


  return true;
}


// ============================================================
// ESTABLECER REFERENCIA DE ALTITUD
// ============================================================
//
// Esta es una de las partes más importantes.
//
// NO utilizamos:
//     900 m
//     934 m
//     1000 m
//
// La primera lectura se convierte en nuestro punto 0.
//

void establishAltitudeReference() {

  Serial.println();
  Serial.println("==========================================");
  Serial.println("      ESTABLECIENDO PUNTO DE REFERENCIA");
  Serial.println("==========================================");

  Serial.println(
    "Mantener el CanSat quieto..."
  );


  // Esperar a que el sensor se estabilice.

  delay(3000);


  // ----------------------------------------------------------
  // Primera medición
  // ----------------------------------------------------------

  initialAltitude =
    bmp.readAltitude(
      SEA_LEVEL_PRESSURE_HPA
    );


  // ----------------------------------------------------------
  // Validar lectura
  // ----------------------------------------------------------

  if (
    isnan(initialAltitude)
  ) {

    Serial.println(
      "ERROR: altitud inicial inválida."
    );

    altitudeReferenceReady = false;

    return;
  }


  // ----------------------------------------------------------
  // Guardar referencia
  // ----------------------------------------------------------

  altitudeReferenceReady = true;


  Serial.print(
    "ALTITUD INICIAL: "
  );

  Serial.print(
    initialAltitude,
    2
  );

  Serial.println(
    " m"
  );


  Serial.println(
    "PUNTO DE LANZAMIENTO = 0.00 m"
  );

  Serial.println(
    "Referencia establecida correctamente."
  );


  // ----------------------------------------------------------
  // Referencia de altitud establecida
  // ----------------------------------------------------------

  delay(1500);
}


// ============================================================
// INICIALIZACIÓN LORA
// ============================================================

bool initializeLoRa() {

  Serial.println();
  Serial.println(
    "Inicializando LoRa..."
  );


  int state = radio.begin(

    LORA_FREQUENCY,

    LORA_BANDWIDTH,

    LORA_SPREAD_FACTOR,

    LORA_CODING_RATE,

    LORA_SYNC_WORD,

    LORA_POWER
  );


  if (
    state != RADIOLIB_ERR_NONE
  ) {

    Serial.print(
      "Error LoRa: "
    );

    Serial.println(
      state
    );

    return false;
  }


  Serial.println(
    "LoRa iniciado correctamente."
  );


  return true;
}


// ============================================================
// LECTURA DE SENSORES
// ============================================================

bool readSensors() {

  // ----------------------------------------------------------
  // Temperatura
  // ----------------------------------------------------------

  telemetry.temperature =
    bmp.readTemperature();


  // ----------------------------------------------------------
  // Presión
  // ----------------------------------------------------------

  telemetry.pressure =
    bmp.readPressure() / 100.0f;


  // ----------------------------------------------------------
  // Altitud absoluta
  // ----------------------------------------------------------

  telemetry.absoluteAltitude =
    bmp.readAltitude(
      SEA_LEVEL_PRESSURE_HPA
    );


  // ----------------------------------------------------------
  // Validar lecturas
  // ----------------------------------------------------------

  if (
    isnan(telemetry.temperature) ||
    isnan(telemetry.pressure) ||
    isnan(telemetry.absoluteAltitude)
  ) {

    return false;
  }


  return true;
}


// ============================================================
// CALCULAR ALTITUD RELATIVA
// ============================================================

void calculateRelativeAltitude() {

  if (
    !altitudeReferenceReady
  ) {

    telemetry.relativeAltitude = 0.0f;

    return;
  }


  telemetry.relativeAltitude =
    telemetry.absoluteAltitude -
    initialAltitude;


  // ----------------------------------------------------------
  // Evitar pequeños valores negativos por ruido
  // ----------------------------------------------------------

  if (
    telemetry.relativeAltitude < 0.0f &&
    telemetry.relativeAltitude > -2.0f
  ) {

    telemetry.relativeAltitude = 0.0f;
  }
}


// ============================================================
// ============================================================
// CONTROL DE CHICHARRA DE RECUPERACIÓN
// ============================================================

void updateBuzzer() {

  // No sonar antes del lanzamiento: armar luego de superar 10 m.
  if (telemetry.relativeAltitude >= BUZZER_ARM_ALTITUDE_M) {
    buzzerArmed = true;
  }

  // Durante el descenso, pitar al llegar a 5 m o menos.
  buzzerNearGround =
    buzzerArmed &&
    telemetry.relativeAltitude <= BUZZER_ALTITUDE_M;

  if (buzzerNearGround) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(BUZZER_PULSE_MS);
  }

  // Apagada entre pitidos; loop() vuelve a llamar esta función
  // aproximadamente una vez por segundo.
  digitalWrite(BUZZER_PIN, LOW);
}

// CREAR PAQUETE DE TELEMETRÍA
// ============================================================
//
// Formato:
//
// PAQUETE,TEMPERATURA,PRESION,ALTITUD_ABSOLUTA,ALTURA
//
// Ejemplo:
//
// 25,24.62,901.43,972.52,31.20
//

String createTelemetryPacket() {

  String packet;

  packet.reserve(80);


  packet +=
    String(
      telemetry.packetNumber
    );

  packet += ",";


  packet +=
    String(
      telemetry.temperature,
      2
    );

  packet += ",";


  packet +=
    String(
      telemetry.pressure,
      2
    );

  packet += ",";


  packet +=
    String(
      telemetry.absoluteAltitude,
      2
    );

  packet += ",";


  packet +=
    String(
      telemetry.relativeAltitude,
      2
    );


  return packet;
}


// ============================================================
// MOSTRAR TELEMETRÍA EN MONITOR SERIE
// ============================================================

void printTelemetry() {

  Serial.println();
  Serial.println(
    "------------------------------------------"
  );


  Serial.print(
    "PAQUETE: "
  );

  Serial.println(
    telemetry.packetNumber
  );


  Serial.print(
    "TEMPERATURA: "
  );

  Serial.print(
    telemetry.temperature,
    2
  );

  Serial.println(
    " C"
  );


  Serial.print(
    "PRESION: "
  );

  Serial.print(
    telemetry.pressure,
    2
  );

  Serial.println(
    " hPa"
  );


  Serial.print(
    "ALTITUD ABSOLUTA: "
  );

  Serial.print(
    telemetry.absoluteAltitude,
    2
  );

  Serial.println(
    " m"
  );


  Serial.print(
    "ALTURA DESDE INICIO: "
  );

  Serial.print(
    telemetry.relativeAltitude,
    2
  );

  Serial.println(
    " m"
  );


  Serial.print(
    "REFERENCIA INICIAL: "
  );

  Serial.print(
    initialAltitude,
    2
  );

  Serial.println(
    " m"
  );


  Serial.print(
    "PAQUETE: "
  );

  Serial.println(
    createTelemetryPacket()
  );


  Serial.println(
    "------------------------------------------"
  );
}


// ============================================================
// TRANSMITIR TELEMETRÍA
// ============================================================

void transmitTelemetry() {

  if (!loraReady) {
    Serial.println("LoRa no está disponible.");
    return;
  }

  String packet = createTelemetryPacket();
  int attempt = 0;

  // Reintentar el mismo paquete hasta que la radio confirme
  // que terminó la transmisión sin error.
  while (true) {

    attempt++;
    Serial.print("Transmitiendo paquete. Intento ");
    Serial.println(attempt);

    int state = radio.transmit(packet);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(">>> PAQUETE ENVIADO OK");
      return;
    }

    Serial.print(">>> ERROR DE TRANSMISION: ");
    Serial.println(state);
    Serial.println("Reintentando...");
    delay(TX_RETRY_DELAY_MS);
  }
}

// ============================================================
// REGISTRAR ERROR EN SERIAL
// ============================================================

void showError(
  const String& title,
  const String& message
) {

  Serial.println();
  Serial.print("ERROR: ");
  Serial.println(title);
  Serial.println(message);
}

// ============================================================
// PARPADEO DE ERROR
// ============================================================

void blinkError() {

  setLEDColor(true, false, false);
  delay(250);
  setLEDColor(false, false, false);
  delay(250);
}