/* Programa para el Detector Calidad del Agua
 * Autor: [Ezequiel Pinzon Vargas] [Kevin]
 * Fecha: [31/07/2023]
 * 
 * Descripción:
 * Este programa utiliza un Sensor casero para medir la calidad del agua y envía los datos a ThingSpeak para su visualización y monitoreo.
 * Además, utiliza Twilio para enviar mensajes de texto con la clasificación de la calidad del agua según los rangos establecidos por la OMS y sensa la temperatura del agua por medio de un sensor DS18B20.
 * 
 * Hardware requerido:
 * - Microcontrolador compatible con WiFi (ejemplo: ESP32
 * - Resistencia de 1k ohms
 * - Jumpers o alambre de conexiones
 * - Sensor DS18B20 
 * 
 * Conexiones:
 * Sensor de Conductividad    Microcontrolador                                                  Sensor DS18B20   
 * ----------------------     ----------------                                                  -------------- 
 * Pin de señal/salida        Pin analógico del microcontrolador (ejemplo: Pin 36)              DQ
 * VCC                        3.3V                                                              VDD
 * GND                        GND                                                               
 *
 * Semáforo de Temperatura
 *
 * LED (Verde)               |   Microcontrolador ESP32      |   LED (Amarillo)           |   Microcontrolador ESP32      |   LED (Rojo)              |   Microcontrolador ESP32
 * ------------------------- | ----------------------------- | -------------------------- | ----------------------------- | ------------------------  | -----------------------
 * Ánodo (+)                 |   Pin digital (Ej. GPI19)     |   Ánodo (+)                |   Pin digital (Ej. GPI21)     |   Ánodo (+)               |   Pin digital (Ej. GPI22)
 * Cátodo (-)                |   GND                         |   Cátodo (-)               |   GND                         |   Cátodo (-)              |   GND
 *
 * 
 * Dependencias:
 * - Biblioteca WiFi.h para el control de WiFi
 * - Biblioteca ThingSpeak.h para enviar datos a ThingSpeak
 * - Librería twilio.hpp para enviar mensajes SMS mediante Twilio
 * - OneWire.h comunicacion con el sensor DS18B20
 * - DallasTemperature.h para el Sensor DS18B20
 *
 * Notas importantes:
 * - Es necesario modificar las constantes ssid y password con los datos de tu red WiFi
 * - También debes reemplazar los valores de channelId y api_key con los correspondientes a tu cuenta de ThingSpeak
 * - Para utilizar Twilio, se deben proporcionar las credenciales de tu cuenta (SID y AuthToken)
 * - Este programa utiliza técnicas de control de tiempo no bloqueante para realizar las lecturas del sensor y enviar datos a ThingSpeak y Twilio en intervalos definidos por las constantes updateThingSpeakInterval y updateTwilioInterval.
 * - Los datos de conductividad y TDS se envían a ThingSpeak en campos 1 y 2 respectivamente.
 * - Los valores de TDS (Total Dissolved Solids) se calculan a partir de la resistencia medida por el sensor y se clasifican según rangos de calidad del agua establecidos por la OMS.
 * - Los mensajes de Twilio indican la calidad del agua según los rangos establecidos por la OMS.
 * - El código incluye comentarios explicativos para facilitar su comprensión y modificaciones.
 * 
 */

#include <WiFi.h>
#include <ThingSpeak.h>
#include "twilio.hpp"
#include <OneWire.h>
#include <DallasTemperature.h>

// Constantes para los pines
const int PIN_SENSOR = 36;
const int PIN_LED_AZUL = 2;
const int PIN_LED_AMARILLO_TEMP = 18;
const int PIN_LED_ROJO_TEMP = 4;

// Constantes para cálculos de TDS
const float VIN = 3.3;
const float R1 = 1000;
const double A = 0.000150;
const float L = 0.05;

// WiFi y ThingSpeak
const char* SSID = "IZZI-9222";
const char* PASSWORD = "D4AB826D9222";
const long CHANNEL_ID = 2226148;
const char* API_KEY = "L59HVYCPR3CVI2YB";
const unsigned long UPDATE_THINGSPEAK_INTERVAL = 10000;  // 10 segundos

// Twilio
Twilio* twilio;
const unsigned long UPDATE_TWILIO_INTERVAL = 60000;  // 1 minuto

// OneWire y Dallas Temperature
const int ONE_WIRE_BUS = 5;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float lectura = 0;
float TDS = 0;

unsigned long lastThingSpeakUpdate = 0;
unsigned long lastTwilioUpdate = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_SENSOR, INPUT);
  pinMode(PIN_LED_AZUL, OUTPUT);
  pinMode(PIN_LED_AMARILLO_TEMP, OUTPUT);
  pinMode(PIN_LED_ROJO_TEMP, OUTPUT);

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }

  Serial.println("Conectado a WiFi");

  twilio = new Twilio("ACb44b95bda0814294c9f0668499e11a8a", "8d92daf9789df1f273a75ad4dc4f294f");

  sensors.begin();
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastThingSpeakUpdate >= UPDATE_THINGSPEAK_INTERVAL) {
    lastThingSpeakUpdate = currentTime;
    actualizarThingSpeak();
  }

  if (currentTime - lastTwilioUpdate >= UPDATE_TWILIO_INTERVAL) {
    lastTwilioUpdate = currentTime;
    enviarMensajeTwilio(TDS);
  }

  actualizarSemaforoTemperatura();
}

void actualizarThingSpeak() {
  lectura = analogRead(PIN_SENSOR);
  float vOut = lectura * VIN / 1023;
  float buffer = (VIN / vOut) - 1;
  float R2 = R1 * buffer;
  delay(500);
  float r = R2 * A / L;
  float C = 1 / r;
  float Cm = C * 10;
  TDS = Cm * 0.7;

  sensors.requestTemperatures();
  float temperaturaAgua = sensors.getTempCByIndex(0);

  WiFiClient client; // Declara la variable client aquí

  ThingSpeak.begin(client);
  ThingSpeak.setField(1, C);
  ThingSpeak.setField(2, TDS);
  ThingSpeak.setField(3, temperaturaAgua);

  int response = ThingSpeak.writeFields(CHANNEL_ID, API_KEY);

  if (response == 200) {
    Serial.println("Datos enviados a ThingSpeak correctamente!");
  } else {
    Serial.println("Error al enviar datos a ThingSpeak. Código de error HTTP: " + String(response));
  }
}

void enviarMensajeTwilio(float valorTDS) {
  String mensaje;
  if (valorTDS < 300) {
    mensaje = "La calidad del agua es Excelente (TDS: " + String(valorTDS) + ")";
  } else if (valorTDS >= 300 && valorTDS < 600) {
    mensaje = "La calidad del agua es Buena (TDS: " + String(valorTDS) + ")";
  } else if (valorTDS >= 600 && valorTDS < 900) {
    mensaje = "La calidad del agua es Regular (TDS: " + String(valorTDS) + ")";
  } else if (valorTDS >= 900 && valorTDS < 1200) {
    mensaje = "La calidad del agua es Pobre (TDS: " + String(valorTDS) + ")";
  } else {
    mensaje = "La calidad del agua es Inaceptable (TDS: " + String(valorTDS) + ")";
  }

  String responseMsg;
  bool successMsg = twilio->send_message("+527771372675", "+13135135251", mensaje, responseMsg);
  if (successMsg) {
    Serial.println("Mensaje enviado correctamente!");
  } else {
    Serial.println(responseMsg);
  }
}

void actualizarSemaforoTemperatura() {
  sensors.requestTemperatures();
  float temperaturaAgua = sensors.getTempCByIndex(0);

  digitalWrite(PIN_LED_AZUL, LOW);
  digitalWrite(PIN_LED_AMARILLO_TEMP, LOW);
  digitalWrite(PIN_LED_ROJO_TEMP, LOW);

  if (temperaturaAgua < 20) {
    digitalWrite(PIN_LED_AZUL, HIGH);  // LED azul: Temperatura fría (por debajo de la temperatura ambiente)
  } else if (temperaturaAgua >= 20 && temperaturaAgua <= 30) {
    digitalWrite(PIN_LED_AMARILLO_TEMP, HIGH);  // LED amarillo: Temperatura ambiente
  } else {
    digitalWrite(PIN_LED_ROJO_TEMP, HIGH);  // LED rojo: Temperatura por encima de la del ambiente
  }
}
