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

// Declaración de variables globales para el sensor de conductividad
float lectura;
int pin = 36;

float vOut = 0;
float vIn = 3.3;
float R1 = 1000;
float R2 = 0;
float buffer = 0;
float TDS;

float R = 0;
float r = 0;
float L = 0.05;
double A = 0.000150;

float C = 0;
float Cm = 0;

// Configuración de la red WiFi
const char* ssid = "SSID";             // Nombre de la red WiFi
const char* password = "PASSWORD";     // Contraseña de la red WiFi

// Configuración de ThingSpeak
const long channelId = 2226148;        // ID del canal en ThingSpeak
const char* api_key = "WRITE API KEYS"; // Clave API de escritura de ThingSpeak

// Objeto para la conexión WiFi
WiFiClient client;

// Objeto para enviar mensajes Twilio
Twilio *twilio;

// Variables para controlar el intervalo de actualización
unsigned long lastThingSpeakUpdate = 0;
unsigned long lastTwilioUpdate = 0;
const unsigned long updateThingSpeakInterval = 10000; // Intervalo de actualización de ThingSpeak (10 segundos)
const unsigned long updateTwilioInterval = 60000;     // Intervalo de envío de mensaje Twilio (60 segundos)

// Configuración del bus OneWire y el sensor DS18B20
const int oneWireBus = 5;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Configuración de pines para el semáforo de temperatura
const int pinLedAzul = 2;               // LED azul: Temperatura fría (por debajo de la temperatura ambiente)
const int pinLedAmarilloTemp = 18;      // LED amarillo: Temperatura ambiente
const int pinLedRojoTemp = 4;           // LED rojo: Temperatura por encima de la del ambiente

// Función de configuración (se ejecuta al inicio)
void setup() {
  Serial.begin(115200);  // Iniciar la comunicación serial

  pinMode(pin, INPUT);   // Configurar el pin del sensor de conductividad como entrada
  pinMode(pinLedAzul, OUTPUT);           // Configurar el pin del LED azul como salida
  pinMode(pinLedAmarilloTemp, OUTPUT);   // Configurar el pin del LED amarillo como salida
  pinMode(pinLedRojoTemp, OUTPUT);       // Configurar el pin del LED rojo como salida

  // Conexión a la red WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Inicialización del objeto Twilio para enviar mensajes
  twilio = new Twilio("Account SID", "Auth Token");

  // Inicialización del sensor de temperatura DS18B20
  sensors.begin();
}

// Función de bucle principal (se ejecuta repetidamente)
void loop() {
  unsigned long currentTime = millis();

  // Actualizar ThingSpeak si ha pasado el intervalo definido
  if (currentTime - lastThingSpeakUpdate >= updateThingSpeakInterval) {
    lastThingSpeakUpdate = currentTime;
    actualizarThingSpeak();
  }

  // Enviar mensaje Twilio si ha pasado el intervalo definido
  if (currentTime - lastTwilioUpdate >= updateTwilioInterval) {
    lastTwilioUpdate = currentTime;
    enviarMensajeTwilio(TDS);
  }

  // Actualizar el semáforo de temperatura
  actualizarSemaforoTemperatura();
}

// Función para actualizar los campos en ThingSpeak
void actualizarThingSpeak() {
  // Realizar mediciones y cálculos de conductividad y TDS
  lectura = analogRead(pin);
  vOut = lectura * 3.3 / 1023;
  buffer = (vIn / vOut) - 1;
  R2 = R1 * buffer;
  delay(500);
  r = R2 * A / L;
  C = 1 / r;
  Cm = C * 10;
  TDS = Cm * 0.7;

  // Obtener la temperatura del sensor DS18B20
  sensors.requestTemperatures();
  float temperaturaAgua = sensors.getTempCByIndex(0);

  // Iniciar comunicación con ThingSpeak y enviar datos
  ThingSpeak.begin(client);
  ThingSpeak.setField(1, C);
  ThingSpeak.setField(2, TDS);
  ThingSpeak.setField(3, temperaturaAgua);

  // Enviar datos al canal de ThingSpeak y verificar la respuesta
  int response = ThingSpeak.writeFields(channelId, api_key);
  if (response == 200) {
    Serial.println("Data sent to ThingSpeak successfully!");
  } else {
    Serial.println("Error sending data to ThingSpeak. HTTP error code " + String(response));
  }
}

// Función para enviar mensajes Twilio
void enviarMensajeTwilio(float valorTDS) {
  // Crear mensaje de texto en función del valor de TDS
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