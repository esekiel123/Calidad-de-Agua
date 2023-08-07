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
 * GND                        GND                                                               GND
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

// Declaración de todas nuestras variables
float lectura;
int pin = 36;

float vOut = 0; // caída de voltaje entre 2 puntos
float vIn = 3.3;
float R1 = 1000;
float R2 = 0;
float buffer = 0;
float TDS;

float R = 0; // resistencia entre los 2 cables
float r = 0; // resistividad
float L = 0.05; // distancia entre los cables en metros
double A = 0.000150; // área de la sección transversal del cable en m^2

float C = 0; // conductividad en S/m
float Cm = 0; // conductividad en mS/cm

const char* ssid = "SSID";      // Cambiar por el nombre de tu red Wi-Fi
const char* password = "PASSWORD"; // Cambiar por la contraseña de tu red Wi-Fi

const long channelId =  channelId; // Reemplazar con el ID de tu canal en ThingSpeak
const char* api_key = "Read Api Key";   // Cambiar por tu API Key de ThingSpeak

WiFiClient client;
Twilio *twilio;

unsigned long lastThingSpeakUpdate = 0;
unsigned long lastTwilioUpdate = 0;
const unsigned long updateThingSpeakInterval = 20000; // Intervalo de actualización de ThingSpeak (15 segundos)
const unsigned long updateTwilioInterval = 60000;    // Intervalo de actualización de Twilio (60 segundos)

// Configuración para el sensor DS18B20
const int oneWireBus = 5; // Pin de datos del sensor DS18B20 (cambiar al pin que desees)
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);

  pinMode(pin, INPUT);

  // Conexión Wi-Fi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  twilio = new Twilio("Account SID", "Auth Token"); // Reemplazar con tus credenciales de Twilio

  // Inicialización del sensor DS18B20
  sensors.begin();
}

void loop() {
  // Obtener el tiempo actual
  unsigned long currentTime = millis();

  // Solo leer el sensor y actualizar los datos si ha pasado el intervalo para ThingSpeak
  if (currentTime - lastThingSpeakUpdate >= updateThingSpeakInterval) {
    lastThingSpeakUpdate = currentTime;
    actualizarThingSpeak();
  }

  // Solo enviar el mensaje de Twilio si ha pasado el intervalo para Twilio
  if (currentTime - lastTwilioUpdate >= updateTwilioInterval) {
    lastTwilioUpdate = currentTime;
    enviarMensajeTwilio(TDS);
  }
}

// Función para actualizar ThingSpeak
void actualizarThingSpeak() {
  lectura = analogRead(pin);

  vOut = lectura * 3.3 / 1023;
  Serial.println(lectura);
  //  Serial.println(vOut);
  buffer = (vIn / vOut) - 1;
  R2 = R1 * buffer;
  Serial.println(R2);
  delay(500);
  // Convertir el voltaje a resistencia
  // Aplicar la fórmula mencionada anteriormente
  r = R2 * A / L; // R=rL/A
  // Convertir resistividad a conductividad
  C = 1 / r;
  Cm = C * 10;
  // Convertir conductividad en mS/cm a TDS
  TDS = Cm * 0.7;

  // Obtener la temperatura del sensor DS18B20
  sensors.requestTemperatures();
  float temperaturaAgua = sensors.getTempCByIndex(0);

  // Enviar datos a ThingSpeak
  ThingSpeak.begin(client);
  ThingSpeak.setField(1, C);              // Campo 1: Valor de conductividad
  ThingSpeak.setField(2, TDS);            // Campo 2: Valor de TDS
  ThingSpeak.setField(3, temperaturaAgua); // Campo 3: Temperatura del agua

  int response = ThingSpeak.writeFields(channelId, api_key);

  if (response == 200) {
    Serial.println("Data sent to ThingSpeak successfully!");
  } else {
    Serial.println("Error sending data to ThingSpeak. HTTP error code " + String(response));
  }
}

// Función para enviar mensaje de Twilio
void enviarMensajeTwilio(float valorTDS) {
  String mensaje;
  // Comparar el valor de TDS con los rangos establecidos por la OMS
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
  bool successMsg = twilio->send_message("Numero del Receptor", "Numero de Twilio ", mensaje, responseMsg); // Reemplazar con los números de destino y de Twilio
  if (successMsg) {
    Serial.println("Sent message successfully!");
  } else {
    Serial.println(responseMsg);
  }
}
