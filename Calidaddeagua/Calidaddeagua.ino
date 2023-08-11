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
 * Semáforo de calidad
 *
 * LED (Verde)               |   Microcontrolador ESP32      |   LED (Amarillo)           |   Microcontrolador ESP32      |   LED (Rojo)              |   Microcontrolador ESP32
 * ------------------------- | ----------------------------- | -------------------------- | ----------------------------- | ------------------------  | -----------------------
 * Ánodo (+)                 |   Pin digital (Ej. GPIO2)     |   Ánodo (+)                |   Pin digital (Ej. GPIO3)     |   Ánodo (+)               |   Pin digital (Ej. GPIO4)
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

// Declaración de todas nuestras variables
float lectura;              // Almacena la lectura del sensor de conductividad
int pin = 36;               // Número de pin al que está conectado el sensor de conductividad

float vOut = 0;             // Voltaje de salida del sensor
float vIn = 3.3;            // Voltaje de entrada
float R1 = 1000;            // Valor de la resistencia R1
float R2 = 0;               // Valor de la resistencia R2
float buffer = 0;           // Variable de cálculo intermedia
float TDS;                  // Almacena el valor de Total Dissolved Solids (TDS)

float R = 0;                // Valor de la resistencia medida por el sensor
float r = 0;                // Valor de la resistividad
float L = 0.05;             // Distancia entre los cables en metros
double A = 0.000150;        // Área de la sección transversal del cable en m^2

float C = 0;                // Valor de la conductividad en S/m
float Cm = 0;               // Valor de la conductividad en mS/cm

const char* ssid = "SSID";      // Nombre de la red WiFi
const char* password = "PASSWORD"; // Contraseña de la red WiFi

const long channelId = CHANNEL_ID; // ID del canal en ThingSpeak
const char* api_key = "API_KEY";   // API Key de ThingSpeak

WiFiClient client;          // Objeto para la conexión WiFi
Twilio *twilio;             // Puntero a un objeto Twilio para enviar mensajes SMS

unsigned long lastThingSpeakUpdate = 0;    // Tiempo de la última actualización de ThingSpeak
unsigned long lastTwilioUpdate = 0;        // Tiempo del último envío de mensaje Twilio
const unsigned long updateThingSpeakInterval = 20000; // Intervalo de actualización de ThingSpeak (20 segundos)
const unsigned long updateTwilioInterval = 60000;     // Intervalo de envío de mensaje Twilio (60 segundos)

// Definición de pines para los LEDs del semáforo
const int pinLedVerde = 2;
const int pinLedAmarillo = 18; // Pin 18 en lugar del pin 3 para el LED amarillo
const int pinLedRojo = 4;

const int oneWireBus = 5;     // Pin de datos del sensor DS18B20
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);

  pinMode(pin, INPUT);
  pinMode(pinLedVerde, OUTPUT);
  pinMode(pinLedAmarillo, OUTPUT);
  pinMode(pinLedRojo, OUTPUT);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  twilio = new Twilio("Account SID", "Auth Token");

  sensors.begin();
}

void loop() {
  unsigned long currentTime = millis();

  // Actualizar ThingSpeak si ha pasado el intervalo
  if (currentTime - lastThingSpeakUpdate >= updateThingSpeakInterval) {
    lastThingSpeakUpdate = currentTime;
    actualizarThingSpeak();
  }

  // Enviar mensaje Twilio si ha pasado el intervalo
  if (currentTime - lastTwilioUpdate >= updateTwilioInterval) {
    lastTwilioUpdate = currentTime;
    enviarMensajeTwilio(TDS);
  }

  // Actualizar el estado de los LEDs en función de la calidad del agua
  actualizarLeds(TDS);
}

void actualizarThingSpeak() {
  // Realizar mediciones y cálculos
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

  int response = ThingSpeak.writeFields(channelId, api_key);

  // Verificar el estado de la respuesta de ThingSpeak
  if (response == 200) {
    Serial.println("Data sent to ThingSpeak successfully!");
  } else {
    Serial.println("Error sending data to ThingSpeak. HTTP error code " + String(response));
  }
}

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

  // Enviar mensaje de texto mediante Twilio
  String responseMsg;
  bool successMsg = twilio->send_message("Numero del Receptor", "Numero de Twilio ", mensaje, responseMsg);
  if (successMsg) {
    Serial.println("Sent message successfully!");
  } else {
    Serial.println(responseMsg);
  }
}

void actualizarLeds(float valorTDS) {
  // Apagar todos los LEDs
  digitalWrite(pinLedVerde, LOW);
  digitalWrite(pinLedAmarillo, LOW);
  digitalWrite(pinLedRojo, LOW);

  // Encender el LED correspondiente según la calidad del agua
  if (valorTDS < 300) {
    digitalWrite(pinLedVerde, HIGH);    // LED verde: Calidad de agua excelente
  } else if (valorTDS >= 300 && valorTDS < 900) {
    digitalWrite(pinLedAmarillo, HIGH); // LED amarillo: Calidad de agua regular
  } else {
    digitalWrite(pinLedRojo, HIGH);     // LED rojo: Calidad de agua pobre o inaceptable
  }
}
