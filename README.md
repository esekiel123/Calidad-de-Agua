# Programa para el Detector Calidad del Agua

## Descripción

Este programa está diseñado para utilizar un sensor casero para medir la calidad del agua y enviar los datos a ThingSpeak para su visualización y monitoreo. Además, utiliza Twilio para enviar mensajes de texto con la clasificación de la calidad del agua según los rangos establecidos por la OMS.

## Limitantes

Este proyecto tiene como finalidad ser usado como 

## Hardware requerido

- Microcontrolador compatible con WiFi (por ejemplo, ESP32)
- Resistencia de 1k ohmios
- Jumpers o alambre de conexiones

## Software necesario

- [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
- [Arduino IDE](https://www.arduino.cc/en/software)
    - [Gestor de tarjetas ESP32](https://github.com/iotechbugs/esp32-arduino/blob/master/docs/arduino-ide/boards_manager.md)

## Conexiones

Sensor de Conductividad | Microcontrolador
---------------------- | ----------------
Pin de señal/salida | Pin analógico del microcontrolador (por ejemplo, Pin 36)
VCC | 3.3V
GND | GND

## Dependencias

- Biblioteca WiFi.h para el control de WiFi
- Biblioteca ThingSpeak.h para enviar datos a ThingSpeak
- Librería twilio.hpp para enviar mensajes SMS mediante Twilio

## Notas importantes

- Es necesario modificar las constantes `ssid` y `password` con los datos de tu red WiFi.
- También debes reemplazar los valores de `channelId` y `api_key` con los correspondientes a tu cuenta de ThingSpeak.
- Para utilizar Twilio, se deben proporcionar las credenciales de tu cuenta (SID y AuthToken).
- Este programa utiliza técnicas de control de tiempo no bloqueante para realizar las lecturas del sensor y enviar datos a ThingSpeak y Twilio en intervalos definidos por las constantes `updateThingSpeakInterval` y `updateTwilioInterval`.
- Los datos de conductividad y TDS (Total Dissolved Solids) se envían a ThingSpeak en campos 1 y 2, respectivamente.
- Los valores de TDS se calculan a partir de la resistencia medida por el sensor y se clasifican según rangos de calidad del agua establecidos por la OMS.
- Los mensajes de Twilio indican la calidad del agua según los rangos establecidos por la OMS.
- El código incluye comentarios explicativos para facilitar su comprensión y modificaciones.

## Importante

Este código es solo un ejemplo y puede requerir ajustes o mejoras según las necesidades específicas del proyecto.

```cpp
#include <WiFi.h>
#include <ThingSpeak.h>
#include "twilio.hpp"

// Declaración de todas nuestras variables
// (El código aquí omite las variables para lectura del sensor y cálculos, que se definen más adelante)

const char* ssid = "SSID";      // Cambiar por el nombre de tu red Wi-Fi
const char* password = "PASSWORD"; // Cambiar por la contraseña de tu red Wi-Fi

const long channelId =  channelId; // Reemplazar con el ID de tu canal en ThingSpeak
const char* api_key = "api_key";   // Cambiar por tu API Key de ThingSpeak

WiFiClient client;
Twilio *twilio;

unsigned long lastThingSpeakUpdate = 0;
unsigned long lastTwilioUpdate = 0;
const unsigned long updateThingSpeakInterval = 20000; // Intervalo de actualización de ThingSpeak (15 segundos)
const unsigned long updateTwilioInterval = 60000;    // Intervalo de actualización de Twilio (60 segundos)

void setup() {
  // (El código aquí omite la configuración de la comunicación serial y el pin de entrada)

  // Conexión Wi-Fi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  twilio = new Twilio("Account SSID", "Auth Token"); // Reemplazar con tus credenciales de Twilio
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
// (El código aquí omite los cálculos y envío de datos a ThingSpeak)

// Función para enviar mensaje de Twilio
// (El código aquí omite los cálculos y envío de mensajes Twilio)
```

## Resultado

