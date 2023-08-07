# Detector de Calidad del Agua con Sensor Casero

Este es un programa para un detector de calidad del agua utilizando un sensor casero. El programa mide la conductividad del agua, calcula el valor de TDS (Total Dissolved Solids) y envía los datos a ThingSpeak para su visualización y monitoreo en tiempo real. Además, utiliza el servicio de Twilio para enviar mensajes de texto con la clasificación de la calidad del agua según los rangos establecidos por la Organización Mundial de la Salud (OMS). También incluye la funcionalidad para sensar la temperatura del agua utilizando un sensor DS18B20.

## Descripción

El detector de calidad del agua se construye utilizando los siguientes componentes:

- Un microcontrolador compatible con WiFi (por ejemplo, ESP32).
- Un sensor de conductividad casero que mide la caída de voltaje entre dos puntos en el agua y calcula la conductividad en S/m (siemens por metro).
- Un sensor DS18B20 para medir la temperatura del agua.
- Una conexión a Internet para enviar datos a ThingSpeak y utilizar el servicio de Twilio.

## Configuración del Hardware

El hardware necesario para el detector de calidad del agua incluye:

- Microcontrolador con WiFi (por ejemplo, ESP32).
- Resistencia de 1k ohm.
- Jumpers o alambre de conexiones.
- Sensor DS18B20.
- Conexiones del sensor de conductividad:
  - Pin de señal/salida del sensor conectado al pin analógico del microcontrolador (por ejemplo, Pin 36).
  - VCC del sensor conectado a 3.3V del microcontrolador.
  - GND del sensor conectado a GND del microcontrolador.

## Dependencias y Librerías

El programa utiliza las siguientes bibliotecas para su funcionamiento:

- `WiFi.h`: Para el control de WiFi en el microcontrolador.
- `ThingSpeak.h`: Para enviar datos a ThingSpeak y visualizarlos.
- `twilio.hpp`: Para enviar mensajes SMS mediante Twilio.
- `OneWire.h`: Para la comunicación con el sensor DS18B20.
- `DallasTemperature.h`: Para el sensor DS18B20.

Asegúrate de instalar estas bibliotecas antes de compilar el programa.

## Configuración de Credenciales

Antes de cargar el programa en el microcontrolador, asegúrate de realizar las siguientes configuraciones:

- Modifica las constantes `ssid` y `password` con los datos de tu red WiFi.
- Reemplaza los valores de `channelId` y `api_key` con los correspondientes a tu cuenta de ThingSpeak.
- Proporciona las credenciales de tu cuenta Twilio en la instancia `twilio`.

## Funcionamiento del Programa

El programa utiliza técnicas de control de tiempo no bloqueante para realizar las lecturas del sensor y enviar datos a ThingSpeak y Twilio en intervalos definidos por las constantes `updateThingSpeakInterval` y `updateTwilioInterval`. Estos intervalos controlan la frecuencia de actualización de los datos.

El valor de conductividad y el valor de TDS (Total Dissolved Solids) se envían a ThingSpeak en campos 1 y 2, respectivamente. El cálculo del valor de TDS se realiza a partir de la resistencia medida por el sensor de conductividad y se clasifica según los rangos de calidad del agua establecidos por la OMS.

Los mensajes de Twilio indican la calidad del agua según los rangos establecidos por la OMS y se envían al número especificado en la función `enviarMensajeTwilio`.

## Notas Finales

El código incluye comentarios explicativos para facilitar su comprensión y hacer modificaciones según sea necesario. Asegúrate de tener todas las conexiones y componentes configurados correctamente antes de probar el detector de calidad del agua.

Es importante destacar que este es un proyecto de sensor casero y los resultados pueden variar según la calidad y precisión del sensor de conductividad utilizado. Además, la calidad del agua puede estar influenciada por otros factores que no se toman en cuenta en este programa.

¡Disfruta construyendo y probando tu Detector de Calidad del Agua con Sensor Casero!