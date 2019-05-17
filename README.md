# Integrate Grove Infrared Temperature Sensor Array with MangoH and Grove IoT Exapansion Card

This project demonstrates how to integrate Grove Infrared Temperature Sensor Array with MangoH and Grove IoT Exapansion Card.


## Prerequisites

* A mangOH Red board.
* A Grove IoT Expansion card.
* Infrared Temperature Sensor Array: See detail of sensor on wiki page: http://wiki.seeedstudio.com/Grove-Infrared_Temperature_Sensor_Array-AMG8833/ 

<img src="https://user-images.githubusercontent.com/17214533/57901207-43d17380-788e-11e9-9cc8-df3ccf7cab8f.jpg" width="350" alt="accessibility text"> 

Infrared Temperature Sensor Array
------------------
The Grove - Infrared Temperature Sensor Array (AMG8833) is a high precision infrared array sensor which based on advanced MEMS technology. It can support temperature detection of two-dimensional area: 8 × 8 (64 pixels)


Legato Application
------------------
There are 3 legato applications:
* ```infraredTemperature```: provides api for reading temperature sensor.
* ```infraredTemperatureSensor```: push temperature sensor value to datahub.
* ```infraredTemperatureDisplay```: register for notification of value updates, create observation (filter) for temperature sensor and display the temperature sensor value.


## Setup
1. Insert Grove IoT Expansion card into Mangoh Red
1. Jump 5V Pin on Grove IoT Card
1. Connect Infrared Temperature Sensor Array with I2C connector on Grove card



## How To Run

1. Build the AlcoholSensorService app by running ```mkapp -t wp85 infraredTemperature.adef``` in datahubInfraredTemperature directory.
1. Run ```instapp infraredTemperature.wp85.update 192.168.2.2``` to install the app.
1. Build the infraredTemperatureSensor app by running ```mkapp -t wp85 infraredTemperatureSensor.adef``` in datahubInfraredTemperature directory.
1. Run ```instapp infraredTemperatureSensor.wp85.update 192.168.2.2``` to install the app.
1. Build the infraredTemperatureDisplay app by running ```mkapp -t wp85 infraredTemperatureDisplay.adef``` in datahubInfraredTemperature directory.
1. Run ```instapp infraredTemperatureDisplay.wp85.update 192.168.2.2``` to install the app.
