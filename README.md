# parallel_processing_ESP32_HR_accel_SpO2
The aim of this project is parallel processing. 
A microcontroller with 2 cores (ESP32) is used to implement parallel processing using FreeRTOS.
2 sensors are being used, an accelerometer (ADXL362) and MAX30105.
The former measures acceleration and the later measures HR and SpO2.

The HR measurements, when there is no movement detection, are taken using the IR LED of MAX30105. 
When there is movement, the green LED is used for that purpose.

The SpO2 measurements take place only when there is no movement.

Event groups are being used to synchronise all the tasks that run in parallel.
The acceleration task runs in Core 0, the HR and the SpO2 tasks run in Core 1.

The data is being displayed in the format of a website. That was build using html, CSS, Javascript. 
The charts are displayed with the aid of the Highcharts Javascrupt library.
The html file is located inside the folder "data".
That file is transfered to the microcontroller using SPIFFS.

Inside this code, there is a commented script about transfering data to FireBase and displaying it in the RTDB.
