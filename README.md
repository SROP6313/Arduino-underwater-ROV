# Arduino-underwater-ROV
The arduino project on Platformio in Microsoft VScode
<br>
## Communication Protocol Overview <I>(last edit : 09/03/2022)</I>
![圖片](https://user-images.githubusercontent.com/103128273/188257759-3a2d44f8-84d8-4325-b277-ad3f182dfbbf.png)
<br>
Purpose: Users can use the HTML UI built by ESP32(CAM) webserver to control motors connected to Mega2560. And the sensor data(pressure & temperature) can also show on the HTML UI.
<br>
## The program flow chart <I>(last edit : 09/12/2022)</I>
![圖片](https://user-images.githubusercontent.com/103128273/189593978-d71ab29a-e495-48fb-960a-d0849e7bd1d6.png)

## The Protocol between Arduino Mega2560 and ESP32  <I>(last edit : 09/09/2022)</I>
<br>
<div align="center">

| Protocol  | Analysis| main code |
| ---------- | -----------| -----------|
| ~~UART~~  | Probaly because UART does not have easy code about interrupting. The serial buffer data often accumulate and delay. The stability is not good. | send：```Serial.print()``` receive：```Serial.read()```|
| ~~I2C~~   | The transmission delay is very small without sensor installed. But there's no example on internet that can show one device can be I2C master and slave at the same time. I tried SoftwareI2C libraries, but they only support I2C master mode, and most sensors do not support the libraries. The SoftwareI2C libraries seem cannot be put together with normal I2C library(Wire.h) in the same sketch.  | send：```Wire.write()``` receive：```Wire.read()```|
| SPI  | The transmission speed is the fastest. But the two-way communication codes are more complex than I2C's. ```SPDR```is the data buffer during transmission. I put the SPI.transfer() in the loop function of the Master device to avoid breaking the routine program. | send：```SPI.transfer()``` receive：```byte c = SPDR```|

</div>

## Webpage view <I>(last edit : 10/16/2022)</I>
|||
| ---------- | -----------|
| ![圖片](https://user-images.githubusercontent.com/103128273/196038210-581b000e-1f84-42a2-913f-999adfb38645.png) | ![圖片](https://user-images.githubusercontent.com/103128273/196038302-268e7ed4-bf36-401f-8699-525f38c08940.png)|

Explanation: The webpage show the `battery voltage indicator icon`,`hyperlink buttons`, `ip camera stream from ESP32-CAM`, `ROV operating buttons`, `current rotating speed status of ROV thrusters` and `sensor values`. It also contain `Wi-fi alert`,`battery voltage alert`. 
