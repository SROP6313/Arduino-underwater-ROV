# Arduino-underwater-ROV
The arduino project on Platformio in Microsoft VScode
<br>
## Communication Protocal Overview <I>(last edit : 09/03/2022)</I>
![圖片](https://user-images.githubusercontent.com/103128273/188257759-3a2d44f8-84d8-4325-b277-ad3f182dfbbf.png)
<br>
Purpose: Users can use the HTML UI built by ESP32(CAM) webserver to control motors connected to Mega2560.
<br>
## The Protocal between Arduino Mega2560 and ESP32
<br>
<div align="center">

| Protocal  | Analysis|
| ---------- | -----------|
| UART   | The serial buffer data often accumulate and delay. The stability is not good. |
| I2C   | The transmission dalay is very small without sensor installed. But there's no example on internet that can show one device can be I2C master and slave at the same time. I tried SoftwareI2C libraries, but they only support I2C master mode, and most sensors do not support the libraries. The SoftwareI2C libraries seem cannot be put together with normal I2C library(Wire.h).  |
| SPI   | The transmission speed is the fastest. But the two-way communication codes are more complex than I2C(I'm still working on this).  |

</div>
<br>
