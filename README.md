# Smart mailbox
This repo contains the full code that I use for my mailbox. As hardware I'm using 2 ToF (Time of Flight) sensors, an RFID reader, an RGBW LED-strip, 2 reed switches, a solenoid lock and an ESP32.

The objective was to be notified when a new letter or package was dropped in my mailbox. I also want to protect my mail by using a lock that can be opened using an RFID tag or MQTT message.

## Hardware
* Geroba® - Pakun (mailbox)
* Espressif ESP32 WROOM 32U devkit C
* 2.4Ghz WiFi antenna (because my mailbox is made out of steel)
* VL6180X ToF sensor (2 pieces)
* MFRC-522 Mini RFID reader
* Electromagnetic lock (24V)
* Reed switch (2 pieces)
* HLK-VRB2403YMD DC-DC stepdown (24V to 3.3V)
* 3D printer to print some casing for all the hardware

## More info
I'm also using Loxone & Loxberry to make my house a little smarter. Communication between the mailbox and Loxone/Loxberry happens with MQTT.

## Libraries
The existing libraries found on Github for the ToF sensors didn't work 100% or where not accurate enough. So I've created a new one based on diffrent repo's (https://github.com/DFRobot/DFRobot_VL6180X & https://github.com/pololu/vl6180x-arduino).
The other libraries (for the RFID reader, MQTT, ...) can be found on Github.

## Photos
![Power and ESP](https://github.com/ruudvdb/smart-mailbox/blob/main/mailbox1.jpg?raw=true)

Box on the left: 
* 1 x MOSFET for the electromagnetic lock (protected with a flyback diode)
* 4 x MOSFET for the RGWB LED-strip
* DC to DC stepdown

Box on te right:
* ESP32 devkit C

---

![Overview](https://github.com/ruudvdb/smart-mailbox/blob/main/mailbox2.jpg?raw=true)

Overall overview
