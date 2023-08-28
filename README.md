# Smart mailbox
This repo contains the full code that I use for my mailbox. As hardware I'm using 2 ToF (Time of Flight) sensors, an RFID reader, an RGBW LED-strip, 2 reed switches, a solenoid lock and an ESP32.

The objective was to be notified when a new letter or package was dropped in my mailbox. I also want to protect my mail by using a lock that can be opened using an RFID tag or MQTT message.

## Hardware
* Geroba® - Pakun (mailbox)
* Espressif ESP32 WROOM 32U devkit C
* 2.4Ghz WiFi antenna (because my mailbox is made out of steel)
* VL6180X ToF sensor (2 pieces)
* MFRC-522 Mini RFID reader
* Elektromagnetic lock (24V)
* Reed switch (2 pieces)
* HLK-VRB2403YMD DC-DC stepdown (24V to 3.3V)

## More info
I'm also using Loxone & Loxberry to make my house a little smarter. Communication between the mailbox and Loxone/Loxberry happens with MQTT.
