Wunderbaums changes:
- made a platformio version out of it.
- wifi-credentials are now in an external file include/wifidata.h
- added a whitelist in include/whitelist.h
- made home assistant auto discovery working


# ESP-EC3000-MQTT-Bridge
Empfängt EC3000-Pakete und dekodiert sie über einen RFM69, der mit einem ESP8266/ESP32 verbunden ist, und sendet sie nach einigen Plausibilitätsprüfungen an MQTT.
---
Receives EC3000 packets and decodes them via a RFM69 connected to an ESP8266/ESP32 and sends them to MQTT after some sanity checks.

!! Ihr müsst eure WLAN und MQTT Daten im Quellcode eingeben !!

MQTT Struktur:<br>
EC3000/debug		-> Removing stale ID (Durch fehlerhafte Pakete werden evtl. IDs erkannt die es nicht gibt welche aber sofort einen Platz im internen Tracker einnehmen.
                                     Wenn innerhalb von 66 Sekunden die ID _nicht_ nochmal empfangen wurde wird sie wieder von der internen Trackerliste gelöscht)<br>
EC3000/debug		-> Discarded (gefolgt von mindestens einer der folgenden Möglichkeiten)<br>
														OnSeconds > TotalSeconds;<br>
														IsOn=No but Power>0;<br>
														Power too high;<br>
														Resets not +1 (last=2050);<br>
														Consumption invalid (last=4.905, delta=11975.270)<br>
							also z.B.<br>
EC3000/debug		-> IsOn=No but Power>0; Power too high; Resets not +1 (last=2050); Consumption invalid (last=4.905, delta=11975.270);<br>
<br>
EC3000/[ID]<br>
EC3000/1234			-> {"TotalSeconds":693790,"OnSeconds":85,"Consumption":0.000,"Power":0.0,"MaximumPower":11.3,"NumberOfResets":3,"IsOn":0,"CRC":"0x7514","RSSI":-72.00}<br>
<br>
Jedes Paket wird einer Plausibilitätsüberprüfung unterzogen bevor es an MQTT geleitet wird.
Die Pakete werden verworfen wenn eine der folgenden Bedingungen zutrifft:
1. OnSeconds größer TotalSeconds
2. "IsOn" gleich "No" aber Power über 0
3. Strom zu hoch (aktuell 2000 Watt)
4. wenn "Resets" nicht gleich geblieben ist oder es sich anders geändert hat als "+1" (ein Reset ist wenn die Steckdose selbst vom Strom getrennt und wieder verbunden wurde)
   (wenn Ihr die Steckdose manuell zurückstellt (5 Sekunden auf die rote LED an der Steckdose drücken bis die LED dauerhaft leuchtet) müsst ihr sogesehen die Bridge auch kurz neustarten 
    da damit alle internen Werte der Steckdose auf 0 gesetzt werden und damit dass Tracking für diese ID verwirren würde)
6. Wenn "Consumption" stärker gestiegen ist als in fünf Sekunden theoretisch maximal möglich ist (0.025 bei 3600 Watt)

NOTES/TODO:
Der RSSI Wert ist immer gleich...

QUELLEN: (EC3000 Paketverarbeitung und dazugehörige RFM69 Initialisierung)
https://github.com/Rainerlan/LaCrosseGatewayMQTT
https://github.com/yustAnotherUser/ESP-EC3000-MQTT-Bridge
