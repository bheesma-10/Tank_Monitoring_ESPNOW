# ESPNOW
this project explores tank monitoring system with a two way communication using ESP-NOW protocol.
One ESP32 device is connected at the motor end while the other is connected at the tank end on terrace.
Distance between the two can be considered between 10-15 meters.  
A push button is provided to initiate the process once until tank gets filled and motor stops automatically. Also, taking into consideration
manual operation also, a 2 way switch is used alongside relay.
Change the receiver and sender MAC address in both the provided code as per your available devices before burning the firmware.  
uint8_t receiver_MAC[] = {0x30,0xc6,0xf7,0x29,0x24,0x01}
uint8_t sender_MAC[]   = {0x3c,0x61,0x05,0x30,0x60,0x61}  
