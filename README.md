| Supported Targets | ESP32-S2 | ESP32-S3 |
| ----------------- | -------- | -------- |

# USB MIDI Class Example (with TX and RX operation)


This example provides basic USB Host Midi functionality by implementing a midi class driver and a Host Library task to send and receive MIDI messages. The example does the following:

1. Install Host Library and register a client
2. Waits for a device connection
3. Prints the device's information (such as device/configuration/string descriptors)
4. Claims Interface with more that zero endpoints from device
5. Prints received bytes, if midi-device sent new data
6. Send MIDI data when a push button is pressed

The example heavily follows the usb_host_lib example and MIDI support implemented in https://github.com/Wunderbaeumchen99817/esp-idf/tree/master/examples/peripherals/usb/host/midi

## How to use example

Simply connect the MIDI device and press the push button connected to the ESP32 to send a MIDI message. The MIDI device can also send MIDI messages back to the ESP32.

### Hardware Required

- Development board with USB capable ESP SoC (ESP32-S2/ESP32-S3, tested on ESP32-S3)
- USB cable for Power supply and programming
- USB OTG Cable
- MIDI-device (e.g. M-VAVE Blackbox)

### Build and Flash


## Example Output
