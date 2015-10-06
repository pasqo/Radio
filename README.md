# RADIO
*NRF24 Library for Arduino, AVR 8 bit Microntrollers.*

## Overview
This library provides a simple but comprehensive support for NRF24 based radios to operate on 8 bit AVR Arduino boards as well as standalone AVR microntrollers.
It is a direct imlementation of the SPI based device specification detailed in the [NRF24 datasheet](https://www.nordicsemi.com/eng/Products/2.4GHz-RF/nRF24L01P).

For simplicity and to require the least amount of configuration, the default settings are for the most general use that includes a two way communication with dynamic payload and payload acknowledgement. The default behavior can be changed to more specific configurations through documented register operations.

For any details please consult the header files and underlying inlined implementation. The code should be fairly self-explanatory.

## Example
Right off the bat the library can be used for a range test between two devices using the `Console` and `Receiver` examples. The `Console` example is an interactive serial console application that when set in base mode can be used to send variable size packets to which the `Receiver` application responds. `Console` can also be used to scan the surroundings for used channels and can be easily extended to perform other test operations.

To use `Console` simply change the code to reflect the pin connections to the Arduino board, compile, upload, and open the Arduino serial console (or your preferred serial console application, I use CoolTerm) configured as 115200 8N1. It is best to use line mode if available with LF only termination.

To perform a channel scan just type `scan` and hit enter.
For a range test instead, load the `Receiver`application to another device and choose the `base`command in `Console`, where packet size and use of acknoledgment can be set as arguments.

For illustration a portion of the `Console` example code is reported here.
The pinout configuration is specified through template programming for both IO speed and code readability. Here, if I have a spare pin available I prefer to use the optional IRQ signal in order to reduce SPI bus activity when internally polling the nRF24 device.
```cpp
//////////////////////////////////////////////////////////////////////////////
// NRF24 Radio Pair Communication Test.
// Pasquale Cocchini <pasquale.cocchini@ieee.org>
//////////////////////////////////////////////////////////////////////////////

#include "AvrMap.h"
#include "Nrf24.h"
#include "ostream.h"

struct Shell
{
  // Choose how to connect the nRF24 to the Arduino.
  // Note how the connection is resolved statically by the compiler.

  // Connect to Arduino SPI pins and pins 7 and 8 for CE and CS.
  //typedef Nrf24< Nrf24CeCs<7, 8> > Radio;

  // Connect to Arduino SPI pins and pins 7, 8, 9 for CE, CS, IRQ.
  typedef Nrf24< Nrf24CeCsIrq<7, 8, 9> > Radio;

  .....

  // In base mode we send packets every second until any key is received
  // on the serial line.
  void Base (uint8_t size, bool ack)
  {
    const char *fp[] = {"FAIL", "PASS"};
    bool rs;
    uint8_t count = 0, data[32], ack_size;
    uint16_t stamp;

    m_radio.TxAddr ("NodeA");
    m_radio.RxAddr ("BaseA");

    m_radio.Reset();
    m_radio.PowerUp();

    while (!Serial.available())
    {
      delay (1000);
      data[0] = count++;
      stamp = micros();
      rs = m_radio.WrPacket (data, size, ack);
      stamp = micros() - stamp;
      ack_size = 0;
      if (m_radio.DataReady())
	    m_radio.RdPacket(data, ack_size);
      cout << "Packet " << count << " TX: " << fp[rs] 
	   << " SZ: " << size
	   << " CNT: " << m_radio.PacketTxCount()
	   << " ACK: " << ack
	   << " ACK_PLD: " << ack_size
	   << " in " << stamp << " us\n";
    }
    cout << "stop\n";

    m_radio.PowerDn();
  }
};

void setup ()
{
  Shell shell;
}

void loop () {}
```
The `Base` function illustrates a typical set-up for the radio.
At the beginning the transmit and receive addresses are specified, which must match corresponding addresses on the receiving device. `TxAddr` is the address of the device we are sending to and `RxAddr` is the address of the device we are programming.
Then we reset the device and power it up and enter a loop where we send packets with `WrPacket` and optionally checking if there is received data with `DataReady` which is then read with `RdPacket`.
Transmission statistics is then printed out on the serial line.

The `Receiver` application is even simpler.
Notice how the addresses are reversed with respect to `Console`.
```cpp
//////////////////////////////////////////////////////////////////////////////
// NRF24 Receiver Example.
// Pasquale Cocchini <pasquale.cocchini@ieee.org>
//////////////////////////////////////////////////////////////////////////////

#include "AvrMap.h"
#include "Nrf24.h"

// Choose how to connect the nRF24 to the Arduino.
// Note how the pinout connection is resolved statically by the compiler.

// Connect to Arduino SPI pins and pins 7 and 8 for CE and CS.
//typedef Nrf24< Nrf24CeCs<7, 8> > Radio;

// Connect to Arduino SPI pins and pins 7, 8, 9 for CE, CS, IRQ.
typedef Nrf24< Nrf24CeCsIrq<7, 8, 9> > Radio;

// Receive dynamically sized packets and optionally acknowledge with 
// variable size payloads.
void setup ()
{
  // Change this value to emulate variable size acknowledge payload.
  uint8_t ack_pld_size = 0;
  uint8_t data[32], size;

  Radio radio;

  radio.TxAddr ("BaseA");
  radio.RxAddr ("NodeA");

  radio.Reset();
  radio.PowerUp();
  radio.RxMode();
  radio.ChipEnable();

  while (1)
  {
    if (ack_pld_size)
      radio.WrAck (data, ack_pld_size);
    radio.RdPacket (data, size, 0);
  }

  radio.ChipDisable();
  radio.TxMode();
  radio.PowerDn();
}

void loop () {}
```

## Dependencies
This library depends on the https://github.com/pasqo/AvrMap library to make the code portable across supported arduino boards and AVR microcontrollers, please install it alongside to the Radio library.

## Portability
Although the code should work for all supported boards I have only been able to test it on boards with ATmega328p AVRs.
Reports of successful usage on other boards, or any issues, are welcome.

## Links
- [nRF24 website](https://www.nordicsemi.com/eng/Products/2.4GHz-RF/nRF24L01P)
