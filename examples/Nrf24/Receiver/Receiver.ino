//////////////////////////////////////////////////////////////////////////////
// RF24 Library for Arduino, AVR 8 bit Microntrollers.
//
// Copyright (C) 2015 Pasquale Cocchini <pasquale.cocchini@ieee.org>
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
