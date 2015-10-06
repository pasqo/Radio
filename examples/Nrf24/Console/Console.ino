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

  Shell () 
  {
    static const char *help =
      "commands:\n"
      "  base [pkt_size [ack]] \n"
      "  node [ack_pld_size]\n"
      "  scan\n"
      "  reset\n"
      "  help\n";

    Serial.begin (115200);

    cout << "Nrf24 Shell by Pasqo\n";

    while (true)
    {
      Serial.print ("$ ");

      Tokenize ();

      if (Match("reset"))
      {
	m_radio.Reset();
      }
      else if (Match("base"))
      {
	uint8_t ack = 1, size = 1;
	if (m_tokc > 1)
          size = atoi (m_tokv[1]);
	if (m_tokc > 2)
          ack = atoi (m_tokv[2]);
	Base (size, ack);
      }
      else if (Match("node"))
      {
	uint8_t ack_pld_size = 0;
	if (m_tokc > 1)
          ack_pld_size = atoi (m_tokv[1]);
	Node (ack_pld_size);
      }
      else if (Match("scan"))
      {
	Scan ();
      }
      else if (Match("help"))
      {
	cout << help;
      }
      else if (*m_line)
      {
	cout << "-E-: Wrong command\n";
      }
    }
  }

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

  // In node mode we receive packets and print stats.
  void Node (uint8_t ack_pld_size)
  {
    const char *fp[] = {"FAIL", "PASS"};
    bool rs;
    uint8_t data[32], size;
    uint16_t stamp;

    m_radio.TxAddr ("BaseA");
    m_radio.RxAddr ("NodeA");

    m_radio.Reset();
    m_radio.PowerUp();
    m_radio.RxMode();
    m_radio.ChipEnable();

    // Listen for arriving packets and check for activity
    // on the serial line and return if any.
    while (!Serial.available())
    {
      data[0] = 0; // Invalid pkt id.
      if (ack_pld_size)
	m_radio.WrAck (data, ack_pld_size);
      rs = m_radio.RdPacket (data, size, 0);
      cout << "Packet " << data[0] << " RX: " << fp[rs] 
	   << " SZ: " << size
	   << " ACK: " << ack_pld_size << '\n';
    }
    cout << "stop\n";

    m_radio.ChipDisable();
    m_radio.TxMode();
    m_radio.PowerDn();
  }

  void Scan (uint16_t sample_time_ms = 8000)
  {
    uint8_t ch, add, val, orig;
    uint8_t rpd[16];
    memset (rpd, 0, 16);

    orig = m_radio.Channel();

    // Power up and set as PRX.
    m_radio.PowerUp();
    m_radio.RxMode();

    uint32_t snap = millis();
    while (millis() - snap < sample_time_ms)
    {
      ch = 0;
      while (!(ch & 0x80))
      {
	m_radio.Channel (ch);
	// According to datasheet RPD is valid after 170us and latched
	// after ACK flow or at CE low which we are doing here.
	m_radio.ChipEnable();
	delayMicroseconds(170);
	m_radio.ChipDisable();
	add = ch >> 3;
	val = 1 << (ch & 0x07);
	if (m_radio.RdReg (Radio::RPD))
	  rpd[add] |= val;
	ch++;
      }
    }

    cout << "Received Power Detection\n";

    ch = 0;
    while (!(ch & 0x80))
    {
      add = ch >> 3;
      val = 1 << (ch & 0x07);
      if (rpd[add] & val)
	cout << "Carrier on channel " << ch << '\n';
      ch++;
    }

    // Set as PTX and power down.
    m_radio.TxMode();
    m_radio.PowerDn();
    // Restore original channel.
    m_radio.Channel (orig);
  }

  private:

  bool Match (const char *tok)
  {
    return !strcmp (m_line, tok);
  }

  void Tokenize ()
  {
    char *c, *e = m_line + 63;
    m_tokc = 0;
    m_tokv[m_tokc] = c = m_line;
    while (c != e)
    {
      if (Serial.available())
      {
	*c = Serial.read();
	if (*c == '\n')
	  break;
	if (*c == ' ')
	{
	  *c = '\0';
	  m_tokv[++m_tokc] = c + 1;
	}
	c++;
      }
    }
    *c = '\0';
    m_tokc++;
  }

  private:

  char m_line[32], *m_tokv[4];
  byte m_tokc;
  Radio m_radio;
};

void setup ()
{
  Shell shell;
}

void loop () {}
