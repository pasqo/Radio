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
// NRF24 Unit Test.
// Pasquale Cocchini <pasquale.cocchini@ieee.org>
//////////////////////////////////////////////////////////////////////////////

#include "AvrMap.h"
#include "Spi.h"
#include "Nrf24.h"
#include "printf.h"

template <class Pinout>
struct Nrf24Shell : public Nrf24<Pinout>
{
  typedef Nrf24<Pinout> RF24;
  typedef typename RF24::RegMap RegMap;

  using RF24::RdReg;
  using RF24::WrReg;
  using RF24::TxAddr;
  using RF24::RxAddr;
  using RF24::RdPld;
  using RF24::WrPld;
  using RF24::ChipEnable;
  using RF24::ChipDisable;
  using RF24::PowerUp;
  using RF24::WrPacket;
  using RF24::WrPacketNoAck;
  using RF24::RdPacket;
  using RF24::Channel;
  using RF24::Reset;
  using RF24::SetReg;
  using RF24::ClrReg;

  Nrf24Shell () 
  {
    printf_init();

    static const char *help =
      "Available Commands:\n"
      "config\n"
      "status\n"
      " reset\n"
      "  scan [<sample_time_ms=1000>]\n"
      "  test\n";

    RegMap r;

    delay (1000);

    Serial.begin (115200);

    printf ("Nrf24 Shell by Dudder\n");

    while (true)
    {
      Serial.print ("$ ");

      Tokenize ();

      if (!strcmp (m_line, "config"))
      {
	r.reg = RdReg (RF24::CONFIG);
	r.CONFIG.Print();
	r.reg = RdReg (RF24::RF_SETUP);
	r.RF_SETUP.Print();
	r.reg = RdReg (RF24::RF_CH);
	r.RF_CH.Print();
	r.reg = RdReg (RF24::SETUP_RETR);
	r.SETUP_RETR.Print();
      }
      else if (!strcmp (m_line, "status"))
      {
	r.reg = RdReg (RF24::STATUS);
	r.STATUS.Print();
	r.reg = RdReg (RF24::FIFO_STATUS);
	r.FIFO_STATUS.Print();
	r.reg = RdReg (RF24::OBSERVE_TX);
	r.OBSERVE_TX.Print();
      }
      else if (!strcmp (m_line, "test"))
      {
	Test();
      }
      else if (!strcmp (m_line, "reset"))
      {
	RF24::Reset();
      }
      else if (!strcmp (m_line, "scan") && m_tokc < 3)
      {
	// Sample time is capped at 65s.
	uint16_t sample_time_ms = 1000;
	if (m_tokc == 2)
          sample_time_ms = atoi (m_tokv[1]);
	ChannelDetector(sample_time_ms);
      }
      else if (!strcmp (m_line, "help"))
      {
	printf ("%s", help);
      }
      else
      {
	printf ("-E-: unknown or wrong command\n");
      }
    }
  }

  bool Test ()
  {
    RegMap r;
    uint8_t test, size;
    uint32_t snap;
    const char *res[] = {"FAIL", "PASS"};
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    uint8_t read[6] = {0, 0, 0, 0, 0, 0};

    printf ("TX_TEST\n");

    RF24::Reset ();

    // Set addresses.
    RxAddr ("BASE0");
    TxAddr ("RCVR0");

    // Test SPI connectivity through a register read-back.
    RdReg (RF24::TX_ADDR, data, 5);
    test = strcmp ((char*)data, "RCVR0") ? 0 : 1;
    printf ("%16s: %s\n", "SPI_LINK", res[test]);

    // Power up the radio.
    PowerUp ();
    r.reg = RdReg (RF24::CONFIG);
    test = r.CONFIG.PWR_UP;
    printf ("%16s: %s\n", "PWR_UP", res[test]);

    // Write a 1 byte packet in no ack mode.
    data[0] = 0xAB;
    WrPld (data, 1, RF24::W_TX_PLD_NOACK);
    r.reg = RdReg (RF24::FIFO_STATUS);
    test = 1 - r.FIFO_STATUS.TX_EMPTY;
    printf ("%16s: %s\n", "WR_PLD_NO_ACK", res[test]);

    // Transmit with a CE pulse.
    ChipEnable();
    snap = micros();
    delayMicroseconds(RF24::T_CE_MIN_HPULSE_us);
    ChipDisable();
    // Wait until the data is sent.
    do { if (micros() - snap > 10000) break; r.reg = RdReg (RF24::STATUS); }
    while (!(r.reg & (RF24::TX_DS | RF24::MAX_RT)));
    snap = micros() - snap;
    test = r.STATUS.TX_DS;
    printf ("%16s: %s TIME: %d us\n", "TX_DS", res[test], snap);
    r.reg = RdReg (RF24::FIFO_STATUS);
    test = r.FIFO_STATUS.TX_EMPTY;
    printf ("%16s: %s\n", "TX_FIFO_EMPTY", res[test]);
    r.reg = RdReg (RF24::STATUS);
    test = 1 - r.STATUS.MAX_RT;
    printf ("%16s: %s\n", "MAX_RT", res[test]);
    // Clear status flags.
    WrReg (RF24::STATUS, RF24::TX_DS | RF24::MAX_RT);
    r.reg = RdReg (RF24::STATUS);
    test = 1 - r.STATUS.TX_DS;
    printf ("%16s: %s\n", "CLR_TX_DS", res[test]);

    snap = micros();
    test = WrPacketNoAck (data, 1);
    snap = micros() - snap;
    printf ("%16s: %s TIME: %d us\n", "TX_PKT_NO_ACK", res[test], snap);

    // Write a packet with ack that fails.
    snap = micros();
    test = 1 - WrPacket (data, 1);
    snap = micros() - snap;
    printf ("%16s: %s TIME: %d us\n", "TX_PKT_ACK", res[test], snap);
    r.reg = RdReg (RF24::OBSERVE_TX);
    test = r.OBSERVE_TX.ARC_CNT == 4 ? 1 : 0;
    printf ("%16s: %s\n", "ARC_CNT", res[test]);
    test = r.OBSERVE_TX.PLOS_CNT > 0 ? 1 : 0;
    printf ("%16s: %s\n", "PLOS_CNT", res[test]);

    printf ("RX_TEST\n");
    snap = micros();
    test = 1 - RdPacket (data, size, 10);
    snap = micros() - snap;
    printf ("%16s: %s TIME: %d us\n", "RX_PKT_ACK", res[test], snap);
    test = size == 0 ? 1 : 0;
    printf ("%16s: %s\n", "RX_PKT_ACK_SIZE", res[test]);
  }

  void ChannelDetector (uint16_t sample_time_ms)
  {
    uint8_t ch, add, val, orig;
    uint8_t rpd[16];
    memset (rpd, 0, 16);

    printf ("CHANNEL CARRIER DETECTION\n");

    Reset ();
    PowerUp ();
    orig = Channel();
    // Set as PRX.
    SetReg (RF24::CONFIG, RF24::PRIM_RX);

    uint32_t snap = millis();
    while (millis() - snap < sample_time_ms)
    {
      ch = 0;
      while (!(ch & 0x80))
      {
	Channel (ch);
	// According to datasheet RPD is valid after 170us and latched
	// after ACK flow or at CE low which we are doing.
	ChipEnable();
	delayMicroseconds(170);
	ChipDisable();
	add = ch >> 3;
	val = 1 << (ch & 0x07);
	if (RdReg (RF24::RPD))
	  rpd[add] |= val;
	ch++;
      }
    }

    ch = 0;
    while (!(ch & 0x80))
    {
      add = ch >> 3;
      val = 1 << (ch & 0x07);
      if (rpd[add] & val)
	printf ("Carrier on channel %i\n", ch);
      ch++;
    }

    // Set as PTX.
    ClrReg (RF24::CONFIG, RF24::PRIM_RX);
    // Wait the radio turn around delay.
    delayMicroseconds(RF24::T_STBY_TO_TXRX_us);
    Channel (orig);
  }

  private:

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

};

typedef Nrf24Shell< Nrf24CeCs<7,8> > Radio;

int main ()
{
  init();
  Radio radio;
}
