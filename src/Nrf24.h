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

#ifndef Nrf24_h
#define Nrf24_h

#define PASQO_NRF24_MAJOR 1
#define PASQO_NRF24_MINOR 0

#include "Spi.h"
#include "printf.h"

template <uint8_t CE_PIN, uint8_t CS_PIN>
struct Nrf24CeCs : public Spi<CS_PIN>
{
  typedef Spi<CS_PIN> SPI;

  enum
  {
    CE = 1 << BITMap(CE_PIN),
    CS = 1 << BITMap(CS_PIN)
  };

  Nrf24CeCs ()
  {
    // Default SPI data rate is 4Mbs for 16MHz clock which
    // is fine for the max radio data rate of 2Mbs.
    SPI::MasterInit();
    // Set the chip enable pin as output, low at reset.
    DDRMap(CE_PIN) |= CE;
  }

  void ChipEnable ()
  {
    PORTMap(CE_PIN) |= CE;
  }

  void ChipDisable ()
  {
    PORTMap(CE_PIN) &= ~CE;
  }

  protected:

  bool IrqEvent () { return true; }
};

template <uint8_t CE_PIN, uint8_t CS_PIN, uint8_t IRQ_PIN>
struct Nrf24CeCsIrq : public Nrf24CeCs<CE_PIN, CS_PIN>
{
  typedef Nrf24CeCs<CE_PIN, CS_PIN> Spi;
  typedef typename Spi::SPI SPI;

  enum
  {
    IRQ = 1 << BITMap(IRQ_PIN)
  };

  Nrf24CeCsIrq ()
  {
    // Set the chip enable pin as input.
    // This is redundant after reset.
    DDRMap(IRQ_PIN) &= ~IRQ;
  }

  bool IrqEvent ()
  {
    return !(PINMap(IRQ_PIN) & IRQ);
  }
};

template <class Pinout>
struct Nrf24 : public Pinout
{
  typedef typename Pinout::SPI SPI;

  using Pinout::IrqEvent;
  using Pinout::ChipEnable;
  using Pinout::ChipDisable;

  union RegMap
  {
    struct CONFIG_T
    {
      uint8_t PRIM_RX     :1;
      uint8_t PWR_UP      :1;
      uint8_t CRCO        :1;
      uint8_t EN_CRC      :1;
      uint8_t MASK_MAX_RT :1;
      uint8_t MASK_TX_DS  :1;
      uint8_t MASK_RX_DR  :1;

      void Print ()
      {
	printf ("CONFIG:\n"
	"  MASK_RX_DR: %i\n"
	"  MASK_TX_DS: %i\n"
	" MASK_MAX_RT: %i\n"
	"      EN_CRC: %i\n"
	"        CRCO: %i\n"
	"      PWR_UP: %i\n"
	"     PRIM_RX: %i\n",
	MASK_RX_DR,
	MASK_TX_DS,
	MASK_MAX_RT,
	EN_CRC,
	CRCO,
	PWR_UP,
	PRIM_RX);
      }
    } CONFIG;

    struct SETUP_RETR_T
    {
      uint8_t ARC         :4;
      uint8_t ARD         :4;

      void Print ()
      {
	printf ("SETUP_RETR:\n"
	"%12s: %i\n"
	"%12s: %i\n",
	"ARC", ARC,
	"ARD", ARD);
      }
    } SETUP_RETR;

    struct RF_SETUP_T
    {
      uint8_t RESERVED1   :1;
      uint8_t RF_PWR      :2;
      uint8_t RF_DR_HIGH  :1;
      uint8_t PLL_LOCK    :1;
      uint8_t RF_DR_LOW   :1;
      uint8_t RESERVED0   :1;
      uint8_t CONT_WAVE   :1;

      void Print ()
      {
	const char *pwr[] = { "-18dBm", "-12dBm", "-6dBm", "0dBm" };
	const char *dr[] = { "1Mbs", "2Mbs", "250kbs" };
	printf ("RF_SETUP:\n"
	"   CONT_WAVE: %i\n"
	"    PLL_LOCK: %i\n"
	"       RF_DR: %s\n"
	"      RF_PWR: %s\n",
	CONT_WAVE,
	PLL_LOCK,
	dr[RF_DR_LOW<<1|RF_DR_HIGH],
	pwr[RF_PWR]);
      }
    } RF_SETUP;

    struct RF_CH_T
    {
      uint8_t RF_CH       :7;
      uint8_t RESERVED    :1;

      void Print ()
      {
	printf (
	"       RF_CH: %i\n",
	RF_CH);
      }
    } RF_CH;

    struct STATUS_T
    {
      uint8_t TX_FULL     :1;
      uint8_t RX_P_NO     :3;
      uint8_t MAX_RT      :1;
      uint8_t TX_DS       :1;
      uint8_t RX_DR       :1;
      uint8_t RESERVED    :1;

      void Print ()
      {
	printf ("STATUS:\n"
	"       RX_DR: %i\n"
	"       TX_DS: %i\n"
	"      MAX_RT: %i\n"
	"     RX_P_NO: %i\n"
	"     TX_FULL: %i\n",
	RX_DR,
	TX_DS,
	MAX_RT,
	RX_P_NO,
	TX_FULL);
      }
    } STATUS;

    struct OBSERVE_TX_T
    {
      uint8_t ARC_CNT     :4;
      uint8_t PLOS_CNT    :4;

      void Print ()
      {
	printf ("OBSERVE_TX:\n"
	"%12s: %i\n"
	"%12s: %i\n",
	"ARC_CNT", ARC_CNT,
	"PLOS_CNT", PLOS_CNT);
      }
    } OBSERVE_TX;

    struct FIFO_STATUS_T
    {
      uint8_t RX_EMPTY    :1;
      uint8_t RX_FULL     :1;
      uint8_t RESERVED1   :2;
      uint8_t TX_EMPTY    :1;
      uint8_t FIFO_FULL   :1;
      uint8_t TX_REUSE    :1;
      uint8_t RESERVED0   :1;

      void Print ()
      {
	printf ("FIFO_STATUS:\n"
	"%12s: %i\n"
	"%12s: %i\n"
	"%12s: %i\n"
	"%12s: %i\n"
	"%12s: %i\n",
	"RX_EMPTY", RX_EMPTY,
	"RX_FULL", RX_FULL,
	"TX_EMPTY", TX_EMPTY,
	"FIFO_FULL", FIFO_FULL,
	"TX_REUSE", TX_REUSE);
      }
    } FIFO_STATUS;

    uint8_t reg;
  };

  enum Registers
  {
    // Configuration Register.
    CONFIG      = 0x00,
    MASK_RX_DR  = 1<<6,
    MASK_TX_DS  = 1<<5,
    MASK_MAX_RT = 1<<4,
    // Enable CRC. Forced high if one of the bits in the EN_AA is high.
    EN_CRC      = 1<<3, // Reset 1.
    // CRC encoding scheme. 0: 1 byte, 1: 2 bytes. Reset 0.
    CRCO        = 1<<2,
    // 1: Power up, 0: Power down. Reset 0.
    PWR_UP      = 1<<1,
    // RX/TX control. 1: PRX, 0: PTX. Reset 0.
    PRIM_RX     = 1<<0,

    // Enable Auto Acknoledgment.
    EN_AA       = 0x01,
    ENAA_P5     = 1<<5,
    ENAA_P4     = 1<<4,
    ENAA_P3     = 1<<3,
    ENAA_P2     = 1<<2,
    ENAA_P1     = 1<<1,
    ENAA_P0     = 1<<0,

    // Enable RX Addresses.
    EN_RXADDR   = 0x02,
    ERX_P5      = 1<<5,
    ERX_P4      = 1<<4,
    ERX_P3      = 1<<3,
    ERX_P2      = 1<<2,
    ERX_P1      = 1<<1, // Reset 1.
    ERX_P0      = 1<<0, // Reset 1.

    // Setup of Address Widths.
    SETUP_AW    = 0x03,
    // Bits 0:1.
    AW_3BYTE    = 1,
    AW_4BYTE    = 2,
    AW_5BYTE    = 3,

    // Setup of Automatic Retransmission.
    SETUP_RETR  = 0x04,
    // Auto Retransmission Delay - Bits 7:4.
    // VAL = [0x0, 0xF]. Delay = 250us * VAL. SETUP_RETR |= VAL << ARD.
    ARD         = 4,
    // Auto Retransmit Count - Bits 3:0.
    // VAL = [0x0, 0xF]. SETUP_RETR |= VAL << ARC.
    ARC         = 0,

    // RF Channel.
    // RF Channel Value - Bits 6:0.
    RF_CH       = 0x05,

    // RF Setup Register.
    RF_SETUP    = 0x06,
    CONT_WAVE   = 1<<7,
    RF_DR_LOW   = 1<<5,
    PLL_LOCK    = 1<<4,
    RF_DR_HIGH  = 1<<3,
    // RF Power Level - Bits 2:1
    RF_PWR      = 1,
    PWR_MASK    = 1<<2 | 1<<1,
    DR_MASK     = RF_DR_LOW | RF_DR_HIGH,

    // Status Register.
    STATUS      = 0x07,
    // Data Ready RX FIFO interrupt. Asserted when new data arrives in RX FIFO.
    // Write 1 to clear bit.
    RX_DR       = 1<<6,
    // Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX. If
    // AUTO_ACK is activated, this bit is set high only when ACK is received.
    // Write 1 to clear bit.
    TX_DS       = 1<<5,
    // Maximum number of TX retransmits interrupt. Write 1 to clear bit. If 
    // MAX_RT is asserted it must be cleared to enable further communication.
    MAX_RT      = 1<<4,
    // Data pipe number for the payload available for reading from RX_FIFO.
    // 000-101: Data Pipe Number. 110: Not Used. 111: RX_FIFO Empty.
    RX_P_NO     = 1, // Bits 3:1.
    // TX FIFO full flag.
    // 1: TX FIFO full. 0: Available locations in TX FIFO.
    TX_FULL     = 1<<0,

    // Transmit Observe Register.
    OBSERVE_TX  = 0x08,
    PLOS_CNT    = 4, // 7:4 - Count lost packets.
    ARC_CNT     = 0, // 3:0 - Count retransmitted packets.

    // Received Power Detector.
    RPD         = 0x09, // Bit 0.

    // Receive address data pipe registers.
    RX_ADDR_P0  = 0x0A, // 39:0 - 0xE7E7E7E7E7
    RX_ADDR_P1  = 0x0B, // 39:0 - 0xC2C2C2C2C2
    RX_ADDR_P2  = 0x0C, //  7:0 - 0xC3
    RX_ADDR_P3  = 0x0D, //  7:0 - 0xC4
    RX_ADDR_P4  = 0x0E, //  7:0 - 0xC5
    RX_ADDR_P5  = 0x0F, //  7:0 - 0xC6

    // Transmit address. Used for a PTX device only.
    TX_ADDR     = 0x10, // 39:0 - 0xE7E7E7E7E7

    // Receive payload width pipe registers. Bits 5:0.
    RX_PW_P0    = 0x11,
    RX_PW_P1    = 0x12,
    RX_PW_P2    = 0x13,
    RX_PW_P3    = 0x14,
    RX_PW_P4    = 0x15,
    RX_PW_P5    = 0x16,

    // FIFO Status,
    FIFO_STATUS = 0x17,
    TX_REUSE    = 1<<6,
    FIFO_FULL   = 1<<5,
    TX_EMPTY    = 1<<4,
    RX_FULL     = 1<<1,
    RX_EMPTY    = 1<<0,

    RESERVED    = 0x18,

    // Enable dynamic payload length.
    DYNPD	= 0x1C,
    DPL_P5      = 1<<5,
    DPL_P4      = 1<<4,
    DPL_P3      = 1<<3,
    DPL_P2      = 1<<2,
    DPL_P1      = 1<<1,
    DPL_P0      = 1<<0,

    // Feature register.
    FEATURE	= 0x1D,
    EN_DPL      = 1<<2, // Enables Dynamic Payload Length.
    EN_ACK_PAY  = 1<<1, // Enables Payload with ACK.
    EN_DYN_ACK  = 1<<0, // Enables the W_TX_PAYLOAD_NOACK command.

    ENDREG      = 0x1E
  };

  enum Commands
  {
    R_REGISTER     = 0x00,
    W_REGISTER     = 0x20,
    R_RX_PAYLOAD   = 0x61,
    W_TX_PAYLOAD   = 0xA0,
    FLUSH_TX       = 0xE1,
    FLUSH_RX       = 0xE2,
    REUSE_TX_PL    = 0xE3,
    R_RX_PL_WID    = 0x60,
    W_ACK_PAYLOAD  = 0xA8,
    W_ACK_PLD_MASK = 0x07,
    W_TX_PLD_NOACK = 0xB0,
    NOP            = 0xFF
  };

  enum PwrLevel
  {
    PA_MINUS_18_dBm  = 0<<RF_PWR,
    PA_MINUS_12_dBm  = 1<<RF_PWR,
    PA_MINUS_6_dBm   = 2<<RF_PWR,
    PA_0_dBm         = 3<<RF_PWR,
  };

  enum RfDataRate
  {
    DR_1Mbs   = 0,
    DR_2Mbs   = RF_DR_HIGH,
    DR_250Kbs = RF_DR_LOW
  };

  enum Timing
  {
    T_RST_TO_PD_ms      = 100,
    T_CE_MIN_HPULSE_us  = 10,
    T_PD_TO_STBY_us     = 4500,
    T_STBY_TO_TXRX_us   = 130,
  };

  enum 
  {
    WR_NO_ACK = 0,
    WR_ACK    = 1
  };

  // The default is a typical use which is also reflected in part in the 
  // default register configuration at reset, using pipe rx 0 for auto ack
  // and rx pipe 1 for rx.
  // I also setup dynamic payoad length and payload ack as it can handle the
  // most general case. We remain always in PTX mode and switch momentarily
  // to PRX mode. More general usage can be enable through the register
  // interface.
  Nrf24 ()
  {
    // If this is power on wait before you can communicate.
    delay (T_RST_TO_PD_ms);
    // Set CRC to 2 bytes, default is PTX mode.
    WrReg (CONFIG, EN_CRC | CRCO);
    // Address width is 5 bytes by default.
    // Retransmission is set at 1500us, 4 retries.
    WrReg (SETUP_RETR, 5 << ARD | 4 << ARC);
    // Default channel is 63.
    Channel (63);
    // Default data rate is 1Mbs.
    DataRate (DR_1Mbs);
    // Default power level max.
    PowerLevel (PA_0_dBm);
    // Use dynamic payloads with auto acknowledgment, ack payload.
    WrReg (FEATURE, EN_DPL | EN_DYN_ACK | EN_ACK_PAY);
    // Even with no ack payload DPL must be enabled on P0 as well.
    WrReg (DYNPD, DPL_P0 | DPL_P1);
    WrReg (EN_AA, ENAA_P0 | ENAA_P1);
  }

  uint8_t RdReg (uint8_t reg)
  {
    return SPI::Read (R_REGISTER | reg);
  }

  void RdReg (uint8_t reg, uint8_t *data, uint8_t size)
  {
    SPI::Read (R_REGISTER | reg, data, size);
  }

  void WrReg (uint8_t reg, uint8_t val)
  {
    SPI::Write (W_REGISTER | reg, val);
  }

  void WrReg (uint8_t reg, const uint8_t *data, uint8_t size)
  {
    SPI::Write (W_REGISTER | reg, data, size);
  }

  void SetReg (uint8_t reg, uint8_t bv)
  {
    WrReg (reg, RdReg (reg) | bv);
  }

  void ClrReg (uint8_t reg, uint8_t bv)
  {
    WrReg (reg, RdReg (reg) & ~bv);
  }

  void ExeCmd (uint8_t cmd)
  {
    SPI::Write (cmd);
  }

  void FlushTx ()
  {
    ExeCmd (FLUSH_TX);
  }

  void FlushRx ()
  {
    ExeCmd (FLUSH_RX);
  }

  void Reset ()
  {
    ExeCmd (FLUSH_RX);
    ExeCmd (FLUSH_TX);
    WrReg (STATUS, RX_DR | TX_DS | MAX_RT);
    WrReg (CONFIG, RdReg (CONFIG) & ~PWR_UP);
  }

  void Channel (uint8_t ch)
  {
    WrReg (RF_CH, ch & 0x7F);
  }

  uint8_t Channel ()
  {
    return RdReg (RF_CH);
  }

  void PowerLevel (PwrLevel pl)
  {
    uint8_t val = RdReg (RF_SETUP) & ~PWR_MASK;
    WrReg (RF_SETUP, val | pl);
  }

  PwrLevel PowerLevel () const
  {
    return RdReg (RF_SETUP) & PWR_MASK;
  }

  void DataRate (RfDataRate dr)
  {
    uint8_t reg = RdReg (RF_SETUP) & ~DR_MASK;
    WrReg (RF_SETUP, reg | dr);
  }

  RfDataRate DataRate () const
  {
    return RdReg (RF_SETUP) & DR_MASK;
  }

  void TxAddr (const void *addr)
  {
    WrReg (TX_ADDR   , (const uint8_t*) addr, 5);
    WrReg (RX_ADDR_P0, (const uint8_t*) addr, 5);
  }

  void RxAddr (const void *addr)
  {
    WrReg (RX_ADDR_P1, (const uint8_t*) addr, 5);
  }

  void TxMode ()
  {
    // Set as PTX.
    ClrReg (CONFIG, PRIM_RX);
    // Wait the radio turn around delay.
    delayMicroseconds (T_STBY_TO_TXRX_us);
  }

  void RxMode ()
  {
    // Set as PRX.
    SetReg (CONFIG, PRIM_RX);
    // Wait the radio turn around delay.
    delayMicroseconds (T_STBY_TO_TXRX_us);
  }

  // By setting the PWR_UP bit in the CONFIG register to 1, the device 
  // enters standby-I mode. Standby-I mode is used to minimize average
  // current consumption while maintaining short start up times. In this 
  // mode only part of the crystal oscillator is active. Change to active
  // modes only happens if CE is set high and when CE is set low, the 
  // nRF24L01 returns to standby-I mode from both the TX and RX modes.
  void PowerUp ()
  {
    SetReg (CONFIG, PWR_UP);
    delayMicroseconds (T_PD_TO_STBY_us);
  }

  void PowerDn ()
  {
    ClrReg (CONFIG, PWR_UP);
  }

  uint8_t PacketTxLost ()
  {
    return RdReg (OBSERVE_TX) >> PLOS_CNT;
  }

  uint8_t PacketTxCount ()
  {
    return RdReg (OBSERVE_TX) & 0xF;
  }

  // The use of IRQ avoids unnecessary SPI activity.
  bool DataReady ()
  {
    return IrqEvent() ? RdReg (STATUS) & RX_DR : 0;
  }

  // The use of IRQ avoids unnecessary SPI activity.
  uint8_t DataSent ()
  {
    return IrqEvent() ? RdReg (STATUS) & (TX_DS | MAX_RT) : 0;
  }

  void WrAck (const uint8_t *data, uint8_t size, uint8_t pipe = 1)
  {
    SPI::Write (W_ACK_PAYLOAD | (pipe & W_ACK_PLD_MASK), data, size);
  }

  void WrPld (const uint8_t *data, uint8_t size, uint8_t mode = W_TX_PAYLOAD)
  {
    SPI::Write (mode, data, size);
  }

  void RdPld (uint8_t *data, uint8_t size)
  {
    SPI::Read (R_RX_PAYLOAD, data, size);
  }

  bool WrPacketNoAck (const uint8_t *data, uint8_t size)
  {
    return WrPacket (data, size, WR_NO_ACK);
  }

  bool WrPacket (const uint8_t *data, uint8_t size, bool ack = WR_ACK)
  {
    // Write the TX payload.
    WrPld (data, size, ack ? W_TX_PAYLOAD : W_TX_PLD_NOACK);
    // Pulse CE to start TX.
    ChipEnable();
    delayMicroseconds(T_CE_MIN_HPULSE_us);
    ChipDisable();
    uint8_t status;
    // Wait for TX done.
    while (!(status = DataSent()));
    // Clear status flags.
    WrReg (STATUS, TX_DS | MAX_RT);
    // Flush TX fifo if TX failed.
    status &= MAX_RT;
    if (status)
      FlushTx();
    return !status;
  }

  // Read a packet if available. To be called when already powered up. 
  // In single mode the switch to TX is done internally. For stream RX
  // it needs to be done externally same way.
  bool RdPacket (uint8_t *data, uint8_t &size, 
                 uint16_t timeout_ms = 100, bool single = false)
  {
    if (single)
    {
      // Set as PRX.
      RxMode();
      // Go on the air.
      ChipEnable ();
    }

    bool timeout = false;
    uint16_t snap = millis();
    // Wait until there is data in the RX FIFO.
    while (!DataReady())
    {
      if (timeout_ms && millis() - snap > timeout_ms)
      {
	timeout = true;
	break;
      }
    }
    if (timeout)
      size = 0;
    else
    {
      // Read the payload size.
      size = RdReg (R_RX_PL_WID);
      // According to specs flush RX fifo if corruption has occurred.
      if (size > 32)
      {
	timeout = true;
	FlushRx();
      }
      else
      {
	// Read the payload if available.
	RdPld (data, size);
      }
      // Clear status.
      WrReg (STATUS, RX_DR);
    }

    if (single)
    {
      // Go off the air.
      ChipDisable();
      // Set the radio as PTX.
      TxMode();
    }
    // Use return and size to find out what happened.
    return !timeout;
  }
};
#endif // Nrf24_h
