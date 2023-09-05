#!/usr/bin/env python3
# -*- coding: utf8 -*-
#
#    Copyright 2014,2018 Mario Gomez <mario.gomez@teubi.co>
#
#    This file is part of MFRC522-Python
#    MFRC522-Python is a simple Python implementation for
#    the MFRC522 NFC Card Reader for the Raspberry Pi.
#
#    MFRC522-Python is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    MFRC522-Python is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with MFRC522-Python.  If not, see <http://www.gnu.org/licenses/>.
#

import RPi.GPIO as GPIO
import spi
import signal
import time

class MFRC522:
  NRSTPD = 22

  MAX_LEN = 16

  # PCD commands (MFRC522)
  PCD_IDLE       = 0x00  # No Action / cancel current Command execution
  PCD_CALCCRC    = 0x03  # Activate CRC Coprocessor or performs a self test
  PCD_TRANSMIT   = 0x04  # Transmit data from FIFO buffer
  PCD_RECEIVE    = 0x08  # Activate the Receiver Circuits
  PCD_TRANSCEIVE = 0x0C
  PCD_AUTHENT    = 0x0E  # MIFARE standard auth as a reader
  PCD_RESETPHASE = 0x0F  # Reset the MFRC522

  # PICC commands used by PCD - to manage communication with several PICCs (ISO 14443-3, Type A, sec 6.4)
  PICC_REQIDL    = 0x26  # Request command, Type A | Invite PICCs in state IDLE => READY (and prepare for AntiColl/Sel)
  PICC_REQALL    = 0x52  # Wake-up command, Type A | Invite PICCs in state IDLE or HALT => READY (and prepare for AntiColl/Sel)
  PICC_SEL_CL1   = 0x93  # Anti-Collision / Select, Cascade Level 1
  PICC_SEL_CL2   = 0x95  # Anti-Collision / Select, Cascade Level 2
  PICC_SEL_CL3   = 0x97  # Anti-Collision / Select, Cascade Level 3
  PICC_TAG_COLL  = 0x88  # Cascade Tag (Tag used during AntiCol; not a command)
  PICC_AUTHENT1A = 0x60
  PICC_AUTHENT1B = 0x61
  PICC_READ      = 0x30
  PICC_WRITE     = 0xA0
  PICC_DECREMENT = 0xC0
  PICC_INCREMENT = 0xC1
  PICC_RESTORE   = 0xC2
  PICC_TRANSFER  = 0xB0
  PICC_HALT      = 0x50

  # Return codes
  MI_OK       = 0
  MI_NOTAGERR = 1
  MI_ERR      = 2

  # MFRC522 register | Page 0: Command and Status
  Reserved00     = 0x00
  CommandReg     = 0x01
  CommIEnReg     = 0x02
  DivlEnReg      = 0x03
  CommIrqReg     = 0x04
  DivIrqReg      = 0x05
  ErrorReg       = 0x06
  Status1Reg     = 0x07
  Status2Reg     = 0x08
  FIFODataReg    = 0x09
  FIFOLevelReg   = 0x0A
  WaterLevelReg  = 0x0B
  ControlReg     = 0x0C
  BitFramingReg  = 0x0D
  CollReg        = 0x0E
  Reserved01     = 0x0F

  # MFRC522 register | Page 1: Command
  Reserved10     = 0x10
  ModeReg        = 0x11
  TxModeReg      = 0x12
  RxModeReg      = 0x13
  TxControlReg   = 0x14
  TxAutoReg      = 0x15
  TxSelReg       = 0x16
  RxSelReg       = 0x17
  RxThresholdReg = 0x18
  DemodReg       = 0x19
  Reserved11     = 0x1A
  Reserved12     = 0x1B
  MifareReg      = 0x1C
  Reserved13     = 0x1D
  Reserved14     = 0x1E
  SerialSpeedReg = 0x1F

  # MFRC522 register | Page 2: Configuration
  Reserved20        = 0x20
  CRCResultRegM     = 0x21
  CRCResultRegL     = 0x22
  Reserved21        = 0x23
  ModWidthReg       = 0x24
  Reserved22        = 0x25
  RFCfgReg          = 0x26
  GsNReg            = 0x27
  CWGsPReg          = 0x28
  ModGsPReg         = 0x29
  TModeReg          = 0x2A
  TPrescalerReg     = 0x2B
  TReloadRegH       = 0x2C
  TReloadRegL       = 0x2D
  TCounterValueRegH = 0x2E
  TCounterValueRegL = 0x2F

  # MFRC522 register | Page 3: Test register
  Reserved30      = 0x30
  TestSel1Reg     = 0x31
  TestSel2Reg     = 0x32
  TestPinEnReg    = 0x33
  TestPinValueReg = 0x34
  TestBusReg      = 0x35
  AutoTestReg     = 0x36
  VersionReg      = 0x37
  AnalogTestReg   = 0x38
  TestDAC1Reg     = 0x39
  TestDAC2Reg     = 0x3A
  TestADCReg      = 0x3B
  Reserved31      = 0x3C
  Reserved32      = 0x3D
  Reserved33      = 0x3E
  Reserved34      = 0x3F

  def __init__(self, device='/dev/spidev0.0', speed=1000000, debug=True):
    self.debug = debug

    self.spiDev = spi.openSPI(device=device, speed=speed)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(self.NRSTPD, GPIO.OUT)
    GPIO.output(self.NRSTPD, 1)
    self.MFRC522_Init()

  def MFRC522_Reset(self):
    self.Write_MFRC522(self.CommandReg, self.PCD_RESETPHASE)

  def Write_MFRC522(self, addr, val):
    # SPI address byte -> write: 0AAAAAA0 (6-bit address)
    spi.transfer(self.spiDev, ((addr<<1) & 0x7E, val))

  def Read_MFRC522(self, addr):
    # SPI address byte -> read: 1AAAAAA0 (6-bit address)
    val = spi.transfer(self.spiDev, (((addr<<1) & 0x7E) | 0x80, 0))
    return val[1]

  def SetBitMask(self, reg, mask):
    tmp = self.Read_MFRC522(reg)
    self.Write_MFRC522(reg, tmp | mask)

  def ClearBitMask(self, reg, mask):
    tmp = self.Read_MFRC522(reg)
    self.Write_MFRC522(reg, tmp & (~mask))

  def AntennaOn(self):
    tmp = self.Read_MFRC522(self.TxControlReg)
    if ~(tmp & 0x03):
      self.SetBitMask(self.TxControlReg, 0x03)

  def AntennaOff(self):
    self.ClearBitMask(self.TxControlReg, 0x03)

  def MFRC522_ToCard(self, command, sendData):
    backData = []
    backLen = 0
    status = self.MI_ERR
    irqEn = 0x00
    waitIRq = 0x00
    lastBits = None
    n = 0

    if command == self.PCD_AUTHENT:
      irqEn = 0x12
      waitIRq = 0x10
    if command == self.PCD_TRANSCEIVE:
      irqEn = 0x77
      waitIRq = 0x30

    self.Write_MFRC522(self.CommIEnReg, irqEn | 0x80)
    self.ClearBitMask(self.CommIrqReg, 0x80)
    self.SetBitMask(self.FIFOLevelReg, 0x80)

    self.Write_MFRC522(self.CommandReg, self.PCD_IDLE)

    for i in range(len(sendData)):
      self.Write_MFRC522(self.FIFODataReg, sendData[i])

    self.Write_MFRC522(self.CommandReg, command)

    if command == self.PCD_TRANSCEIVE:
      self.SetBitMask(self.BitFramingReg, 0x80)

    i = 2000
    while True:
      n = self.Read_MFRC522(self.CommIrqReg)
      i -= 1
      if ~((i!=0) and ~(n & 0x01) and ~(n & waitIRq)):
        break

    self.ClearBitMask(self.BitFramingReg, 0x80)

    if i != 0:
      if (self.Read_MFRC522(self.ErrorReg) & 0x1B) == 0x00:
        status = self.MI_OK

        if n & irqEn & 0x01:
          status = self.MI_NOTAGERR

        if command == self.PCD_TRANSCEIVE:
          n = self.Read_MFRC522(self.FIFOLevelReg)
          lastBits = self.Read_MFRC522(self.ControlReg) & 0x07
          if lastBits != 0:
            backLen = (n-1)*8 + lastBits
          else:
            backLen = n*8

          if n == 0:
            n = 1
          elif n > self.MAX_LEN:
            n = self.MAX_LEN

          for i in range(n):
            backData.append(self.Read_MFRC522(self.FIFODataReg))
      else:
        status = self.MI_ERR

    return (status, backData, backLen)


  def MFRC522_Request(self, reqMode):
    status = None
    backBits = None
    TagType = []

    self.Write_MFRC522(self.BitFramingReg, 0x07)

    TagType.append(reqMode)
    (status, backData, backBits) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, TagType)

    if ((status != self.MI_OK) | (backBits != 0x10)):
      status = self.MI_ERR

    return (status, backBits)


  def MFRC522_Anticoll(self):
    backData = []

    self.Write_MFRC522(self.BitFramingReg, 0x00)

    serNum = [self.PICC_SEL_CL1, 0x20]

    (status, backData, backBits) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, serNum)

    if status == self.MI_OK:
      if len(backData) == 5:
        serNumCheck = 0
        for i in range(4):
          serNumCheck = serNumCheck ^ backData[i]
        if serNumCheck != backData[4]:
          status = self.MI_ERR
      else:
        status = self.MI_ERR

    return (status, backData)

  def CalulateCRC(self, pIndata):
    self.ClearBitMask(self.DivIrqReg, 0x04)
    self.SetBitMask(self.FIFOLevelReg, 0x80)
    for i in range(len(pIndata)):
      self.Write_MFRC522(self.FIFODataReg, pIndata[i])
    self.Write_MFRC522(self.CommandReg, self.PCD_CALCCRC)
    i = 0xFF
    while True:
      n = self.Read_MFRC522(self.DivIrqReg)
      i -= 1
      if not ((i != 0) and not (n & 0x04)):
        break
    pOutData = []
    pOutData.append(self.Read_MFRC522(self.CRCResultRegL))
    pOutData.append(self.Read_MFRC522(self.CRCResultRegM))
    return pOutData

  def MFRC522_SelectTag(self, serNum):
    backData = []
    buf = []
    buf.append(self.PICC_SEL_CL1)
    buf.append(0x70)
    for i in range(5):
      buf.append(serNum[i])
    pOut = self.CalulateCRC(buf)
    buf.append(pOut[0])
    buf.append(pOut[1])
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buf)

    if status == self.MI_OK and backLen == 0x18:
      print(f"Size: {backData[0]}")
      return backData[0]

    return 0

  def MFRC522_Auth(self, authMode, BlockAddr, Sectorkey, serNum):
    buf = []

    # First byte should be the authMode (A or B)
    buf.append(authMode)

    # Second byte is the trailerBlock (usually 7)
    buf.append(BlockAddr)

    # Now we need to append the authKey which usually is 6 bytes of 0xFF
    for i in range(len(Sectorkey)):
      buf.append(Sectorkey[i])

    # Next we append the first 4 bytes of the UID
    for i in range(4):
      buf.append(serNum[i])

    # Now we start the authentication itself
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_AUTHENT, buf)

    # Check if an error occurred
    if not(status == self.MI_OK):
      print("AUTH ERROR!!")
    if not (self.Read_MFRC522(self.Status2Reg) & 0x08) != 0:
      print("AUTH ERROR(status2reg & 0x08) != 0")

    # Return the status
    return status

  def MFRC522_StopCrypto1(self):
    self.ClearBitMask(self.Status2Reg, 0x08)

  def MFRC522_Read(self, blockAddr):
    recvData = []
    recvData.append(self.PICC_READ)
    recvData.append(blockAddr)
    pOut = self.CalulateCRC(recvData)
    recvData.append(pOut[0])
    recvData.append(pOut[1])
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, recvData)
    if not(status == self.MI_OK):
      print("Error while reading!")
    if len(backData) == 16:
      print("Sector {} {}".format(blockAddr, backData))

  def MFRC522_Write(self, blockAddr, writeData):
    buff = []
    buff.append(self.PICC_WRITE)
    buff.append(blockAddr)
    crc = self.CalulateCRC(buff)
    buff.append(crc[0])
    buff.append(crc[1])
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buff)
    if not(status == self.MI_OK) or not(backLen == 4) or not((backData[0] & 0x0F) == 0x0A):
        status = self.MI_ERR

    print(f"{backLen} backdata &0x0F == 0x0A {backData[0] & 0x0F}")
    if status == self.MI_OK:
        buf = []
        for i in range(16):
            buf.append(writeData[i])
        crc = self.CalulateCRC(buf)
        buf.append(crc[0])
        buf.append(crc[1])
        (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE,buf)
        if not(status == self.MI_OK) or not(backLen == 4) or not((backData[0] & 0x0F) == 0x0A):
            print("Error while writing")
        if status == self.MI_OK:
            print("Data written")

  def MFRC522_DumpClassic1K(self, key, uid):
    for i in range(64):
        status = self.MFRC522_Auth(self.PICC_AUTHENT1A, i, key, uid)
        # Check if authenticated
        if status == self.MI_OK:
            self.MFRC522_Read(i)
        else:
            print("Authentication error")

  def MFRC522_Init(self):
    GPIO.output(self.NRSTPD, 1)

    self.MFRC522_Reset()

    self.Write_MFRC522(self.TModeReg, 0x8D)
    self.Write_MFRC522(self.TPrescalerReg, 0x3E)
    self.Write_MFRC522(self.TReloadRegL, 30)
    self.Write_MFRC522(self.TReloadRegH, 0)

    self.Write_MFRC522(self.TxAutoReg, 0x40)
    self.Write_MFRC522(self.ModeReg, 0x3D)
    self.AntennaOn()
