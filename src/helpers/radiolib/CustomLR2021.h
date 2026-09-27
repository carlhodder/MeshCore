#pragma once

#include <RadioLib.h>
#include "MeshCore.h"

class CustomLR2021 : public LR2021 {
  uint32_t _preambleMillis = 66;
  uint32_t _maxPayloadMillis = 3934;
  uint32_t _activityAt = 0;
  bool _headerSeen = false;
  bool _rx_boosted = false;
  bool _pkt_interrupted = false;

  public:
    CustomLR2021(Module *mod) : LR2021(mod) { irqDioNum = LR2021_IRQ_DIO; }

    bool std_init(SPIClass* spi = NULL)
    {
      
  #ifdef LR2021_TCXO_VOLTAGE
      float tcxo = LR2021_TCXO_VOLTAGE;
  #else
      float tcxo = 1.6f;
  #endif

  #ifdef LORA_CR
      uint8_t cr = LORA_CR;
  #else
      uint8_t cr = 5;
  #endif

  #if defined(P_LORA_SCLK)
    #ifdef NRF52_PLATFORM
      if (spi) { spi->setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI); spi->begin(); }
    #elif defined(RP2040_PLATFORM)
      if (spi) {
        spi->setMISO(P_LORA_MISO);
        //spi->setCS(P_LORA_NSS); // Setting CS results in freeze
        spi->setSCK(P_LORA_SCLK);
        spi->setMOSI(P_LORA_MOSI);
        spi->begin();
      }
    #else
      if (spi) spi->begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
    #endif
  #endif
      int status = begin(LORA_FREQ, LORA_BW, LORA_SF, cr, RADIOLIB_LR2021_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo);
      // if radio init fails with -707/-706, try again with tcxo voltage set to 0.0f
      if (status == RADIOLIB_ERR_SPI_CMD_FAILED || status == RADIOLIB_ERR_SPI_CMD_INVALID) {
        tcxo = 0.0f;
        status = begin(LORA_FREQ, LORA_BW, LORA_SF, cr, RADIOLIB_LR2021_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo);
      }
      if (status != RADIOLIB_ERR_NONE) {
        Serial.print("ERROR: radio init failed: ");
        Serial.println(status);
        return false;  // fail
      }
    
      setCRC(2);
      explicitHeader();

      
    #ifdef LR2021_RX_BOOSTED_GAIN
      setRxBoostedGainMode(LR2021_RX_BOOSTED_GAIN);
    #endif

      return true;  // success
    }
    
    float getFreqMHz() const { return freqMHz; }

    bool getRxBoostedGainMode() const { return _rx_boosted; }

    int16_t startReceive() override {
      // include the PREAMBLE_DETECTED irq bit in reported flags
      return LR2021::startReceive(RADIOLIB_LR2021_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
    }

    bool isReceiving() {
      uint32_t irq = getIrqStatus();
      bool preamble = irq & RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED;  // bit 5
      bool header   = irq & RADIOLIB_LR2021_IRQ_LORA_HEADER_VALID;  // bit 6
      bool hdrErr   = irq & RADIOLIB_LR2021_IRQ_LORA_HDR_CRC_ERROR; // bit 9
      uint32_t now  = millis();
      if (hdrErr) {
        // DELETEME: This header isn't used by RadioLib or us. It has no real effect - a corrupted header means the modem can't decode the payload.
        //           Avoid a race that clears a true valid header IRQ and only use this for timing. e.g. this call can be late.
        clearIrqFlags(RADIOLIB_LR2021_IRQ_LORA_HDR_CRC_ERROR);
        _activityAt = 0;
        _headerSeen = false;
      }
      if (!header && _headerSeen) {
        // something cleared the header flag, reset our state.
        _activityAt = 0; _headerSeen = false; _pkt_interrupted = false;
        // DELETEME: Do not return here - we may have a preamble aleady detected.
      }

      if (header) {
        if (!_headerSeen) { _headerSeen = true; _activityAt = now; };
        if (now - _activityAt > _maxPayloadMillis) {
          MESH_DEBUG_PRINTLN("Clearing header IRQ after %ums", _maxPayloadMillis);
          clearIrqFlags(RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED | RADIOLIB_LR2021_IRQ_LORA_HEADER_VALID | RADIOLIB_LR2021_IRQ_LORA_HDR_CRC_ERROR);
          _activityAt = 0; _headerSeen = false;
          return false;
        }
        return true;
      }
      if (preamble) {
        if (_activityAt == 0) _activityAt = now;
        if (now - _activityAt > _preambleMillis) {
          clearIrqFlags(RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED);
          _activityAt = 0;
          MESH_DEBUG_PRINTLN("Clearing preamble IRQ after %ums", _preambleMillis);
          return false;
        }
        return true;
      }
      _activityAt = 0; _headerSeen = false;
      return false;
    }

    int16_t readData(uint8_t* data, size_t len) override {
      uint16_t fifoLevel = 0;
      getRxFifoLevel(&fifoLevel);

      // DELETEME: If there's too few bytes it could be a ghost RX_DONE (0 bytes in buffer) or so late we're mid next packet. We handle this 
      //           ourselves instead of calling readData().
      //           (The ghost RX done are incidents I see testing on all 3 LR2021 devices: valid IRQ flags, a packet length, but no data in the FIFO)
      if (fifoLevel != len) {
        if(fifoLevel >= 256) {
          // DELETEME:  An overflow stops writing new bytes, so we may have only part of the packet we think we have. Discard:
          (void)clearRxFifo();
          (void)clearIrqFlags(RADIOLIB_LR2021_IRQ_ALL);
          _pkt_interrupted = false;
          return RADIOLIB_ERR_PACKET_TOO_LONG;
        }

        if (_pkt_interrupted && fifoLevel > len) {
          // DELETEME: Second time around, we now know the data at the start is old so clear it before reading
          (void)readRadioRxFifo(data, fifoLevel - len);
        } else {
          // DELETEME: Either we're late (whole packets), mid incoming packet, or just caught a bit of a new packet on the last read between reading
          //           and the fifo clear. As we don't know if we need to discard the bytes from the start or end, we can only mark our current state
          //           and abondon the read.
          //           We also shouldn't read the current data as the packet length we have may be associated this incoming packet.
          //           (Datasheet only says you should read it directly after RX_DONE, not when it's set) even if we knew the above.
          //           
          //           We selectivley clear IRQs as the other IRQ (header valid for radiolib CRC check) could belong to this incoming packet
          //           and we need them active for the isReceiving and the readData header checks.
          //           We should be safe here - we need the settings from the (valid) header to be decoding the payload data.
          _pkt_interrupted = true;
          _activityAt = millis();
          clearIrqFlags(RADIOLIB_LR2021_IRQ_RX_DONE | RADIOLIB_LR2021_IRQ_CRC_ERROR);
          // DELETEME: Hijacked this RadioLib error code.
          return RADIOLIB_ERR_PACKET_TOO_SHORT;
        }
      }
      // DELETEME: Good read, reset overflow state.
      _pkt_interrupted = false;
      return LR2021::readData(data, len);
    }
    
    void setPreambleMillis(uint32_t preambleMillis) {
      _preambleMillis = preambleMillis;
      MESH_DEBUG_PRINTLN("Set _preambleMillis=%u", _preambleMillis);
    }
    void setMaxPayloadMillis(uint32_t payloadMillis) {
      _maxPayloadMillis = payloadMillis;
      MESH_DEBUG_PRINTLN("Set _maxPayloadMillis=%u", _maxPayloadMillis);
    }


    uint8_t getSpreadingFactor() const { return spreadingFactor; }
};