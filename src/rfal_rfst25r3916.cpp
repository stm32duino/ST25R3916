/******************************************************************************
  * \attention
  *
  * <h2><center>&copy; COPYRIGHT 2021 STMicroelectronics</center></h2>
  *
  * Licensed under ST MIX MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        www.st.com/mix_myliberty
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*! \file
 *
 *  \author SRA
 *
 *  \brief RF Abstraction Layer (RFAL)
 *
 *  RFAL implementation for ST25R3911
 */


/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "rfal_rfst25r3916.h"

/*******************************************************************************/
RfalRfST25R3916Class::RfalRfST25R3916Class(SPIClass *spi, int cs_pin, int int_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), int_pin(int_pin), spi_speed(spi_speed)
{
  memset(&gRFAL, 0, sizeof(rfal));
  memset(&gRfalAnalogConfigMgmt, 0, sizeof(rfalAnalogConfigMgmt));
  memset(&iso15693PhyConfig, 0, sizeof(iso15693PhyConfig_t));
  gST25R3916NRT_64fcs = 0;
  memset((void *)&st25r3916interrupt, 0, sizeof(st25r3916Interrupt));
  timerStopwatchTick = 0;
  i2c_enabled = false;
  dev_i2c = NULL;
  isr_pending = false;
  bus_busy = false;
  irq_handler = NULL;
}

RfalRfST25R3916Class::RfalRfST25R3916Class(TwoWire *i2c, int int_pin) : dev_i2c(i2c), int_pin(int_pin)
{
  memset(&gRFAL, 0, sizeof(rfal));
  memset(&gRfalAnalogConfigMgmt, 0, sizeof(rfalAnalogConfigMgmt));
  memset(&iso15693PhyConfig, 0, sizeof(iso15693PhyConfig_t));
  gST25R3916NRT_64fcs = 0;
  memset((void *)&st25r3916interrupt, 0, sizeof(st25r3916Interrupt));
  timerStopwatchTick = 0;
  i2c_enabled = true;
  dev_spi = NULL;
  isr_pending = false;
  bus_busy = false;
  irq_handler = NULL;
}


ReturnCode RfalRfST25R3916Class::rfalInitialize(void)
{
  ReturnCode err;

  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);

  pinMode(int_pin, INPUT);
  Callback<void()>::func = std::bind(&RfalRfST25R3916Class::st25r3916Isr, this);
  irq_handler = static_cast<ST25R3916IrqHandler>(Callback<void()>::callback);
  attachInterrupt(int_pin, irq_handler, RISING);

  rfalAnalogConfigInitialize();              /* Initialize RFAL's Analog Configs */

  EXIT_ON_ERR(err, st25r3916Initialize());

  st25r3916ClearInterrupts();

  /* Disable any previous observation mode */
  rfalST25R3916ObsModeDisable();

  /*******************************************************************************/
  /* Apply RF Chip generic initialization */
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_INIT));


  /*******************************************************************************/
  /* Enable External Field Detector as: Automatics */
  st25r3916ChangeRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_en_fd_mask, ST25R3916_REG_OP_CONTROL_en_fd_auto_efd);

  /* Clear FIFO status local copy */
  rfalFIFOStatusClear();

  /*******************************************************************************/
  gRFAL.state              = RFAL_STATE_INIT;
  gRFAL.mode               = RFAL_MODE_NONE;
  gRFAL.field              = false;

  /* Set RFAL default configs */
  gRFAL.conf.obsvModeRx    = RFAL_OBSMODE_DISABLE;
  gRFAL.conf.obsvModeTx    = RFAL_OBSMODE_DISABLE;
  gRFAL.conf.eHandling     = RFAL_ERRORHANDLING_NONE;

  /* Transceive set to IDLE */
  gRFAL.TxRx.lastState     = RFAL_TXRX_STATE_IDLE;
  gRFAL.TxRx.state         = RFAL_TXRX_STATE_IDLE;

  /* Disable all timings */
  gRFAL.timings.FDTListen  = RFAL_TIMING_NONE;
  gRFAL.timings.FDTPoll    = RFAL_TIMING_NONE;
  gRFAL.timings.GT         = RFAL_TIMING_NONE;

  gRFAL.tmr.GT             = RFAL_TIMING_NONE;

  gRFAL.callbacks.preTxRx  = NULL;
  gRFAL.callbacks.postTxRx = NULL;

  /* Initialize NFC-V Data */
  gRFAL.nfcvData.ignoreBits = 0;

  /* Initialize Wake-Up Mode */
  gRFAL.wum.state = RFAL_WUM_STATE_NOT_INIT;

  /*******************************************************************************/
  /* Perform Automatic Calibration (if configured to do so).                     *
   * Registers set by rfalSetAnalogConfig will tell rfalCalibrate what to perform*/
  rfalCalibrate();

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalCalibrate(void)
{
  uint16_t resValue;

  /* Check if RFAL is not initialized */
  if (gRFAL.state == RFAL_STATE_IDLE) {
    return ERR_WRONG_STATE;
  }

  /*******************************************************************************/
  /* Perform ST25R3916 regulators and antenna calibration                        */
  /*******************************************************************************/

  /* Automatic regulator adjustment only performed if not set manually on Analog Configs */
  if (st25r3916CheckReg(ST25R3916_REG_REGULATOR_CONTROL, ST25R3916_REG_REGULATOR_CONTROL_reg_s, 0x00)) {
    /* Adjust the regulators so that Antenna Calibrate has better Regulator values */
    st25r3916AdjustRegulators(&resValue);
  }

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalAdjustRegulators(uint16_t *result)
{
  return st25r3916AdjustRegulators(result);
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalSetUpperLayerCallback(rfalUpperLayerCallback pFunc)
{
  st25r3916IRQCallbackSet(pFunc);
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalSetPreTxRxCallback(rfalPreTxRxCallback pFunc)
{
  gRFAL.callbacks.preTxRx = pFunc;
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalSetPostTxRxCallback(rfalPostTxRxCallback pFunc)
{
  gRFAL.callbacks.postTxRx = pFunc;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalDeinitialize(void)
{
  /* Deinitialize chip */
  st25r3916Deinitialize();

  /* Set Analog configurations for deinitialization */
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_DEINIT));

  gRFAL.state = RFAL_STATE_IDLE;

  detachInterrupt(int_pin);
  irq_handler = NULL;

  return ERR_NONE;
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalSetObsvMode(uint8_t txMode, uint8_t rxMode)
{
  gRFAL.conf.obsvModeTx = txMode;
  gRFAL.conf.obsvModeRx = rxMode;
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalGetObsvMode(uint8_t *txMode, uint8_t *rxMode)
{
  if (txMode != NULL) {
    *txMode = gRFAL.conf.obsvModeTx;
  }

  if (rxMode != NULL) {
    *rxMode = gRFAL.conf.obsvModeRx;
  }
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalDisableObsvMode(void)
{
  gRFAL.conf.obsvModeTx = RFAL_OBSMODE_DISABLE;
  gRFAL.conf.obsvModeRx = RFAL_OBSMODE_DISABLE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalSetMode(rfalMode mode, rfalBitRate txBR, rfalBitRate rxBR)
{
  /* Check if RFAL is not initialized */
  if (gRFAL.state == RFAL_STATE_IDLE) {
    return ERR_WRONG_STATE;
  }

  /* Check allowed bit rate value */
  if ((txBR == RFAL_BR_KEEP) || (rxBR == RFAL_BR_KEEP)) {
    return ERR_PARAM;
  }

  switch (mode) {
    /*******************************************************************************/
    case RFAL_MODE_POLL_NFCA:
      /* Disable wake up mode, if set */
      st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_wu);

      /* Enable ISO14443A mode */
      st25r3916WriteRegister(ST25R3916_REG_MODE, ST25R3916_REG_MODE_om_iso14443a);

      /* Set Analog configurations for this mode and bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_POLL_NFCA_T1T:
      /* Disable wake up mode, if set */
      st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_wu);

      /* Enable Topaz mode */
      st25r3916WriteRegister(ST25R3916_REG_MODE, ST25R3916_REG_MODE_om_topaz);

      /* Set Analog configurations for this mode and bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_POLL_NFCB:
      /* Disable wake up mode, if set */
      st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_wu);

      /* Enable ISO14443B mode */
      st25r3916WriteRegister(ST25R3916_REG_MODE, ST25R3916_REG_MODE_om_iso14443b);

      /* Set the EGT, SOF, EOF and EOF */
      st25r3916ChangeRegisterBits(ST25R3916_REG_ISO14443B_1,
                                  (ST25R3916_REG_ISO14443B_1_egt_mask | ST25R3916_REG_ISO14443B_1_sof_mask | ST25R3916_REG_ISO14443B_1_eof),
                                  ((0U << ST25R3916_REG_ISO14443B_1_egt_shift) | ST25R3916_REG_ISO14443B_1_sof_0_10etu | ST25R3916_REG_ISO14443B_1_sof_1_2etu | ST25R3916_REG_ISO14443B_1_eof_10etu));

      /* Set the minimum TR1, SOF, EOF and EOF12 */
      st25r3916ChangeRegisterBits(ST25R3916_REG_ISO14443B_2,
                                  (ST25R3916_REG_ISO14443B_2_tr1_mask | ST25R3916_REG_ISO14443B_2_no_sof | ST25R3916_REG_ISO14443B_2_no_eof),
                                  (ST25R3916_REG_ISO14443B_2_tr1_80fs80fs));

      /* Set Analog configurations for this mode and bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_POLL_B_PRIME:
      /* Disable wake up mode, if set */
      st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_wu);

      /* Enable ISO14443B mode */
      st25r3916WriteRegister(ST25R3916_REG_MODE, ST25R3916_REG_MODE_om_iso14443b);

      /* Set the EGT, SOF, EOF and EOF */
      st25r3916ChangeRegisterBits(ST25R3916_REG_ISO14443B_1,
                                  (ST25R3916_REG_ISO14443B_1_egt_mask | ST25R3916_REG_ISO14443B_1_sof_mask | ST25R3916_REG_ISO14443B_1_eof),
                                  ((0U << ST25R3916_REG_ISO14443B_1_egt_shift) | ST25R3916_REG_ISO14443B_1_sof_0_10etu | ST25R3916_REG_ISO14443B_1_sof_1_2etu | ST25R3916_REG_ISO14443B_1_eof_10etu));

      /* Set the minimum TR1, EOF and EOF12 */
      st25r3916ChangeRegisterBits(ST25R3916_REG_ISO14443B_2,
                                  (ST25R3916_REG_ISO14443B_2_tr1_mask | ST25R3916_REG_ISO14443B_2_no_sof | ST25R3916_REG_ISO14443B_2_no_eof),
                                  (ST25R3916_REG_ISO14443B_2_tr1_80fs80fs | ST25R3916_REG_ISO14443B_2_no_sof));

      /* Set Analog configurations for this mode and bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_POLL_B_CTS:
      /* Disable wake up mode, if set */
      st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_wu);

      /* Enable ISO14443B mode */
      st25r3916WriteRegister(ST25R3916_REG_MODE, ST25R3916_REG_MODE_om_iso14443b);

      /* Set the EGT, SOF, EOF and EOF */
      st25r3916ChangeRegisterBits(ST25R3916_REG_ISO14443B_1,
                                  (ST25R3916_REG_ISO14443B_1_egt_mask | ST25R3916_REG_ISO14443B_1_sof_mask | ST25R3916_REG_ISO14443B_1_eof),
                                  ((0U << ST25R3916_REG_ISO14443B_1_egt_shift) | ST25R3916_REG_ISO14443B_1_sof_0_10etu | ST25R3916_REG_ISO14443B_1_sof_1_2etu | ST25R3916_REG_ISO14443B_1_eof_10etu));

      /* Set the minimum TR1, clear SOF, EOF and EOF12 */
      st25r3916ChangeRegisterBits(ST25R3916_REG_ISO14443B_2,
                                  (ST25R3916_REG_ISO14443B_2_tr1_mask | ST25R3916_REG_ISO14443B_2_no_sof | ST25R3916_REG_ISO14443B_2_no_eof),
                                  (ST25R3916_REG_ISO14443B_2_tr1_80fs80fs | ST25R3916_REG_ISO14443B_2_no_sof | ST25R3916_REG_ISO14443B_2_no_eof));

      /* Set Analog configurations for this mode and bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_POLL_NFCF:
      /* Disable wake up mode, if set */
      st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_wu);

      /* Enable FeliCa mode */
      st25r3916WriteRegister(ST25R3916_REG_MODE, ST25R3916_REG_MODE_om_felica);

      /* Set Analog configurations for this mode and bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_POLL_NFCV:
    case RFAL_MODE_POLL_PICOPASS:
      /* Disable wake up mode, if set */
      st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_wu);

      /* Set Analog configurations for this mode and bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_POLL_ACTIVE_P2P:
      /* Set NFCIP1 active communication Initiator mode and Automatic Response RF Collision Avoidance to always after EOF */
      st25r3916WriteRegister(ST25R3916_REG_MODE, (ST25R3916_REG_MODE_targ_init | ST25R3916_REG_MODE_om_nfc | ST25R3916_REG_MODE_nfc_ar_eof));

      /* External Field Detector enabled as Automatics on rfalInitialize() */

      /* Set NRT to start at end of TX (own) field */
      st25r3916ChangeRegisterBits(ST25R3916_REG_TIMER_EMV_CONTROL, ST25R3916_REG_TIMER_EMV_CONTROL_nrt_nfc, ST25R3916_REG_TIMER_EMV_CONTROL_nrt_nfc_off);

      /* Set GPT to start after end of TX, as GPT is used in active communication mode to timeout the field switching off */
      /* The field is turned off 37.76us after the end of the transmission  Trfw                                          */
      st25r3916SetStartGPTimer((uint16_t)rfalConv1fcTo8fc(RFAL_AP2P_FIELDOFF_TRFW), ST25R3916_REG_TIMER_EMV_CONTROL_gptc_etx_nfc);

      /* Set PPon2 timer with the max time between our field Off and other peer field On : Tadt + (n x Trfw)    */
      st25r3916WriteRegister(ST25R3916_REG_PPON2, (uint8_t)rfalConv1fcTo64fc(RFAL_AP2P_FIELDON_TADTTRFW));

      /* Set Analog configurations for this mode and bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_LISTEN_ACTIVE_P2P:
      /* Set NFCIP1 active communication Target mode and Automatic Response RF Collision Avoidance to always after EOF */
      st25r3916WriteRegister(ST25R3916_REG_MODE, (ST25R3916_REG_MODE_targ_targ | ST25R3916_REG_MODE_om_targ_nfcip | ST25R3916_REG_MODE_nfc_ar_eof));

      /* External Field Detector enabled as Automatics on rfalInitialize() */

      /* Set NRT to start at end of TX (own) field */
      st25r3916ChangeRegisterBits(ST25R3916_REG_TIMER_EMV_CONTROL, ST25R3916_REG_TIMER_EMV_CONTROL_nrt_nfc, ST25R3916_REG_TIMER_EMV_CONTROL_nrt_nfc_off);

      /* Set GPT to start after end of TX, as GPT is used in active communication mode to timeout the field switching off */
      /* The field is turned off 37.76us after the end of the transmission  Trfw                                          */
      st25r3916SetStartGPTimer((uint16_t)rfalConv1fcTo8fc(RFAL_AP2P_FIELDOFF_TRFW), ST25R3916_REG_TIMER_EMV_CONTROL_gptc_etx_nfc);

      /* Set PPon2 timer with the max time between our field Off and other peer field On : Tadt + (n x Trfw)    */
      st25r3916WriteRegister(ST25R3916_REG_PPON2, (uint8_t)rfalConv1fcTo64fc(RFAL_AP2P_FIELDON_TADTTRFW));

      /* Set Analog configurations for this mode and bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_LISTEN_NFCA:
      /* Disable wake up mode, if set */
      st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_wu);

      /* Enable Passive Target NFC-A mode, disable any Collision Avoidance */
      st25r3916WriteRegister(ST25R3916_REG_MODE, (ST25R3916_REG_MODE_targ | ST25R3916_REG_MODE_om_targ_nfca | ST25R3916_REG_MODE_nfc_ar_off));

      /* Set Analog configurations for this mode */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_LISTEN_NFCF:
      /* Disable wake up mode, if set */
      st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_wu);

      /* Enable Passive Target NFC-F mode, disable any Collision Avoidance */
      st25r3916WriteRegister(ST25R3916_REG_MODE, (ST25R3916_REG_MODE_targ | ST25R3916_REG_MODE_om_targ_nfcf | ST25R3916_REG_MODE_nfc_ar_off));

      /* Set Analog configurations for this mode */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_LISTEN_NFCB:
      return ERR_NOTSUPP;

    /*******************************************************************************/
    default:
      return ERR_NOT_IMPLEMENTED;
  }

  /* Set state as STATE_MODE_SET only if not initialized yet (PSL) */
  gRFAL.state = ((gRFAL.state < RFAL_STATE_MODE_SET) ? RFAL_STATE_MODE_SET : gRFAL.state);
  gRFAL.mode  = mode;

  /* Apply the given bit rate */
  return rfalSetBitRate(txBR, rxBR);
}


/*******************************************************************************/
rfalMode RfalRfST25R3916Class::rfalGetMode(void)
{
  return gRFAL.mode;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalSetBitRate(rfalBitRate txBR, rfalBitRate rxBR)
{
  ReturnCode ret;

  /* Check if RFAL is not initialized */
  if (gRFAL.state == RFAL_STATE_IDLE) {
    return ERR_WRONG_STATE;
  }

  /* Store the new Bit Rates */
  gRFAL.txBR = ((txBR == RFAL_BR_KEEP) ? gRFAL.txBR : txBR);
  gRFAL.rxBR = ((rxBR == RFAL_BR_KEEP) ? gRFAL.rxBR : rxBR);

  /* Update the bitrate reg if not in NFCV mode (streaming) */
  if ((RFAL_MODE_POLL_NFCV != gRFAL.mode) && (RFAL_MODE_POLL_PICOPASS != gRFAL.mode)) {
    /* Set bit rate register */
    EXIT_ON_ERR(ret, st25r3916SetBitrate((uint8_t)gRFAL.txBR, (uint8_t)gRFAL.rxBR));
  }


  switch (gRFAL.mode) {
    /*******************************************************************************/
    case RFAL_MODE_POLL_NFCA:
    case RFAL_MODE_POLL_NFCA_T1T:
      /* Set Analog configurations for this bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_POLL_COMMON));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_POLL_NFCB:
    case RFAL_MODE_POLL_B_PRIME:
    case RFAL_MODE_POLL_B_CTS:
      /* Set Analog configurations for this bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_POLL_COMMON));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_POLL_NFCF:
      /* Set Analog configurations for this bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_POLL_COMMON));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_POLL_NFCV:
    case RFAL_MODE_POLL_PICOPASS:
      if (((gRFAL.rxBR != RFAL_BR_26p48) && (gRFAL.rxBR != RFAL_BR_52p97) && (gRFAL.rxBR != RFAL_BR_106) && (gRFAL.rxBR != RFAL_BR_212))
          || ((gRFAL.txBR != RFAL_BR_1p66) && (gRFAL.txBR != RFAL_BR_26p48))) {
        return ERR_PARAM;
      }

      {
        const struct iso15693StreamConfig *isoStreamConfig;
        struct st25r3916StreamConfig      streamConf;
        iso15693PhyConfig_t                config;

        config.coding     = ((gRFAL.txBR == RFAL_BR_1p66) ? ISO15693_VCD_CODING_1_256 : ISO15693_VCD_CODING_1_4);
        switch (gRFAL.rxBR) {
          case RFAL_BR_52p97:
            config.speedMode = 1;
            break;
          case RFAL_BR_106:
            config.speedMode = 2;
            break;
          case RFAL_BR_212:
            config.speedMode = 3;
            break;
          default:
            config.speedMode = 0;
            break;
        }

        iso15693PhyConfigure(&config, &isoStreamConfig);

        /* MISRA 11.3 - Cannot point directly into different object type, copy to local var */
        streamConf.din                  = isoStreamConfig->din;
        streamConf.dout                 = isoStreamConfig->dout;
        streamConf.report_period_length = isoStreamConfig->report_period_length;
        streamConf.useBPSK              = isoStreamConfig->useBPSK;
        st25r3916StreamConfigure(&streamConf);
      }

      /* Set Analog configurations for this bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_POLL_COMMON));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_POLL_ACTIVE_P2P:
      /* Set Analog configurations for this bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_POLL_COMMON));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_LISTEN_ACTIVE_P2P:
      /* Set Analog configurations for this bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_LISTEN_COMMON));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_LISTEN_NFCA:
      /* Set Analog configurations for this bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_LISTEN_COMMON));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_NFCA | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_NFCA | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_LISTEN_NFCF:
      /* Set Analog configurations for this bit rate */
      rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_LISTEN_COMMON));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_NFCF | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX));
      rfalSetAnalogConfig((rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_NFCF | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX));
      break;

    /*******************************************************************************/
    case RFAL_MODE_LISTEN_NFCB:
    case RFAL_MODE_NONE:
      return ERR_WRONG_STATE;

    /*******************************************************************************/
    default:
      return ERR_NOT_IMPLEMENTED;
  }

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalGetBitRate(rfalBitRate *txBR, rfalBitRate *rxBR)
{
  if ((gRFAL.state == RFAL_STATE_IDLE) || (gRFAL.mode == RFAL_MODE_NONE)) {
    return ERR_WRONG_STATE;
  }

  if (txBR != NULL) {
    *txBR = gRFAL.txBR;
  }

  if (rxBR != NULL) {
    *rxBR = gRFAL.rxBR;
  }

  return ERR_NONE;
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalSetErrorHandling(rfalEHandling eHandling)
{
  switch (eHandling) {
    case RFAL_ERRORHANDLING_NFC:
    case RFAL_ERRORHANDLING_NONE:
      st25r3916ClrRegisterBits(ST25R3916_REG_EMD_SUP_CONF, ST25R3916_REG_EMD_SUP_CONF_emd_emv);
      break;

    case RFAL_ERRORHANDLING_EMVCO:
      /* MISRA 16.4: no empty default statement (in case RFAL_SW_EMD is defined) */
      st25r3916ModifyRegister(ST25R3916_REG_EMD_SUP_CONF,
                              (ST25R3916_REG_EMD_SUP_CONF_emd_emv | ST25R3916_REG_EMD_SUP_CONF_emd_crc_prot | ST25R3916_REG_EMD_SUP_CONF_emd_res_bits | ST25R3916_REG_EMD_SUP_CONF_emd_thld_mask),
                              (ST25R3916_REG_EMD_SUP_CONF_emd_emv_on | ST25R3916_REG_EMD_SUP_CONF_emd_crc_prot_off | ST25R3916_REG_EMD_SUP_CONF_emd_res_bits_off | RFAL_EMVCO_RX_MAXLEN));
      break;
    default:
      /* MISRA 16.4: no empty default statement (a comment being enough) */
      break;
  }

  gRFAL.conf.eHandling = eHandling;
}


/*******************************************************************************/
rfalEHandling RfalRfST25R3916Class::rfalGetErrorHandling(void)
{
  return gRFAL.conf.eHandling;
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalSetFDTPoll(uint32_t FDTPoll)
{
  gRFAL.timings.FDTPoll = MIN(FDTPoll, RFAL_ST25R3916_GPT_MAX_1FC);
}


/*******************************************************************************/
uint32_t RfalRfST25R3916Class::rfalGetFDTPoll(void)
{
  return gRFAL.timings.FDTPoll;
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalSetFDTListen(uint32_t FDTListen)
{
  gRFAL.timings.FDTListen = MIN(FDTListen, RFAL_ST25R3916_MRT_MAX_1FC);
}

/*******************************************************************************/
uint32_t RfalRfST25R3916Class::rfalGetFDTListen(void)
{
  return gRFAL.timings.FDTListen;
}

void RfalRfST25R3916Class::rfalSetGT(uint32_t GT)
{
  gRFAL.timings.GT = MIN(GT, RFAL_ST25R3916_GT_MAX_1FC);
}

/*******************************************************************************/
uint32_t RfalRfST25R3916Class::rfalGetGT(void)
{
  return gRFAL.timings.GT;
}

/*******************************************************************************/
bool RfalRfST25R3916Class::rfalIsGTExpired(void)
{
  if (gRFAL.tmr.GT != RFAL_TIMING_NONE) {
    if (!rfalTimerisExpired(gRFAL.tmr.GT)) {
      return false;
    }
  }
  return true;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalFieldOnAndStartGT(void)
{
  ReturnCode ret;

  /* Check if RFAL has been initialized (Oscillator should be running) and also
   * if a direct register access has been performed and left the Oscillator Off */
  if (!st25r3916IsOscOn() || (gRFAL.state < RFAL_STATE_INIT)) {
    return ERR_WRONG_STATE;
  }

  ret = ERR_NONE;

  /* Set Analog configurations for Field On event */
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_FIELD_ON));

  /*******************************************************************************/
  /* Perform collision avoidance and turn field On if not already On */
  if (!st25r3916IsTxEnabled() || !gRFAL.field) {
    /* Set TARFG: 0 (75us+0ms=75us), GT is fulfilled using a SW timer */
    st25r3916WriteRegister(ST25R3916_REG_FIELD_ON_GT, 0U);

    /* Use Thresholds set by AnalogConfig */
    ret = st25r3916PerformCollisionAvoidance(ST25R3916_CMD_INITIAL_RF_COLLISION, ST25R3916_THRESHOLD_DO_NOT_SET, ST25R3916_THRESHOLD_DO_NOT_SET, 0);

    gRFAL.field = st25r3916IsTxEnabled(); //(ret == ERR_NONE);

    /* Only turn on Receiver and Transmitter if field was successfully turned On */
    if (gRFAL.field) {
      st25r3916TxRxOn(); /* Enable Tx and Rx (Tx is already On)*/
    }
  }

  /*******************************************************************************/
  /* Start GT timer in case the GT value is set */
  if ((gRFAL.timings.GT != RFAL_TIMING_NONE)) {
    /* Ensure that a SW timer doesn't have a lower value then the minimum  */
    rfalTimerStart(gRFAL.tmr.GT, rfalConv1fcToMs(MAX((gRFAL.timings.GT), RFAL_ST25R3916_GT_MIN_1FC)));
  }

  return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalFieldOff(void)
{
  /* Check whether a TxRx is not yet finished */
  if (gRFAL.TxRx.state != RFAL_TXRX_STATE_IDLE) {
    rfalCleanupTransceive();
  }

  /* Disable Tx and Rx */
  st25r3916TxRxOff();

  /* Set Analog configurations for Field Off event */
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_FIELD_OFF));
  gRFAL.field = false;

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalStartTransceive(const rfalTransceiveContext *ctx)
{
  uint32_t FxTAdj;  /* FWT or FDT adjustment calculation */

  /* Check for valid parameters */
  if (ctx == NULL) {
    return ERR_PARAM;
  }

  /* Ensure that RFAL is already Initialized and the mode has been set */
  if ((gRFAL.state >= RFAL_STATE_MODE_SET) /*&& (gRFAL.TxRx.state == RFAL_TXRX_STATE_INIT )*/) {
    /*******************************************************************************/
    /* Check whether the field is already On, otherwise no TXE will be received  */
    if (!st25r3916IsTxEnabled() && (!rfalIsModePassiveListen(gRFAL.mode) && (ctx->txBuf != NULL))) {
      return ERR_WRONG_STATE;
    }

    gRFAL.TxRx.ctx = *ctx;

    /*******************************************************************************/
    if (gRFAL.timings.FDTListen != RFAL_TIMING_NONE) {
      /* Calculate MRT adjustment accordingly to the current mode */
      FxTAdj = RFAL_FDT_LISTEN_MRT_ADJUSTMENT;
      if (gRFAL.mode == RFAL_MODE_POLL_NFCA)      {
        FxTAdj += (uint32_t)RFAL_FDT_LISTEN_A_ADJUSTMENT;
      }
      if (gRFAL.mode == RFAL_MODE_POLL_NFCA_T1T)  {
        FxTAdj += (uint32_t)RFAL_FDT_LISTEN_A_ADJUSTMENT;
      }
      if (gRFAL.mode == RFAL_MODE_POLL_NFCB)      {
        FxTAdj += (uint32_t)RFAL_FDT_LISTEN_B_ADJUSTMENT;
      }
      if (gRFAL.mode == RFAL_MODE_POLL_NFCV)      {
        FxTAdj += (uint32_t)RFAL_FDT_LISTEN_V_ADJUSTMENT;
      }

      /* Ensure that MRT is using 64/fc steps */
      st25r3916ClrRegisterBits(ST25R3916_REG_TIMER_EMV_CONTROL, ST25R3916_REG_TIMER_EMV_CONTROL_mrt_step);

      /* If Correlator is being used further adjustment is required for NFCB */
      if ((st25r3916CheckReg(ST25R3916_REG_AUX, ST25R3916_REG_AUX_dis_corr, 0x00U)) && (gRFAL.mode == RFAL_MODE_POLL_NFCB)) {
        FxTAdj += (uint32_t)RFAL_FDT_LISTEN_B_ADJT_CORR;                                                                                        /* Reduce FDT(Listen)                   */
        st25r3916SetRegisterBits(ST25R3916_REG_CORR_CONF1, ST25R3916_REG_CORR_CONF1_corr_s3);                                                   /* Ensure BPSK start to 33 pilot pulses */
        st25r3916ChangeRegisterBits(ST25R3916_REG_SUBC_START_TIME, ST25R3916_REG_SUBC_START_TIME_sst_mask, RFAL_FDT_LISTEN_B_ADJT_CORR_SST);    /* Set sst                              */
      }

      /* Set Minimum FDT(Listen) in which PICC is not allowed to send a response */
      st25r3916WriteRegister(ST25R3916_REG_MASK_RX_TIMER, (uint8_t)rfalConv1fcTo64fc((FxTAdj > gRFAL.timings.FDTListen) ? RFAL_ST25R3916_MRT_MIN_1FC : (gRFAL.timings.FDTListen - FxTAdj)));
    }

    /*******************************************************************************/
    /* FDT Poll will be loaded in rfalPrepareTransceive() once the previous was expired */

    /*******************************************************************************/
    if ((gRFAL.TxRx.ctx.fwt != RFAL_FWT_NONE) && (gRFAL.TxRx.ctx.fwt != 0U)) {
      /* Ensure proper timing configuration */
      if (gRFAL.timings.FDTListen >= gRFAL.TxRx.ctx.fwt) {
        return ERR_PARAM;
      }

      FxTAdj = RFAL_FWT_ADJUSTMENT;
      if (gRFAL.mode == RFAL_MODE_POLL_NFCA)      {
        FxTAdj += (uint32_t)RFAL_FWT_A_ADJUSTMENT;
      }
      if (gRFAL.mode == RFAL_MODE_POLL_NFCA_T1T)  {
        FxTAdj += (uint32_t)RFAL_FWT_A_ADJUSTMENT;
      }
      if (gRFAL.mode == RFAL_MODE_POLL_NFCB)      {
        FxTAdj += (uint32_t)RFAL_FWT_B_ADJUSTMENT;
      }
      if (gRFAL.mode == RFAL_MODE_POLL_NFCF) {
        FxTAdj += (uint32_t)((gRFAL.txBR == RFAL_BR_212) ? RFAL_FWT_F_212_ADJUSTMENT : RFAL_FWT_F_424_ADJUSTMENT);
      }

      /* Ensure that the given FWT doesn't exceed NRT maximum */
      gRFAL.TxRx.ctx.fwt = MIN((gRFAL.TxRx.ctx.fwt + FxTAdj), RFAL_ST25R3916_NRT_MAX_1FC);

      /* Set FWT in the NRT */
      st25r3916SetNoResponseTime(rfalConv1fcTo64fc(gRFAL.TxRx.ctx.fwt));
    } else {
      /* Disable NRT, no NRE will be triggered, therefore wait endlessly for Rx */
      st25r3916SetNoResponseTime(RFAL_ST25R3916_NRT_DISABLED);
    }

    gRFAL.state       = RFAL_STATE_TXRX;
    gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_IDLE;
    gRFAL.TxRx.status = ERR_BUSY;

    /*******************************************************************************/
    if ((RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode)) {
      /* Exchange receive buffer with internal buffer */
      gRFAL.nfcvData.origCtx = gRFAL.TxRx.ctx;

      gRFAL.TxRx.ctx.rxBuf    = ((gRFAL.nfcvData.origCtx.rxBuf != NULL) ? gRFAL.nfcvData.codingBuffer : NULL);
      gRFAL.TxRx.ctx.rxBufLen = (uint16_t)rfalConvBytesToBits(sizeof(gRFAL.nfcvData.codingBuffer));
      gRFAL.TxRx.ctx.flags = (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL
                             | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP
                             | (uint32_t)RFAL_TXRX_FLAGS_NFCIP1_OFF
                             | (uint32_t)(gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_AGC_OFF)
                             | (uint32_t)RFAL_TXRX_FLAGS_PAR_RX_KEEP
                             | (uint32_t)RFAL_TXRX_FLAGS_PAR_TX_NONE;

      /* In NFCV a TxRx with a valid txBuf and txBufSize==0 indicates to send an EOF */
      /* Skip logic below that would go directly into receive                        */
      if (gRFAL.TxRx.ctx.txBuf != NULL) {
        return  ERR_NONE;
      }
    }

    /*******************************************************************************/
    /* Check if the Transceive start performing Tx or goes directly to Rx          */
    if ((gRFAL.TxRx.ctx.txBuf == NULL) || (gRFAL.TxRx.ctx.txBufLen == 0U)) {
      /* Clear FIFO, Clear and Enable the Interrupts */
      rfalPrepareTransceive();

      /* Disable our field upon a Rx re-enable on AP2P */
      if (rfalIsModeActiveComm(gRFAL.mode)) {
        st25r3916TxOff();
      }

      /* No Tx done, enable the Receiver */
      st25r3916ExecuteCommand(ST25R3916_CMD_UNMASK_RECEIVE_DATA);

      /* Start NRT manually, if FWT = 0 (wait endlessly for Rx) chip will ignore anyhow */
      st25r3916ExecuteCommand(ST25R3916_CMD_START_NO_RESPONSE_TIMER);

      gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_IDLE;
    }

    return ERR_NONE;
  }

  return ERR_WRONG_STATE;
}


/*******************************************************************************/
bool RfalRfST25R3916Class::rfalIsTransceiveInTx(void)
{
  return ((gRFAL.TxRx.state >= RFAL_TXRX_STATE_TX_IDLE) && (gRFAL.TxRx.state < RFAL_TXRX_STATE_RX_IDLE));
}


/*******************************************************************************/
bool RfalRfST25R3916Class::rfalIsTransceiveInRx(void)
{
  return (gRFAL.TxRx.state >= RFAL_TXRX_STATE_RX_IDLE);
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalTransceiveBlockingTx(uint8_t *txBuf, uint16_t txBufLen, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *actLen, uint32_t flags, uint32_t fwt)
{
  ReturnCode               ret;
  rfalTransceiveContext    ctx;

  rfalCreateByteFlagsTxRxContext(ctx, txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt);
  EXIT_ON_ERR(ret, rfalStartTransceive(&ctx));

  return rfalTransceiveRunBlockingTx();
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalTransceiveRunBlockingTx(void)
{
  ReturnCode ret;

  do {
    rfalWorker();
    ret = rfalGetTransceiveStatus();
  } while (rfalIsTransceiveInTx() && (ret == ERR_BUSY));

  if (rfalIsTransceiveInRx()) {
    return ERR_NONE;
  }

  return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalTransceiveBlockingRx(void)
{
  ReturnCode ret;

  do {
    rfalWorker();
    ret = rfalGetTransceiveStatus();
  } while (rfalIsTransceiveInRx() && (ret == ERR_BUSY));

  return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalTransceiveBlockingTxRx(uint8_t *txBuf, uint16_t txBufLen, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *actLen, uint32_t flags, uint32_t fwt)
{
  ReturnCode ret;

  EXIT_ON_ERR(ret, rfalTransceiveBlockingTx(txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt));
  ret = rfalTransceiveBlockingRx();

  /* Convert received bits to bytes */
  if (actLen != NULL) {
    *actLen = rfalConvBitsToBytes(*actLen);
  }

  return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalRunTransceiveWorker(void)
{
  if (gRFAL.state == RFAL_STATE_TXRX) {
    /* Run Tx or Rx state machines */
    if (rfalIsTransceiveInTx()) {
      rfalTransceiveTx();
      return rfalGetTransceiveStatus();
    }
    if (rfalIsTransceiveInRx()) {
      rfalTransceiveRx();
      return rfalGetTransceiveStatus();
    }
  }
  return ERR_WRONG_STATE;
}

/*******************************************************************************/
rfalTransceiveState RfalRfST25R3916Class::rfalGetTransceiveState(void)
{
  return gRFAL.TxRx.state;
}

ReturnCode RfalRfST25R3916Class::rfalGetTransceiveStatus(void)
{
  return ((gRFAL.TxRx.state == RFAL_TXRX_STATE_IDLE) ? gRFAL.TxRx.status : ERR_BUSY);
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalGetTransceiveRSSI(uint16_t *rssi)
{
  uint16_t amRSSI;
  uint16_t pmRSSI;
  bool     isSumMode;

  if (rssi == NULL) {
    return ERR_PARAM;
  }

  st25r3916GetRSSI(&amRSSI, &pmRSSI);

  /* Check if Correlator Summation mode is being used */
  isSumMode = (st25r3916CheckReg(ST25R3916_REG_CORR_CONF1, ST25R3916_REG_CORR_CONF1_corr_s4, ST25R3916_REG_CORR_CONF1_corr_s4) ? st25r3916CheckReg(ST25R3916_REG_AUX, ST25R3916_REG_AUX_dis_corr, 0x00) : false);
  if (isSumMode) {
    /*******************************************************************************/
    /* Using SQRT from math.h and float. If due to compiler, resources or performance
     * issue this cannot be used, other approaches can be foreseen with less accuracy:
     *    Use a simpler sqrt algorithm
     *    *rssi = MAX( amRSSI, pmRSSI );
     *    *rssi = ( (amRSSI + pmRSSI) / 2);
     */
    *rssi = (uint16_t) sqrt(((double)amRSSI * (double)amRSSI) + ((double)pmRSSI * (double)pmRSSI));             /*  PRQA S 5209 # MISRA 4.9 - External function (sqrt()) requires double */
  } else {
    /* Check which channel was used */
    *rssi = (st25r3916CheckReg(ST25R3916_REG_AUX_DISPLAY, ST25R3916_REG_AUX_DISPLAY_a_cha, ST25R3916_REG_AUX_DISPLAY_a_cha) ? pmRSSI : amRSSI);
  }
  return ERR_NONE;
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalWorker(void)
{
  switch (gRFAL.state) {
    case RFAL_STATE_TXRX:
      rfalRunTransceiveWorker();
      break;

    case RFAL_STATE_WUM:
      rfalRunWakeUpModeWorker();
      break;

    /* Nothing to be done */
    default:
      /* MISRA 16.4: no empty default statement (a comment being enough) */
      break;
  }
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalErrorHandling(void)
{
  uint16_t fifoBytesToRead;

  fifoBytesToRead = rfalFIFOStatusGetNumBytes();

  /*******************************************************************************/
  /* ISO14443A Mode                                                              */
  /*******************************************************************************/
  if (gRFAL.mode == RFAL_MODE_POLL_NFCA) {
    /*******************************************************************************/
    /* If we received a frame with a incomplete byte we`ll raise a specific error  *
     * ( support for T2T 4 bit ACK / NAK, MIFARE and Kovio )                       */
    /*******************************************************************************/
    if ((gRFAL.TxRx.status == ERR_PAR) || (gRFAL.TxRx.status == ERR_CRC)) {
      if (rfalFIFOStatusIsIncompleteByte()) {
        st25r3916ReadFifo((uint8_t *)(gRFAL.TxRx.ctx.rxBuf), fifoBytesToRead);
        if ((gRFAL.TxRx.ctx.rxRcvdLen) != NULL) {
          *gRFAL.TxRx.ctx.rxRcvdLen = rfalFIFOGetNumIncompleteBits();
        }

        gRFAL.TxRx.status = ERR_INCOMPLETE_BYTE;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
      }
    }
  }
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalCleanupTransceive(void)
{
  /*******************************************************************************/
  /* Transceive flags                                                            */
  /*******************************************************************************/

  /* Restore default settings on NFCIP1 mode, Receiving parity + CRC bits and manual Tx Parity*/
  st25r3916ClrRegisterBits(ST25R3916_REG_ISO14443A_NFC, (ST25R3916_REG_ISO14443A_NFC_no_tx_par | ST25R3916_REG_ISO14443A_NFC_no_rx_par | ST25R3916_REG_ISO14443A_NFC_nfc_f0));

  /* Restore AGC enabled */
  st25r3916SetRegisterBits(ST25R3916_REG_RX_CONF2, ST25R3916_REG_RX_CONF2_agc_en);

  /*******************************************************************************/

  /*******************************************************************************/
  /* Execute Post Transceive Callback                                            */
  /*******************************************************************************/
  if (gRFAL.callbacks.postTxRx != NULL) {
    gRFAL.callbacks.postTxRx();
  }
  /*******************************************************************************/
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalPrepareTransceive(void)
{
  uint32_t maskInterrupts;
  uint8_t  reg;

  /* If we are in RW or AP2P mode */
  if (!rfalIsModePassiveListen(gRFAL.mode)) {
    /* Reset receive logic with STOP command */
    st25r3916ExecuteCommand(ST25R3916_CMD_STOP);

    /* Reset Rx Gain */
    st25r3916ExecuteCommand(ST25R3916_CMD_RESET_RXGAIN);
  } else {
    /* In Passive Listen Mode do not use STOP as it stops FDT timer */
    st25r3916ExecuteCommand(ST25R3916_CMD_CLEAR_FIFO);
  }

  /*******************************************************************************/
  /* FDT Poll                                                                    */
  /*******************************************************************************/
  if (rfalIsModePassiveComm(gRFAL.mode)) {   /* Passive Comms */
    /* In Passive communications General Purpose Timer is used to measure FDT Poll */
    if (gRFAL.timings.FDTPoll != RFAL_TIMING_NONE) {
      /* Configure GPT to start at RX end */
      st25r3916SetStartGPTimer((uint16_t)rfalConv1fcTo8fc(MIN(gRFAL.timings.FDTPoll, (gRFAL.timings.FDTPoll - RFAL_FDT_POLL_ADJUSTMENT))), ST25R3916_REG_TIMER_EMV_CONTROL_gptc_erx);
    }
  }

  /*******************************************************************************/
  /* Execute Pre Transceive Callback                                             */
  /*******************************************************************************/
  if (gRFAL.callbacks.preTxRx != NULL) {
    gRFAL.callbacks.preTxRx();
  }
  /*******************************************************************************/

  maskInterrupts = (ST25R3916_IRQ_MASK_FWL  | ST25R3916_IRQ_MASK_TXE  |
                    ST25R3916_IRQ_MASK_RXS  | ST25R3916_IRQ_MASK_RXE  |
                    ST25R3916_IRQ_MASK_PAR  | ST25R3916_IRQ_MASK_CRC  |
                    ST25R3916_IRQ_MASK_ERR1 | ST25R3916_IRQ_MASK_ERR2 |
                    ST25R3916_IRQ_MASK_NRE);

  /*******************************************************************************/
  /* Transceive flags                                                            */
  /*******************************************************************************/

  reg = (ST25R3916_REG_ISO14443A_NFC_no_tx_par_off | ST25R3916_REG_ISO14443A_NFC_no_rx_par_off | ST25R3916_REG_ISO14443A_NFC_nfc_f0_off);

  /* Check if NFCIP1 mode is to be enabled */
  if ((gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_NFCIP1_ON) != 0U) {
    reg |= ST25R3916_REG_ISO14443A_NFC_nfc_f0;
  }

  /* Check if Parity check is to be skipped and to keep the parity + CRC bits in FIFO */
  if ((gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_PAR_RX_KEEP) != 0U) {
    reg |= ST25R3916_REG_ISO14443A_NFC_no_rx_par;
  }

  /* Check if automatic Parity bits is to be disabled */
  if ((gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_PAR_TX_NONE) != 0U) {
    reg |= ST25R3916_REG_ISO14443A_NFC_no_tx_par;
  }

  /* Apply current TxRx flags on ISO14443A and NFC 106kb/s Settings Register */
  st25r3916ChangeRegisterBits(ST25R3916_REG_ISO14443A_NFC, (ST25R3916_REG_ISO14443A_NFC_no_tx_par | ST25R3916_REG_ISO14443A_NFC_no_rx_par | ST25R3916_REG_ISO14443A_NFC_nfc_f0), reg);

  /* Check if AGC is to be disabled */
  if ((gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_AGC_OFF) != 0U) {
    st25r3916ClrRegisterBits(ST25R3916_REG_RX_CONF2, ST25R3916_REG_RX_CONF2_agc_en);
  } else {
    st25r3916SetRegisterBits(ST25R3916_REG_RX_CONF2, ST25R3916_REG_RX_CONF2_agc_en);
  }
  /*******************************************************************************/

  /*******************************************************************************/
  /* EMVCo NRT mode                                                              */
  /*******************************************************************************/
  if (gRFAL.conf.eHandling == RFAL_ERRORHANDLING_EMVCO) {
    st25r3916SetRegisterBits(ST25R3916_REG_TIMER_EMV_CONTROL, ST25R3916_REG_TIMER_EMV_CONTROL_nrt_emv);
    maskInterrupts |= ST25R3916_IRQ_MASK_RX_REST;
  } else {
    st25r3916ClrRegisterBits(ST25R3916_REG_TIMER_EMV_CONTROL, ST25R3916_REG_TIMER_EMV_CONTROL_nrt_emv);
  }
  /*******************************************************************************/

  /* In Passive Listen mode additionally enable External Field interrupts  */
  if (rfalIsModePassiveListen(gRFAL.mode)) {
    maskInterrupts |= (ST25R3916_IRQ_MASK_EOF | ST25R3916_IRQ_MASK_WU_F);        /* Enable external Field interrupts to detect Link Loss and SENF_REQ auto responses */
  }

  /* In Active comms enable also External Field interrupts  */
  if (rfalIsModeActiveComm(gRFAL.mode)) {
    maskInterrupts |= (ST25R3916_IRQ_MASK_EOF  | ST25R3916_IRQ_MASK_EON  | ST25R3916_IRQ_MASK_PPON2 | ST25R3916_IRQ_MASK_CAT | ST25R3916_IRQ_MASK_CAC);
  }

  /*******************************************************************************/
  /* clear and enable these interrupts */
  st25r3916GetInterrupt(maskInterrupts);
  st25r3916EnableInterrupts(maskInterrupts);

  /* Clear FIFO status local copy */
  rfalFIFOStatusClear();
}

/*******************************************************************************/
void RfalRfST25R3916Class::rfalTransceiveTx(void)
{
  volatile uint32_t irqs;
  uint16_t          tmp;
  ReturnCode        ret;

  /* Suppress warning in case NFC-V feature is disabled */
  ret = ERR_NONE;
  NO_WARNING(ret);

  irqs = ST25R3916_IRQ_MASK_NONE;

  if (gRFAL.TxRx.state != gRFAL.TxRx.lastState) {
    /* rfalLogD( "RFAL: lastSt: %d curSt: %d \r\n", gRFAL.TxRx.lastState, gRFAL.TxRx.state ); */
    gRFAL.TxRx.lastState = gRFAL.TxRx.state;
  }

  switch (gRFAL.TxRx.state) {
    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_IDLE:

      /* Nothing to do */

      gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_WAIT_GT ;
    /* fall through */


    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_WAIT_GT:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */

      if (!rfalIsGTExpired()) {
        break;
      }

      gRFAL.tmr.GT = RFAL_TIMING_NONE;

      gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_WAIT_FDT;
    /* fall through */


    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_WAIT_FDT:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */

      /* Only in Passive communications GPT is used to measure FDT Poll */
      if (rfalIsModePassiveComm(gRFAL.mode)) {
        if (st25r3916IsGPTRunning()) {
          break;
        }
      }

      gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_TRANSMIT;
    /* fall through */


    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_TRANSMIT:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */

      /* Clear FIFO, Clear and Enable the Interrupts */
      rfalPrepareTransceive();

      /* ST25R3916 has a fixed FIFO water level */
      gRFAL.fifo.expWL = RFAL_FIFO_OUT_WL;

      /*******************************************************************************/
      /* In NFC-V streaming mode, the FIFO needs to be loaded with the coded bits    */
      if ((RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode)) {
#if 0
        /* Debugging code: output the payload bits by writing into the FIFO and subsequent clearing */
        st25r3916WriteFifo(gRFAL.TxRx.ctx.txBuf, rfalConvBitsToBytes(gRFAL.TxRx.ctx.txBufLen));
        st25r3916ExecuteCommand(ST25R3916_CMD_CLEAR_FIFO);
#endif
        /* Calculate the bytes needed to be Written into FIFO (a incomplete byte will be added as 1byte) */
        gRFAL.nfcvData.nfcvOffset = 0;
        ret = iso15693VCDCode(gRFAL.TxRx.ctx.txBuf, rfalConvBitsToBytes(gRFAL.TxRx.ctx.txBufLen), (((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL) != 0U) ? false : true), (((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL) != 0U) ? false : true), (RFAL_MODE_POLL_PICOPASS == gRFAL.mode),
                              &gRFAL.fifo.bytesTotal, &gRFAL.nfcvData.nfcvOffset, gRFAL.nfcvData.codingBuffer, MIN((uint16_t)ST25R3916_FIFO_DEPTH, (uint16_t)sizeof(gRFAL.nfcvData.codingBuffer)), &gRFAL.fifo.bytesWritten);

        if ((ret != ERR_NONE) && (ret != ERR_AGAIN)) {
          gRFAL.TxRx.status = ret;
          gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
          break;
        }
        /* Set the number of full bytes and bits to be transmitted */
        st25r3916SetNumTxBits((uint16_t)rfalConvBytesToBits(gRFAL.fifo.bytesTotal));

        /* Load FIFO with coded bytes */
        st25r3916WriteFifo(gRFAL.nfcvData.codingBuffer, gRFAL.fifo.bytesWritten);

      }
      /*******************************************************************************/
      else {
        /* Calculate the bytes needed to be Written into FIFO (a incomplete byte will be added as 1byte) */
        gRFAL.fifo.bytesTotal = (uint16_t)rfalCalcNumBytes(gRFAL.TxRx.ctx.txBufLen);

        /* Set the number of full bytes and bits to be transmitted */
        st25r3916SetNumTxBits(gRFAL.TxRx.ctx.txBufLen);

        /* Load FIFO with total length or FIFO's maximum */
        gRFAL.fifo.bytesWritten = MIN(gRFAL.fifo.bytesTotal, ST25R3916_FIFO_DEPTH);
        st25r3916WriteFifo(gRFAL.TxRx.ctx.txBuf, gRFAL.fifo.bytesWritten);
      }

      /*Check if Observation Mode is enabled and set it on ST25R391x */
      rfalCheckEnableObsModeTx();


      /*******************************************************************************/
      /* If we're in Passive Listen mode ensure that the external field is still On  */
      if (rfalIsModePassiveListen(gRFAL.mode)) {
        if (!rfalIsExtFieldOn()) {
          gRFAL.TxRx.status = ERR_LINK_LOSS;
          gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
          break;
        }
      }

      /*******************************************************************************/
      /* Trigger/Start transmission                                                  */
      if ((gRFAL.TxRx.ctx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL) != 0U) {
        st25r3916ExecuteCommand(ST25R3916_CMD_TRANSMIT_WITHOUT_CRC);
      } else {
        st25r3916ExecuteCommand(ST25R3916_CMD_TRANSMIT_WITH_CRC);
      }

      /* Check if a WL level is expected or TXE should come */
      gRFAL.TxRx.state = ((gRFAL.fifo.bytesWritten < gRFAL.fifo.bytesTotal) ? RFAL_TXRX_STATE_TX_WAIT_WL : RFAL_TXRX_STATE_TX_WAIT_TXE);
      break;

    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_WAIT_WL:

      irqs = st25r3916GetInterrupt((ST25R3916_IRQ_MASK_FWL | ST25R3916_IRQ_MASK_TXE));
      if (irqs == ST25R3916_IRQ_MASK_NONE) {
        break;  /* No interrupt to process */
      }

      if (((irqs & ST25R3916_IRQ_MASK_FWL) != 0U) && ((irqs & ST25R3916_IRQ_MASK_TXE) == 0U)) {
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_RELOAD_FIFO;
      } else {
        gRFAL.TxRx.status = ERR_IO;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
        break;
      }

    /* fall through */

    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_RELOAD_FIFO:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */

      /*******************************************************************************/
      /* In NFC-V streaming mode, the FIFO needs to be loaded with the coded bits    */
      if ((RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode)) {
        uint16_t maxLen;

        /* Load FIFO with the remaining length or maximum available (which fit on the coding buffer) */
        maxLen = (uint16_t)MIN((gRFAL.fifo.bytesTotal - gRFAL.fifo.bytesWritten), gRFAL.fifo.expWL);
        maxLen = (uint16_t)MIN(maxLen, sizeof(gRFAL.nfcvData.codingBuffer));
        tmp    = 0;

        /* Calculate the bytes needed to be Written into FIFO (a incomplete byte will be added as 1byte) */
        ret = iso15693VCDCode(gRFAL.TxRx.ctx.txBuf, rfalConvBitsToBytes(gRFAL.TxRx.ctx.txBufLen), (((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL) != 0U) ? false : true), (((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL) != 0U) ? false : true), (RFAL_MODE_POLL_PICOPASS == gRFAL.mode),
                              &gRFAL.fifo.bytesTotal, &gRFAL.nfcvData.nfcvOffset, gRFAL.nfcvData.codingBuffer, maxLen, &tmp);

        if ((ret != ERR_NONE) && (ret != ERR_AGAIN)) {
          gRFAL.TxRx.status = ret;
          gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
          break;
        }

        /* Load FIFO with coded bytes */
        st25r3916WriteFifo(gRFAL.nfcvData.codingBuffer, tmp);
      }
      /*******************************************************************************/
      else {
        /* Load FIFO with the remaining length or maximum available */
        tmp = MIN((gRFAL.fifo.bytesTotal - gRFAL.fifo.bytesWritten), gRFAL.fifo.expWL);        /* tmp holds the number of bytes written on this iteration */
        st25r3916WriteFifo(&gRFAL.TxRx.ctx.txBuf[gRFAL.fifo.bytesWritten], tmp);
      }

      /* Update total written bytes to FIFO */
      gRFAL.fifo.bytesWritten += tmp;

      /* Check if a WL level is expected or TXE should come */
      gRFAL.TxRx.state = ((gRFAL.fifo.bytesWritten < gRFAL.fifo.bytesTotal) ? RFAL_TXRX_STATE_TX_WAIT_WL : RFAL_TXRX_STATE_TX_WAIT_TXE);
      break;


    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_WAIT_TXE:

      irqs = st25r3916GetInterrupt((ST25R3916_IRQ_MASK_FWL | ST25R3916_IRQ_MASK_TXE));
      if (irqs == ST25R3916_IRQ_MASK_NONE) {
        break;  /* No interrupt to process */
      }


      if ((irqs & ST25R3916_IRQ_MASK_TXE) != 0U) {
        gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_DONE;
      } else if ((irqs & ST25R3916_IRQ_MASK_FWL) != 0U) {
        break;  /* Ignore ST25R3916 FIFO WL if total TxLen is already on the FIFO */
      } else {
        gRFAL.TxRx.status = ERR_IO;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
        break;
      }

    /* fall through */


    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_DONE:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */

      /* If no rxBuf is provided do not wait/expect Rx */
      if (gRFAL.TxRx.ctx.rxBuf == NULL) {
        /*Check if Observation Mode was enabled and disable it on ST25R391x */
        rfalCheckDisableObsMode();

        /* Clean up Transceive */
        rfalCleanupTransceive();

        gRFAL.TxRx.status = ERR_NONE;
        gRFAL.TxRx.state  =  RFAL_TXRX_STATE_IDLE;
        break;
      }

      rfalCheckEnableObsModeRx();

      /* Goto Rx */
      gRFAL.TxRx.state  =  RFAL_TXRX_STATE_RX_IDLE;
      break;

    /*******************************************************************************/
    case RFAL_TXRX_STATE_TX_FAIL:

      /* Error should be assigned by previous state */
      if (gRFAL.TxRx.status == ERR_BUSY) {
        gRFAL.TxRx.status = ERR_SYSTEM;
      }

      /*Check if Observation Mode was enabled and disable it on ST25R391x */
      rfalCheckDisableObsMode();

      /* Clean up Transceive */
      rfalCleanupTransceive();

      gRFAL.TxRx.state = RFAL_TXRX_STATE_IDLE;
      break;

    /*******************************************************************************/
    default:
      gRFAL.TxRx.status = ERR_SYSTEM;
      gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
      break;
  }
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalTransceiveRx(void)
{
  volatile uint32_t irqs;
  uint16_t          tmp;
  uint16_t          aux;

  irqs = ST25R3916_IRQ_MASK_NONE;

  if (gRFAL.TxRx.state != gRFAL.TxRx.lastState) {
    /* rfalLogD( "RFAL: lastSt: %d curSt: %d \r\n", gRFAL.TxRx.lastState, gRFAL.TxRx.state ); */
    gRFAL.TxRx.lastState = gRFAL.TxRx.state;
  }

  switch (gRFAL.TxRx.state) {
    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_IDLE:

      /* Clear rx counters */
      gRFAL.fifo.bytesWritten   = 0;            /* Total bytes written on RxBuffer         */
      gRFAL.fifo.bytesTotal     = 0;            /* Total bytes in FIFO will now be from Rx */
      if (gRFAL.TxRx.ctx.rxRcvdLen != NULL) {
        *gRFAL.TxRx.ctx.rxRcvdLen = 0;
      }

      gRFAL.TxRx.state = (rfalIsModeActiveComm(gRFAL.mode) ? RFAL_TXRX_STATE_RX_WAIT_EON : RFAL_TXRX_STATE_RX_WAIT_RXS);
      break;


    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_WAIT_RXS:

      /*******************************************************************************/
      irqs = st25r3916GetInterrupt((ST25R3916_IRQ_MASK_RXS | ST25R3916_IRQ_MASK_NRE | ST25R3916_IRQ_MASK_EOF));
      if (irqs == ST25R3916_IRQ_MASK_NONE) {
        break;  /* No interrupt to process */
      }

      /* Only raise Timeout if NRE is detected with no Rx Start (NRT EMV mode) */
      if (((irqs & ST25R3916_IRQ_MASK_NRE) != 0U) && ((irqs & ST25R3916_IRQ_MASK_RXS) == 0U)) {
        gRFAL.TxRx.status = ERR_TIMEOUT;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
        break;
      }

      /* Only raise Link Loss if EOF is detected with no Rx Start */
      if (((irqs & ST25R3916_IRQ_MASK_EOF) != 0U) && ((irqs & ST25R3916_IRQ_MASK_RXS) == 0U)) {
        /* In AP2P a Field On has already occurred - treat this as timeout | mute */
        gRFAL.TxRx.status = (rfalIsModeActiveComm(gRFAL.mode) ? ERR_TIMEOUT : ERR_LINK_LOSS);
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
        break;
      }

      if ((irqs & ST25R3916_IRQ_MASK_RXS) != 0U) {
        /*******************************************************************************/
        /* REMARK: Silicon workaround ST25R3916 Errata #TBD                            */
        /* Rarely on corrupted frames I_rxs gets signaled but I_rxe is not signaled    */
        /* Use a SW timer to handle an eventual missing RXE                            */
        rfalTimerStart(gRFAL.tmr.RXE, RFAL_NORXE_TOUT);
        /*******************************************************************************/

        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_WAIT_RXE;
      } else {
        gRFAL.TxRx.status = ERR_IO;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
        break;
      }

      /* remove NRE that might appear together (NRT EMV mode), and remove RXS, but keep EOF if present for next state */
      irqs &= ~(ST25R3916_IRQ_MASK_RXS | ST25R3916_IRQ_MASK_NRE);

    /* fall through */


    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_WAIT_RXE:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */


      /*******************************************************************************/
      /* REMARK: Silicon workaround ST25R3916 Errata #TBD                            */
      /* ST25R396 may indicate RXS without RXE afterwards, this happens rarely on    */
      /* corrupted frames.                                                           */
      /* SW timer is used to timeout upon a missing RXE                              */
      if (rfalTimerisExpired(gRFAL.tmr.RXE)) {
        gRFAL.TxRx.status = ERR_FRAMING;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
      }
      /*******************************************************************************/

      irqs |= st25r3916GetInterrupt((ST25R3916_IRQ_MASK_RXE  | ST25R3916_IRQ_MASK_FWL | ST25R3916_IRQ_MASK_EOF | ST25R3916_IRQ_MASK_RX_REST | ST25R3916_IRQ_MASK_WU_F));
      if (irqs == ST25R3916_IRQ_MASK_NONE) {
        break;  /* No interrupt to process */
      }

      if ((irqs & ST25R3916_IRQ_MASK_RX_REST) != 0U) {
        /* RX_REST indicates that Receiver has been reset due to EMD, therefore a RXS + RXE should *
         * follow if a good reception is followed within the valid initial timeout                   */

        /* Check whether NRT has expired already, if so signal a timeout */
        if (st25r3916GetInterrupt(ST25R3916_IRQ_MASK_NRE) != 0U) {
          gRFAL.TxRx.status = ERR_TIMEOUT;
          gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
          break;
        }
        if (st25r3916CheckReg(ST25R3916_REG_NFCIP1_BIT_RATE, ST25R3916_REG_NFCIP1_BIT_RATE_nrt_on, 0)) {    /* MISRA 13.5 */
          gRFAL.TxRx.status = ERR_TIMEOUT;
          gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
          break;
        }

        /* Discard any previous RXS */
        st25r3916GetInterrupt(ST25R3916_IRQ_MASK_RXS);

        /* Check whether a following reception has already started */
        if (st25r3916CheckReg(ST25R3916_REG_AUX_DISPLAY, ST25R3916_REG_AUX_DISPLAY_rx_act, ST25R3916_REG_AUX_DISPLAY_rx_act)) {
          gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_WAIT_RXE;
          break;
        }

        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_WAIT_RXS;
        break;
      }

      if (((irqs & ST25R3916_IRQ_MASK_FWL) != 0U) && ((irqs & ST25R3916_IRQ_MASK_RXE) == 0U)) {
        gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_READ_FIFO;
        break;
      }

      /* Automatic responses allowed during TxRx only for the SENSF_REQ */
      if ((irqs & ST25R3916_IRQ_MASK_WU_F) != 0U) {
        gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_WAIT_RXS;
        break;
      }

      /* After RXE retrieve and check for any error irqs */
      irqs |= st25r3916GetInterrupt((ST25R3916_IRQ_MASK_CRC | ST25R3916_IRQ_MASK_PAR | ST25R3916_IRQ_MASK_ERR1 | ST25R3916_IRQ_MASK_ERR2 | ST25R3916_IRQ_MASK_COL));

      gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_ERR_CHECK;
    /* fall through */


    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_ERR_CHECK:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */

      if ((irqs & ST25R3916_IRQ_MASK_ERR1) != 0U) {
        gRFAL.TxRx.status = ERR_FRAMING;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;

        /* Check if there's a specific error handling for this */
        rfalErrorHandling();
        break;
      }
      /* Discard Soft Framing errors in AP2P and CE */
      else if (rfalIsModePassivePoll(gRFAL.mode) && ((irqs & ST25R3916_IRQ_MASK_ERR2) != 0U)) {
        gRFAL.TxRx.status = ERR_FRAMING;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;

        /* Check if there's a specific error handling for this */
        rfalErrorHandling();
        break;
      } else if ((irqs & ST25R3916_IRQ_MASK_PAR) != 0U) {
        gRFAL.TxRx.status = ERR_PAR;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;

        /* Check if there's a specific error handling for this */
        rfalErrorHandling();
        break;
      } else if ((irqs & ST25R3916_IRQ_MASK_CRC) != 0U) {
        gRFAL.TxRx.status = ERR_CRC;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;

        /* Check if there's a specific error handling for this */
        rfalErrorHandling();
        break;
      } else if ((irqs & ST25R3916_IRQ_MASK_COL) != 0U) {
        gRFAL.TxRx.status = ERR_RF_COLLISION;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;

        /* Check if there's a specific error handling for this */
        rfalErrorHandling();
        break;
      } else if (rfalIsModePassiveListen(gRFAL.mode) && ((irqs & ST25R3916_IRQ_MASK_EOF) != 0U)) {
        gRFAL.TxRx.status = ERR_LINK_LOSS;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
        break;
      } else if ((irqs & ST25R3916_IRQ_MASK_RXE) != 0U) {
        /* Reception ended without any error indication,                  *
         * check FIFO status for malformed or incomplete frames           */

        /* Check if the reception ends with an incomplete byte (residual bits) */
        if (rfalFIFOStatusIsIncompleteByte()) {
          gRFAL.TxRx.status = ERR_INCOMPLETE_BYTE;
        }
        /* Check if the reception ends missing parity bit */
        else if (rfalFIFOStatusIsMissingPar()) {
          gRFAL.TxRx.status = ERR_FRAMING;
        } else {
          /* MISRA 15.7 - Empty else */
        }

        gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_READ_DATA;
      } else {
        gRFAL.TxRx.status = ERR_IO;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
        break;
      }

    /* fall through */


    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_READ_DATA:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */

      tmp = rfalFIFOStatusGetNumBytes();

      /*******************************************************************************/
      /* Check if CRC should not be placed in rxBuf                                  */
      if (((gRFAL.TxRx.ctx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP) == 0U)) {
        /* if received frame was bigger than CRC */
        if ((uint16_t)(gRFAL.fifo.bytesTotal + tmp) > 0U) {
          /* By default CRC will not be placed into the rxBuffer */
          if ((tmp > RFAL_CRC_LEN)) {
            tmp -= RFAL_CRC_LEN;
          }
          /* If the CRC was already placed into rxBuffer (due to WL interrupt where CRC was already in FIFO Read)
           * cannot remove it from rxBuf. Can only remove it from rxBufLen not indicate the presence of CRC    */
          else if (gRFAL.fifo.bytesTotal > RFAL_CRC_LEN) {
            gRFAL.fifo.bytesTotal -= RFAL_CRC_LEN;
          } else {
            /* MISRA 15.7 - Empty else */
          }
        }
      }

      gRFAL.fifo.bytesTotal += tmp;                    /* add to total bytes counter */

      /*******************************************************************************/
      /* Check if remaining bytes fit on the rxBuf available                         */
      if (gRFAL.fifo.bytesTotal > rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen)) {
        tmp = (uint16_t)(rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen) - gRFAL.fifo.bytesWritten);

        gRFAL.TxRx.status = ERR_NOMEM;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
      }

      /*******************************************************************************/
      /* Retrieve remaining bytes from FIFO to rxBuf, and assign total length rcvd   */
      st25r3916ReadFifo(&gRFAL.TxRx.ctx.rxBuf[gRFAL.fifo.bytesWritten], tmp);
      if (gRFAL.TxRx.ctx.rxRcvdLen != NULL) {
        (*gRFAL.TxRx.ctx.rxRcvdLen) = (uint16_t)rfalConvBytesToBits(gRFAL.fifo.bytesTotal);
        if (rfalFIFOStatusIsIncompleteByte()) {
          (*gRFAL.TxRx.ctx.rxRcvdLen) -= (RFAL_BITS_IN_BYTE - rfalFIFOGetNumIncompleteBits());
        }
      }

      /*******************************************************************************/
      /* Decode sub bit stream into payload bits for NFCV, if no error found so far  */
      if (((RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode)) && (gRFAL.TxRx.status == ERR_BUSY)) {
        ReturnCode ret;
        uint16_t offset = 0; /* REMARK offset not currently used */

        ret = iso15693VICCDecode(gRFAL.TxRx.ctx.rxBuf, gRFAL.fifo.bytesTotal,
                                 gRFAL.nfcvData.origCtx.rxBuf, rfalConvBitsToBytes(gRFAL.nfcvData.origCtx.rxBufLen), &offset, gRFAL.nfcvData.origCtx.rxRcvdLen, gRFAL.nfcvData.ignoreBits, (RFAL_MODE_POLL_PICOPASS == gRFAL.mode));

        if (((ERR_NONE == ret) || (ERR_CRC == ret))
            && (((uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP & gRFAL.nfcvData.origCtx.flags) == 0U)
            && ((*gRFAL.nfcvData.origCtx.rxRcvdLen % RFAL_BITS_IN_BYTE) == 0U)
            && (*gRFAL.nfcvData.origCtx.rxRcvdLen >= rfalConvBytesToBits(RFAL_CRC_LEN))
           ) {
          *gRFAL.nfcvData.origCtx.rxRcvdLen -= (uint16_t)rfalConvBytesToBits(RFAL_CRC_LEN); /* Remove CRC */
        }
#if 0
        /* Debugging code: output the payload bits by writing into the FIFO and subsequent clearing */
        st25r3916WriteFifo(gRFAL.nfcvData.origCtx.rxBuf, rfalConvBitsToBytes(*gRFAL.nfcvData.origCtx.rxRcvdLen));
        st25r3916ExecuteCommand(ST25R3916_CMD_CLEAR_FIFO);
#endif

        /* Restore original ctx */
        gRFAL.TxRx.ctx    = gRFAL.nfcvData.origCtx;
        gRFAL.TxRx.status = ((ret != ERR_NONE) ? ret : ERR_BUSY);
      }

      /*******************************************************************************/
      /* If an error as been marked/detected don't fall into to RX_DONE  */
      if (gRFAL.TxRx.status != ERR_BUSY) {
        gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_FAIL;
        break;
      }

      if (rfalIsModeActiveComm(gRFAL.mode)) {
        gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_WAIT_EOF;
        break;
      }

      gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_DONE;
    /* fall through */


    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_DONE:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */

      /*Check if Observation Mode was enabled and disable it on ST25R391x */
      rfalCheckDisableObsMode();

      /* Clean up Transceive */
      rfalCleanupTransceive();


      gRFAL.TxRx.status = ERR_NONE;
      gRFAL.TxRx.state  = RFAL_TXRX_STATE_IDLE;
      break;


    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_READ_FIFO:

      /*******************************************************************************/
      /* REMARK: Silicon workaround ST25R3916 Errata #TBD                            */
      /* Rarely on corrupted frames I_rxs gets signaled but I_rxe is not signaled    */
      /* Use a SW timer to handle an eventual missing RXE                            */
      rfalTimerStart(gRFAL.tmr.RXE, RFAL_NORXE_TOUT);
      /*******************************************************************************/

      tmp = rfalFIFOStatusGetNumBytes();
      gRFAL.fifo.bytesTotal += tmp;

      /*******************************************************************************/
      /* Calculate the amount of bytes that still fits in rxBuf                      */
      aux = ((gRFAL.fifo.bytesTotal > rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen)) ? (rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen) - gRFAL.fifo.bytesWritten) : tmp);

      /*******************************************************************************/
      /* Retrieve incoming bytes from FIFO to rxBuf, and store already read amount   */
      st25r3916ReadFifo(&gRFAL.TxRx.ctx.rxBuf[gRFAL.fifo.bytesWritten], aux);
      gRFAL.fifo.bytesWritten += aux;

      /*******************************************************************************/
      /* If the bytes already read were not the full FIFO WL, dump the remaining     *
       * FIFO so that ST25R391x can continue with reception                          */
      if (aux < tmp) {
        st25r3916ReadFifo(NULL, (tmp - aux));
      }

      rfalFIFOStatusClear();
      gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_WAIT_RXE;
      break;


    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_FAIL:

      /*Check if Observation Mode was enabled and disable it on ST25R391x */
      rfalCheckDisableObsMode();

      /* Clean up Transceive */
      rfalCleanupTransceive();

      /* Error should be assigned by previous state */
      if (gRFAL.TxRx.status == ERR_BUSY) {
        gRFAL.TxRx.status = ERR_SYSTEM;
      }

      /*rfalLogD( "RFAL: curSt: %d  Error: %d \r\n", gRFAL.TxRx.state, gRFAL.TxRx.status );*/
      gRFAL.TxRx.state = RFAL_TXRX_STATE_IDLE;
      break;


    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_WAIT_EON:

      irqs = st25r3916GetInterrupt((ST25R3916_IRQ_MASK_EON | ST25R3916_IRQ_MASK_NRE | ST25R3916_IRQ_MASK_PPON2));
      if (irqs == ST25R3916_IRQ_MASK_NONE) {
        break;  /* No interrupt to process */
      }

      if ((irqs & ST25R3916_IRQ_MASK_EON) != 0U) {
        gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_WAIT_RXS;
      }

      if ((irqs & ST25R3916_IRQ_MASK_NRE) != 0U) {
        gRFAL.TxRx.status = ERR_TIMEOUT;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
      }
      if ((irqs & ST25R3916_IRQ_MASK_PPON2) != 0U) {
        gRFAL.TxRx.status = ERR_LINK_LOSS;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
      }
      break;


    /*******************************************************************************/
    case RFAL_TXRX_STATE_RX_WAIT_EOF:

      irqs = st25r3916GetInterrupt((ST25R3916_IRQ_MASK_CAT | ST25R3916_IRQ_MASK_CAC));
      if (irqs == ST25R3916_IRQ_MASK_NONE) {
        break;  /* No interrupt to process */
      }

      if ((irqs & ST25R3916_IRQ_MASK_CAT) != 0U) {
        gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_DONE;
      } else if ((irqs & ST25R3916_IRQ_MASK_CAC) != 0U) {
        gRFAL.TxRx.status = ERR_RF_COLLISION;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
      } else {
        gRFAL.TxRx.status = ERR_IO;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
      }
      break;


    /*******************************************************************************/
    default:
      gRFAL.TxRx.status = ERR_SYSTEM;
      gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
      break;
  }
}

/*******************************************************************************/
void RfalRfST25R3916Class::rfalFIFOStatusUpdate(void)
{
  if (gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] == RFAL_FIFO_STATUS_INVALID) {
    st25r3916ReadMultipleRegisters(ST25R3916_REG_FIFO_STATUS1, gRFAL.fifo.status, ST25R3916_FIFO_STATUS_LEN);
  }
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalFIFOStatusClear(void)
{
  gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] = RFAL_FIFO_STATUS_INVALID;
}


/*******************************************************************************/
uint8_t RfalRfST25R3916Class::rfalFIFOStatusGetNumBytes(void)
{
  uint16_t result;

  rfalFIFOStatusUpdate();

  result  = ((((uint16_t)gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] & ST25R3916_REG_FIFO_STATUS2_fifo_b_mask) >> ST25R3916_REG_FIFO_STATUS2_fifo_b_shift) << RFAL_BITS_IN_BYTE);
  result |= (((uint16_t)gRFAL.fifo.status[RFAL_FIFO_STATUS_REG1]) & 0x00FFU);
  return result;
}


/*******************************************************************************/
bool RfalRfST25R3916Class::rfalFIFOStatusIsIncompleteByte(void)
{
  rfalFIFOStatusUpdate();
  return ((gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] & ST25R3916_REG_FIFO_STATUS2_fifo_lb_mask) != 0U);
}


/*******************************************************************************/
bool RfalRfST25R3916Class::rfalFIFOStatusIsMissingPar(void)
{
  rfalFIFOStatusUpdate();
  return ((gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] & ST25R3916_REG_FIFO_STATUS2_np_lb) != 0U);
}


/*******************************************************************************/
uint8_t RfalRfST25R3916Class::rfalFIFOGetNumIncompleteBits(void)
{
  rfalFIFOStatusUpdate();
  return ((gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] & ST25R3916_REG_FIFO_STATUS2_fifo_lb_mask) >> ST25R3916_REG_FIFO_STATUS2_fifo_lb_shift);
}

/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalISO14443ATransceiveShortFrame(rfal14443AShortFrameCmd txCmd, uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *rxRcvdLen, uint32_t fwt)
{
  ReturnCode ret;
  uint8_t    directCmd;

  /* Check if RFAL is properly initialized */
  if (!st25r3916IsTxEnabled() || (gRFAL.state < RFAL_STATE_MODE_SET) || ((gRFAL.mode != RFAL_MODE_POLL_NFCA) && (gRFAL.mode != RFAL_MODE_POLL_NFCA_T1T))) {
    return ERR_WRONG_STATE;
  }

  /* Check for valid parameters */
  if ((rxBuf == NULL) || (rxRcvdLen == NULL) || (fwt == RFAL_FWT_NONE)) {
    return ERR_PARAM;
  }

  /*******************************************************************************/
  /* Select the Direct Command to be performed                                   */
  switch (txCmd) {
    case RFAL_14443A_SHORTFRAME_CMD_WUPA:
      directCmd = ST25R3916_CMD_TRANSMIT_WUPA;
      break;

    case RFAL_14443A_SHORTFRAME_CMD_REQA:
      directCmd = ST25R3916_CMD_TRANSMIT_REQA;
      break;

    default:
      return ERR_PARAM;
  }


  /* Disable CRC while receiving since ATQA has no CRC included */
  st25r3916SetRegisterBits(ST25R3916_REG_AUX, ST25R3916_REG_AUX_no_crc_rx);


  /*******************************************************************************/
  /* Wait for GT and FDT */
  while (!rfalIsGTExpired())      { /* MISRA 15.6: mandatory brackets */ };
  while (st25r3916IsGPTRunning()) { /* MISRA 15.6: mandatory brackets */ };

  gRFAL.tmr.GT = RFAL_TIMING_NONE;


  /*******************************************************************************/
  /* Prepare for Transceive, Receive only (bypass Tx states) */
  gRFAL.TxRx.ctx.flags     = ((uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP);
  gRFAL.TxRx.ctx.rxBuf     = rxBuf;
  gRFAL.TxRx.ctx.rxBufLen  = rxBufLen;
  gRFAL.TxRx.ctx.rxRcvdLen = rxRcvdLen;

  /*******************************************************************************/
  /* Load NRT with FWT */
  st25r3916SetNoResponseTime(rfalConv1fcTo64fc(MIN((fwt + RFAL_FWT_ADJUSTMENT + RFAL_FWT_A_ADJUSTMENT), RFAL_ST25R3916_NRT_MAX_1FC)));

  if (gRFAL.timings.FDTListen != RFAL_TIMING_NONE) {

    /* Ensure that MRT is using 64/fc steps */
    st25r3916ClrRegisterBits(ST25R3916_REG_TIMER_EMV_CONTROL, ST25R3916_REG_TIMER_EMV_CONTROL_mrt_step);

    /* Set Minimum FDT(Listen) in which PICC is not allowed to send a response */
    st25r3916WriteRegister(ST25R3916_REG_MASK_RX_TIMER, (uint8_t)rfalConv1fcTo64fc(((RFAL_FDT_LISTEN_MRT_ADJUSTMENT + RFAL_FDT_LISTEN_A_ADJUSTMENT) > gRFAL.timings.FDTListen) ? RFAL_ST25R3916_MRT_MIN_1FC : (gRFAL.timings.FDTListen - (RFAL_FDT_LISTEN_MRT_ADJUSTMENT + RFAL_FDT_LISTEN_A_ADJUSTMENT))));
  }

  /* In Passive communications General Purpose Timer is used to measure FDT Poll */
  if (gRFAL.timings.FDTPoll != RFAL_TIMING_NONE) {
    /* Configure GPT to start at RX end */
    st25r3916SetStartGPTimer((uint16_t)rfalConv1fcTo8fc(MIN(gRFAL.timings.FDTPoll, (gRFAL.timings.FDTPoll - RFAL_FDT_POLL_ADJUSTMENT))), ST25R3916_REG_TIMER_EMV_CONTROL_gptc_erx);
  }

  /*******************************************************************************/
  rfalPrepareTransceive();

  /* Also enable bit collision interrupt */
  st25r3916GetInterrupt(ST25R3916_IRQ_MASK_COL);
  st25r3916EnableInterrupts(ST25R3916_IRQ_MASK_COL);

  /*Check if Observation Mode is enabled and set it on ST25R391x */
  rfalCheckEnableObsModeTx();

  /*******************************************************************************/
  /* Clear nbtx bits before sending WUPA/REQA - otherwise ST25R3916 will report parity error, Note2 of the register */
  st25r3916WriteRegister(ST25R3916_REG_NUM_TX_BYTES2, 0);

  /* Send either WUPA or REQA. All affected tags will backscatter ATQA and change to READY state */
  st25r3916ExecuteCommand(directCmd);

  /* Wait for TXE */
  if (st25r3916WaitForInterruptsTimed(ST25R3916_IRQ_MASK_TXE, (uint16_t)MAX(rfalConv1fcToMs(fwt), RFAL_ST25R3916_SW_TMR_MIN_1MS)) == 0U) {
    ret = ERR_IO;
  } else {
    /*Check if Observation Mode is enabled and set it on ST25R391x */
    rfalCheckEnableObsModeRx();

    /* Jump into a transceive Rx state for reception (bypass Tx states) */
    gRFAL.state       = RFAL_STATE_TXRX;
    gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_IDLE;
    gRFAL.TxRx.status = ERR_BUSY;

    /* Execute Transceive Rx blocking */
    ret = rfalTransceiveBlockingRx();
  }


  /* Disable Collision interrupt */
  st25r3916DisableInterrupts((ST25R3916_IRQ_MASK_COL));

  /* Re-enable CRC on Rx */
  st25r3916ClrRegisterBits(ST25R3916_REG_AUX, ST25R3916_REG_AUX_no_crc_rx);

  return ret;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalISO14443ATransceiveAnticollisionFrame(uint8_t *buf, uint8_t *bytesToSend, uint8_t *bitsToSend, uint16_t *rxLength, uint32_t fwt)
{
  ReturnCode            ret;
  rfalTransceiveContext ctx;
  uint8_t               collByte;
  uint8_t               collData;

  /* Check if RFAL is properly initialized */
  if ((gRFAL.state < RFAL_STATE_MODE_SET) || (gRFAL.mode != RFAL_MODE_POLL_NFCA)) {
    return ERR_WRONG_STATE;
  }

  /* Check for valid parameters */
  if ((buf == NULL) || (bytesToSend == NULL) || (bitsToSend == NULL) || (rxLength == NULL)) {
    return ERR_PARAM;
  }

  /*******************************************************************************/
  /* Set specific Analog Config for Anticolission if needed */
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_ANTICOL));


  /*******************************************************************************/
  /* Enable anti collision to recognise collision in first byte of SENS_REQ */
  st25r3916SetRegisterBits(ST25R3916_REG_ISO14443A_NFC, ST25R3916_REG_ISO14443A_NFC_antcl);

  /* Disable CRC while receiving */
  st25r3916SetRegisterBits(ST25R3916_REG_AUX, ST25R3916_REG_AUX_no_crc_rx);



  /*******************************************************************************/
  /* Prepare for Transceive                                                      */
  ctx.flags     = ((uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP);
  ctx.txBuf     = buf;
  ctx.txBufLen  = (uint16_t)(rfalConvBytesToBits(*bytesToSend) + *bitsToSend);
  ctx.rxBuf     = &buf[*bytesToSend];
  ctx.rxBufLen  = (uint16_t)rfalConvBytesToBits(RFAL_ISO14443A_SDD_RES_LEN);
  ctx.rxRcvdLen = rxLength;
  ctx.fwt       = fwt;

  /* Disable Automatic Gain Control (AGC) for better detection of collisions if using Coherent Receiver */
  ctx.flags    |= (st25r3916CheckReg(ST25R3916_REG_AUX, ST25R3916_REG_AUX_dis_corr, ST25R3916_REG_AUX_dis_corr) ? (uint32_t)RFAL_TXRX_FLAGS_AGC_OFF : 0x00U);


  rfalStartTransceive(&ctx);

  /* Additionally enable bit collision interrupt */
  st25r3916GetInterrupt(ST25R3916_IRQ_MASK_COL);
  st25r3916EnableInterrupts(ST25R3916_IRQ_MASK_COL);

  /*******************************************************************************/
  collByte = 0;

  /* save the collision byte */
  if ((*bitsToSend) > 0U) {
    buf[(*bytesToSend)] <<= (RFAL_BITS_IN_BYTE - (*bitsToSend));
    buf[(*bytesToSend)] >>= (RFAL_BITS_IN_BYTE - (*bitsToSend));
    collByte = buf[(*bytesToSend)];
  }


  /*******************************************************************************/
  /* Run Transceive blocking */
  ret = rfalTransceiveRunBlockingTx();
  if (ret == ERR_NONE) {
    ret = rfalTransceiveBlockingRx();

    /*******************************************************************************/
    if ((*bitsToSend) > 0U) {
      buf[(*bytesToSend)] >>= (*bitsToSend);
      buf[(*bytesToSend)] <<= (*bitsToSend);
      buf[(*bytesToSend)] |= collByte;
    }

    if ((ERR_RF_COLLISION == ret)) {
      /* read out collision register */
      st25r3916ReadRegister(ST25R3916_REG_COLLISION_STATUS, &collData);

      (*bytesToSend) = ((collData >> ST25R3916_REG_COLLISION_STATUS_c_byte_shift) & 0x0FU); // 4-bits Byte information
      (*bitsToSend)  = ((collData >> ST25R3916_REG_COLLISION_STATUS_c_bit_shift)  & 0x07U); // 3-bits bit information

    }
  }


  /*******************************************************************************/
  /* Disable Collision interrupt */
  st25r3916DisableInterrupts((ST25R3916_IRQ_MASK_COL));

  /* Disable anti collision again */
  st25r3916ClrRegisterBits(ST25R3916_REG_ISO14443A_NFC, ST25R3916_REG_ISO14443A_NFC_antcl);

  /* Re-enable CRC on Rx */
  st25r3916ClrRegisterBits(ST25R3916_REG_AUX, ST25R3916_REG_AUX_no_crc_rx);
  /*******************************************************************************/

  /* Restore common Analog configurations for this mode */
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX));
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX));

  return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalISO15693TransceiveAnticollisionFrame(uint8_t *txBuf, uint8_t txBufLen, uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen)
{
  ReturnCode            ret;
  rfalTransceiveContext ctx;

  /* Check if RFAL is properly initialized */
  if ((gRFAL.state < RFAL_STATE_MODE_SET) || (gRFAL.mode != RFAL_MODE_POLL_NFCV)) {
    return ERR_WRONG_STATE;
  }

  /*******************************************************************************/
  /* Set specific Analog Config for Anticolission if needed */
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_ANTICOL));


  /* Ignoring collisions before the UID (RES_FLAG + DSFID) */
  gRFAL.nfcvData.ignoreBits = (uint16_t)RFAL_ISO15693_IGNORE_BITS;

  /*******************************************************************************/
  /* Prepare for Transceive  */
  ctx.flags     = ((txBufLen == 0U) ? (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL : (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_AUTO) | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP | (uint32_t)RFAL_TXRX_FLAGS_AGC_OFF | ((txBufLen == 0U) ? (uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL : (uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_AUTO); /* Disable Automatic Gain Control (AGC) for better detection of collision */
  ctx.txBuf     = txBuf;
  ctx.txBufLen  = (uint16_t)rfalConvBytesToBits(txBufLen);
  ctx.rxBuf     = rxBuf;
  ctx.rxBufLen  = (uint16_t)rfalConvBytesToBits(rxBufLen);
  ctx.rxRcvdLen = actLen;
  ctx.fwt       = rfalConv64fcTo1fc(ISO15693_FWT);

  rfalStartTransceive(&ctx);

  /*******************************************************************************/
  /* Run Transceive blocking */
  ret = rfalTransceiveRunBlockingTx();
  if (ret == ERR_NONE) {
    ret = rfalTransceiveBlockingRx();
  }

  /* Check if a Transmission error and received data is less then expected */
  if (((ret == ERR_RF_COLLISION) || (ret == ERR_CRC) || (ret == ERR_FRAMING)) && (rfalConvBitsToBytes(*ctx.rxRcvdLen) < RFAL_ISO15693_INV_RES_LEN)) {
    /* If INVENTORY_RES is shorter than expected, tag is still modulating *
     * Ensure that response is complete before next frame                 */
    delay((uint8_t)((RFAL_ISO15693_INV_RES_LEN - rfalConvBitsToBytes(*ctx.rxRcvdLen)) / ((RFAL_ISO15693_INV_RES_LEN / RFAL_ISO15693_INV_RES_DUR) + 1U)));
  }

  /* Restore common Analog configurations for this mode */
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX));
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX));

  gRFAL.nfcvData.ignoreBits = 0;
  return ret;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalISO15693TransceiveEOFAnticollision(uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen)
{
  uint8_t dummy;

  return rfalISO15693TransceiveAnticollisionFrame(&dummy, 0, rxBuf, rxBufLen, actLen);
}

/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalISO15693TransceiveEOF(uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen)
{
  ReturnCode ret;
  uint8_t    dummy;

  /* Check if RFAL is properly initialized */
  if ((gRFAL.state < RFAL_STATE_MODE_SET) || (gRFAL.mode != RFAL_MODE_POLL_NFCV)) {
    return ERR_WRONG_STATE;
  }

  /*******************************************************************************/
  /* Run Transceive blocking */
  ret = rfalTransceiveBlockingTxRx(&dummy,
                                   0,
                                   rxBuf,
                                   rxBufLen,
                                   actLen,
                                   ((uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP | (uint32_t)RFAL_TXRX_FLAGS_AGC_ON),
                                   rfalConv64fcTo1fc(ISO15693_FWT));
  return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalFeliCaPoll(rfalFeliCaPollSlots slots, uint16_t sysCode, uint8_t reqCode, rfalFeliCaPollRes *pollResList, uint8_t pollResListSize, uint8_t *devicesDetected, uint8_t *collisionsDetected)
{
  ReturnCode        ret;
  uint8_t           frame[RFAL_FELICA_POLL_REQ_LEN - RFAL_FELICA_LEN_LEN];  // LEN is added by ST25R391x automatically
  uint16_t          actLen;
  uint8_t           frameIdx;
  uint8_t           devDetected;
  uint8_t           colDetected;
  rfalEHandling     curHandling;
  uint8_t           nbSlots;

  /* Check if RFAL is properly initialized */
  if ((gRFAL.state < RFAL_STATE_MODE_SET) || (gRFAL.mode != RFAL_MODE_POLL_NFCF)) {
    return ERR_WRONG_STATE;
  }

  frameIdx    = 0;
  colDetected = 0;
  devDetected = 0;
  nbSlots     = (uint8_t)slots;

  /*******************************************************************************/
  /* Compute SENSF_REQ frame */
  frame[frameIdx++] = (uint8_t)FELICA_CMD_POLLING; /* CMD: SENF_REQ                       */
  frame[frameIdx++] = (uint8_t)(sysCode >> 8);     /* System Code (SC)                    */
  frame[frameIdx++] = (uint8_t)(sysCode & 0xFFU);  /* System Code (SC)                    */
  frame[frameIdx++] = reqCode;                     /* Communication Parameter Request (RC)*/
  frame[frameIdx++] = nbSlots;                     /* TimeSlot (TSN)                      */


  /*******************************************************************************/
  /* NRT should not stop on reception - Use EMVCo mode to run NRT in nrt_emv     *
   * ERRORHANDLING_EMVCO has no special handling for NFC-F mode                  */
  curHandling = gRFAL.conf.eHandling;
  rfalSetErrorHandling(RFAL_ERRORHANDLING_EMVCO);

  /*******************************************************************************/
  /* Run transceive blocking,
   * Calculate Total Response Time in(64/fc):
   *                       512 PICC process time + (n * 256 Time Slot duration)  */
  ret = rfalTransceiveBlockingTx(frame,
                                 (uint16_t)frameIdx,
                                 (uint8_t *)gRFAL.nfcfData.pollResponses,
                                 RFAL_FELICA_POLL_RES_LEN,
                                 &actLen,
                                 (RFAL_TXRX_FLAGS_DEFAULT),
                                 rfalConv64fcTo1fc(RFAL_FELICA_POLL_DELAY_TIME + (RFAL_FELICA_POLL_SLOT_TIME * ((uint32_t)nbSlots + 1U))));

  /*******************************************************************************/
  /* If Tx OK, Wait for all responses, store them as soon as they appear         */
  if (ret == ERR_NONE) {
    bool timeout;

    do {
      ret = rfalTransceiveBlockingRx();
      if (ret == ERR_TIMEOUT) {
        /* Upon timeout the full Poll Delay + (Slot time)*(nbSlots) has expired */
        timeout = true;
      } else {
        /* Reception done, re-enabled Rx for following Slot */
        st25r3916ExecuteCommand(ST25R3916_CMD_UNMASK_RECEIVE_DATA);
        st25r3916ExecuteCommand(ST25R3916_CMD_RESET_RXGAIN);

        /* If the reception was OK, new device found */
        if (ret == ERR_NONE) {
          devDetected++;

          /* Overwrite the Transceive context for the next reception */
          gRFAL.TxRx.ctx.rxBuf = (uint8_t *)gRFAL.nfcfData.pollResponses[devDetected];
        }
        /* If the reception was not OK, mark as collision */
        else {
          colDetected++;
        }

        /* Check whether NRT has expired meanwhile  */
        timeout = st25r3916CheckReg(ST25R3916_REG_NFCIP1_BIT_RATE, ST25R3916_REG_NFCIP1_BIT_RATE_nrt_on, 0x00);
        if (!timeout) {
          /* Jump again into transceive Rx state for the following reception */
          gRFAL.TxRx.status = ERR_BUSY;
          gRFAL.state       = RFAL_STATE_TXRX;
          gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_IDLE;
        }
      }


    } while (((nbSlots--) != 0U) && !timeout);
  }

  /*******************************************************************************/
  /* Restore NRT to normal mode - back to previous error handling */
  rfalSetErrorHandling(curHandling);

  /*******************************************************************************/
  /* Assign output parameters if requested                                       */

  if ((pollResList != NULL) && (pollResListSize > 0U) && (devDetected > 0U)) {
    ST_MEMCPY(pollResList, gRFAL.nfcfData.pollResponses, (RFAL_FELICA_POLL_RES_LEN * (uint32_t)MIN(pollResListSize, devDetected)));
  }

  if (devicesDetected != NULL) {
    *devicesDetected = devDetected;
  }

  if (collisionsDetected != NULL) {
    *collisionsDetected = colDetected;
  }

  return (((colDetected != 0U) || (devDetected != 0U)) ? ERR_NONE : ret);
}


/*******************************************************************************/
bool RfalRfST25R3916Class::rfalIsExtFieldOn(void)
{
  return st25r3916IsExtFieldOn();
}

/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalListenStart(uint32_t lmMask, const rfalLmConfPA *confA, const rfalLmConfPB *confB, const rfalLmConfPF *confF, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rxLen)
{
  (void)lmMask;
  (void)confA;
  (void)confB;
  (void)confF;
  (void)rxBuf;
  (void)rxBufLen;
  (void)rxLen;
  return ERR_NOTSUPP;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalListenSleepStart(rfalLmState sleepSt, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rxLen)
{
  (void)sleepSt;
  (void)rxBuf;
  (void)rxBufLen;
  (void)rxLen;
  return ERR_NOTSUPP;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalListenStop(void)
{
  return ERR_NOTSUPP;
}

/*******************************************************************************/
rfalLmState RfalRfST25R3916Class::rfalListenGetState(bool *dataFlag, rfalBitRate *lastBR)
{
  (void)dataFlag;
  (void)lastBR;
  return RFAL_LM_STATE_NOT_INIT;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalListenSetState(rfalLmState newSt)
{
  (void)newSt;
  return ERR_NOTSUPP;
}


/*******************************************************************************
 *  Wake-Up Mode                                                               *
 *******************************************************************************/

/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalWakeUpModeStart(const rfalWakeUpConfig *config)
{
  uint8_t                aux;
  uint8_t                reg;
  uint32_t               irqs;

  /* The Wake-Up procedure is explained in detail in Application Note: AN4985 */

  if (config == NULL) {
    gRFAL.wum.cfg.period      = RFAL_WUM_PERIOD_500MS;
    gRFAL.wum.cfg.irqTout     = false;
    gRFAL.wum.cfg.swTagDetect = true;

    gRFAL.wum.cfg.indAmp.enabled   = true;
    gRFAL.wum.cfg.indPha.enabled   = false;
    gRFAL.wum.cfg.cap.enabled      = false;
    gRFAL.wum.cfg.indAmp.delta     = 2U;
    gRFAL.wum.cfg.indAmp.reference = RFAL_WUM_REFERENCE_AUTO;
    gRFAL.wum.cfg.indAmp.autoAvg   = false;
  } else {
    gRFAL.wum.cfg = *config;
  }

  /* Check for valid configuration */
  if ((!gRFAL.wum.cfg.cap.enabled && !gRFAL.wum.cfg.indAmp.enabled && !gRFAL.wum.cfg.indPha.enabled)  ||
      (gRFAL.wum.cfg.cap.enabled  && (gRFAL.wum.cfg.indAmp.enabled || gRFAL.wum.cfg.indPha.enabled))  ||
      (gRFAL.wum.cfg.cap.enabled  &&  gRFAL.wum.cfg.swTagDetect)) {
    return ERR_PARAM;
  }

  irqs = ST25R3916_IRQ_MASK_NONE;

  /* Disable Tx, Rx, External Field Detector and set default ISO14443A mode */
  st25r3916TxRxOff();
  st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_en_fd_mask);
  st25r3916ChangeRegisterBits(ST25R3916_REG_MODE, (ST25R3916_REG_MODE_targ | ST25R3916_REG_MODE_om_mask), (ST25R3916_REG_MODE_targ_init | ST25R3916_REG_MODE_om_iso14443a));

  /* Set Analog configurations for Wake-up On event */
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_WAKEUP_ON));


  /*******************************************************************************/
  /* Check if AAT is enabled. If so disable en bit to give time for the Voltage  *
   * on the to varicaps to settle and have a stable reference measurement        */
  if (st25r3916CheckReg(ST25R3916_REG_IO_CONF2, ST25R3916_REG_IO_CONF2_aat_en, ST25R3916_REG_IO_CONF2_aat_en) && !gRFAL.wum.cfg.swTagDetect) {
    st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_en);
    delay(RFAL_ST25R3916_AAT_SETTLE_OFF);
  }
  /* In SW Tag detection remain in ready let the varicaps settle  */
  else if (gRFAL.wum.cfg.swTagDetect) {
    delay(RFAL_ST25R3916_AAT_SETTLE_ON);
  } else {
    /* MISRA 15.7 - Empty else */
  }


  /*******************************************************************************/
  /* Prepare Wake-Up Timer Control Register */
  reg  = (uint8_t)(((uint8_t)gRFAL.wum.cfg.period & 0x0FU) << ST25R3916_REG_WUP_TIMER_CONTROL_wut_shift);
  reg |= (uint8_t)(((uint8_t)gRFAL.wum.cfg.period < (uint8_t)RFAL_WUM_PERIOD_100MS) ? ST25R3916_REG_WUP_TIMER_CONTROL_wur : 0x00U);

  if (gRFAL.wum.cfg.irqTout || gRFAL.wum.cfg.swTagDetect) {
    reg  |= ST25R3916_REG_WUP_TIMER_CONTROL_wto;
    irqs |= ST25R3916_IRQ_MASK_WT;
  }

  /*******************************************************************************/
  /* Check if Inductive Amplitude is to be performed */
  if (gRFAL.wum.cfg.indAmp.enabled) {
    aux  = (uint8_t)((gRFAL.wum.cfg.indAmp.delta) << ST25R3916_REG_AMPLITUDE_MEASURE_CONF_am_d_shift);
    aux |= (uint8_t)(gRFAL.wum.cfg.indAmp.aaInclMeas ? ST25R3916_REG_AMPLITUDE_MEASURE_CONF_am_aam : 0x00U);
    aux |= (uint8_t)(((uint8_t)gRFAL.wum.cfg.indAmp.aaWeight << ST25R3916_REG_AMPLITUDE_MEASURE_CONF_am_aew_shift) & ST25R3916_REG_AMPLITUDE_MEASURE_CONF_am_aew_mask);
    aux |= (uint8_t)(gRFAL.wum.cfg.indAmp.autoAvg ? ST25R3916_REG_AMPLITUDE_MEASURE_CONF_am_ae : 0x00U);

    st25r3916WriteRegister(ST25R3916_REG_AMPLITUDE_MEASURE_CONF, aux);

    /* Only need to set the reference if not using Auto Average */
    if (!gRFAL.wum.cfg.indAmp.autoAvg || gRFAL.wum.cfg.swTagDetect) {
      if (gRFAL.wum.cfg.indAmp.reference == RFAL_WUM_REFERENCE_AUTO) {
        st25r3916MeasureAmplitude(&gRFAL.wum.cfg.indAmp.reference);
      }
      st25r3916WriteRegister(ST25R3916_REG_AMPLITUDE_MEASURE_REF, gRFAL.wum.cfg.indAmp.reference);
    }

    reg  |= ST25R3916_REG_WUP_TIMER_CONTROL_wam;
    irqs |= ST25R3916_IRQ_MASK_WAM;
  }

  /*******************************************************************************/
  /* Check if Inductive Phase is to be performed */
  if (gRFAL.wum.cfg.indPha.enabled) {
    aux  = (uint8_t)((gRFAL.wum.cfg.indPha.delta) << ST25R3916_REG_PHASE_MEASURE_CONF_pm_d_shift);
    aux |= (uint8_t)(gRFAL.wum.cfg.indPha.aaInclMeas ? ST25R3916_REG_PHASE_MEASURE_CONF_pm_aam : 0x00U);
    aux |= (uint8_t)(((uint8_t)gRFAL.wum.cfg.indPha.aaWeight << ST25R3916_REG_PHASE_MEASURE_CONF_pm_aew_shift) & ST25R3916_REG_PHASE_MEASURE_CONF_pm_aew_mask);
    aux |= (uint8_t)(gRFAL.wum.cfg.indPha.autoAvg ? ST25R3916_REG_PHASE_MEASURE_CONF_pm_ae : 0x00U);

    st25r3916WriteRegister(ST25R3916_REG_PHASE_MEASURE_CONF, aux);

    /* Only need to set the reference if not using Auto Average */
    if (!gRFAL.wum.cfg.indPha.autoAvg || gRFAL.wum.cfg.swTagDetect) {
      if (gRFAL.wum.cfg.indPha.reference == RFAL_WUM_REFERENCE_AUTO) {
        st25r3916MeasurePhase(&gRFAL.wum.cfg.indPha.reference);

      }
      st25r3916WriteRegister(ST25R3916_REG_PHASE_MEASURE_REF, gRFAL.wum.cfg.indPha.reference);
    }

    reg  |= ST25R3916_REG_WUP_TIMER_CONTROL_wph;
    irqs |= ST25R3916_IRQ_MASK_WPH;
  }

  /*******************************************************************************/
  /* Check if Capacitive is to be performed */
  if (gRFAL.wum.cfg.cap.enabled) {
    /*******************************************************************************/
    /* Perform Capacitive sensor calibration */

    /* Disable Oscillator and Field */
    st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, (ST25R3916_REG_OP_CONTROL_en | ST25R3916_REG_OP_CONTROL_tx_en));

    /* Sensor gain should be configured on Analog Config: RFAL_ANALOG_CONFIG_CHIP_WAKEUP_ON */

    /* Perform calibration procedure */
    st25r3916CalibrateCapacitiveSensor(NULL);


    /*******************************************************************************/
    aux  = (uint8_t)((gRFAL.wum.cfg.cap.delta) << ST25R3916_REG_CAPACITANCE_MEASURE_CONF_cm_d_shift);
    aux |= (uint8_t)(gRFAL.wum.cfg.cap.aaInclMeas ? ST25R3916_REG_CAPACITANCE_MEASURE_CONF_cm_aam : 0x00U);
    aux |= (uint8_t)(((uint8_t)gRFAL.wum.cfg.cap.aaWeight << ST25R3916_REG_CAPACITANCE_MEASURE_CONF_cm_aew_shift) & ST25R3916_REG_CAPACITANCE_MEASURE_CONF_cm_aew_mask);
    aux |= (uint8_t)(gRFAL.wum.cfg.cap.autoAvg ? ST25R3916_REG_CAPACITANCE_MEASURE_CONF_cm_ae : 0x00U);

    st25r3916WriteRegister(ST25R3916_REG_CAPACITANCE_MEASURE_CONF, aux);

    /* Only need to set the reference if not using Auto Average */
    if (!gRFAL.wum.cfg.cap.autoAvg || gRFAL.wum.cfg.swTagDetect) {
      if (gRFAL.wum.cfg.indPha.reference == RFAL_WUM_REFERENCE_AUTO) {
        st25r3916MeasureCapacitance(&gRFAL.wum.cfg.cap.reference);
      }
      st25r3916WriteRegister(ST25R3916_REG_CAPACITANCE_MEASURE_REF, gRFAL.wum.cfg.cap.reference);
    }

    reg  |= ST25R3916_REG_WUP_TIMER_CONTROL_wcap;
    irqs |= ST25R3916_IRQ_MASK_WCAP;
  }


  /* Disable and clear all interrupts except Wake-Up IRQs */
  st25r3916DisableInterrupts(ST25R3916_IRQ_MASK_ALL);
  st25r3916GetInterrupt(irqs);
  st25r3916EnableInterrupts(irqs);

  /* On SW Tag Detection no HW automatic measurements are to be performed, only make use of WTO */
  if (gRFAL.wum.cfg.swTagDetect) {
    reg &= ~(ST25R3916_REG_WUP_TIMER_CONTROL_wam | ST25R3916_REG_WUP_TIMER_CONTROL_wph | ST25R3916_REG_WUP_TIMER_CONTROL_wcap);
  }

  /* Enable Low Power Wake-Up Mode (Disable: Oscilattor, Tx, Rx and External Field Detector)*/
  st25r3916WriteRegister(ST25R3916_REG_WUP_TIMER_CONTROL, reg);
  st25r3916ChangeRegisterBits(ST25R3916_REG_OP_CONTROL,
                              (ST25R3916_REG_OP_CONTROL_en | ST25R3916_REG_OP_CONTROL_rx_en | ST25R3916_REG_OP_CONTROL_tx_en |
                               ST25R3916_REG_OP_CONTROL_en_fd_mask | ST25R3916_REG_OP_CONTROL_wu),
                              ST25R3916_REG_OP_CONTROL_wu);


  gRFAL.wum.state = RFAL_WUM_STATE_ENABLED;
  gRFAL.state     = RFAL_STATE_WUM;

  return ERR_NONE;
}


/*******************************************************************************/
bool RfalRfST25R3916Class::rfalWakeUpModeHasWoke(void)
{
  return (gRFAL.wum.state >= RFAL_WUM_STATE_ENABLED_WOKE);
}


/*******************************************************************************/
void RfalRfST25R3916Class::rfalRunWakeUpModeWorker(void)
{
  uint32_t irqs;
  uint8_t  reg;

  if (gRFAL.state != RFAL_STATE_WUM) {
    return;
  }

  switch (gRFAL.wum.state) {
    case RFAL_WUM_STATE_ENABLED:
    case RFAL_WUM_STATE_ENABLED_WOKE:

      irqs = st25r3916GetInterrupt((ST25R3916_IRQ_MASK_WT | ST25R3916_IRQ_MASK_WAM | ST25R3916_IRQ_MASK_WPH | ST25R3916_IRQ_MASK_WCAP));
      if (irqs == ST25R3916_IRQ_MASK_NONE) {
        break;  /* No interrupt to process */
      }

      /*******************************************************************************/
      /* Check and mark which measurement(s) cause interrupt */
      if ((irqs & ST25R3916_IRQ_MASK_WAM) != 0U) {
        st25r3916ReadRegister(ST25R3916_REG_AMPLITUDE_MEASURE_RESULT, &reg);
        gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
      }

      if ((irqs & ST25R3916_IRQ_MASK_WPH) != 0U) {
        st25r3916ReadRegister(ST25R3916_REG_PHASE_MEASURE_RESULT, &reg);
        gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
      }

      if ((irqs & ST25R3916_IRQ_MASK_WCAP) != 0U) {
        st25r3916ReadRegister(ST25R3916_REG_CAPACITANCE_MEASURE_RESULT, &reg);
        gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
      }

      if ((irqs & ST25R3916_IRQ_MASK_WT) != 0U) {
        /*******************************************************************************/
        if (gRFAL.wum.cfg.swTagDetect) {
          /* Enable Ready mode */
          st25r3916ChangeRegisterBits(ST25R3916_REG_OP_CONTROL, (ST25R3916_REG_OP_CONTROL_en | ST25R3916_REG_OP_CONTROL_wu), (ST25R3916_REG_OP_CONTROL_en));
          delay(RFAL_ST25R3916_AAT_SETTLE_ON);


          if (gRFAL.wum.cfg.indAmp.enabled) {
            st25r3916MeasureAmplitude(&reg);
            if ((reg >= (gRFAL.wum.cfg.indAmp.reference + gRFAL.wum.cfg.indAmp.delta)) || (reg <= (gRFAL.wum.cfg.indAmp.reference - gRFAL.wum.cfg.indAmp.delta))) {
              gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
              break;
            }
          }

          if (gRFAL.wum.cfg.indPha.enabled) {
            st25r3916MeasurePhase(&reg);
            if ((reg >= (gRFAL.wum.cfg.indPha.reference + gRFAL.wum.cfg.indPha.delta)) || (reg <= (gRFAL.wum.cfg.indPha.reference - gRFAL.wum.cfg.indPha.delta))) {
              gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
              break;
            }
          }

          if (gRFAL.wum.cfg.cap.enabled) {
            st25r3916MeasureCapacitance(&reg);
            if ((reg >= (gRFAL.wum.cfg.cap.reference + gRFAL.wum.cfg.cap.delta)) || (reg <= (gRFAL.wum.cfg.cap.reference - gRFAL.wum.cfg.cap.delta))) {
              gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
              break;
            }
          }

          /* Re-Enable low power Wake-Up mode for wto to trigger another measurement(s) */
          st25r3916ChangeRegisterBits(ST25R3916_REG_OP_CONTROL, (ST25R3916_REG_OP_CONTROL_en | ST25R3916_REG_OP_CONTROL_wu), (ST25R3916_REG_OP_CONTROL_wu));
        }
      }
      break;

    default:
      /* MISRA 16.4: no empty default statement (a comment being enough) */
      break;
  }
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalWakeUpModeStop(void)
{
  if (gRFAL.wum.state == RFAL_WUM_STATE_NOT_INIT) {
    return ERR_WRONG_STATE;
  }

  gRFAL.wum.state = RFAL_WUM_STATE_NOT_INIT;

  /* Disable Wake-Up Mode */
  st25r3916ClrRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_wu);
  st25r3916DisableInterrupts((ST25R3916_IRQ_MASK_WT | ST25R3916_IRQ_MASK_WAM | ST25R3916_IRQ_MASK_WPH | ST25R3916_IRQ_MASK_WCAP));

  /* Re-Enable External Field Detector as: Automatics */
  st25r3916ChangeRegisterBits(ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_en_fd_mask, ST25R3916_REG_OP_CONTROL_en_fd_auto_efd);

  /* Re-Enable the Oscillator */
  st25r3916OscOn();

  /* Set Analog configurations for Wake-up Off event */
  rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_WAKEUP_OFF));

  return ERR_NONE;
}


/*******************************************************************************
 *  RF Chip                                                                    *
 *******************************************************************************/

/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipWriteReg(uint16_t reg, const uint8_t *values, uint8_t len)
{
  if (!st25r3916IsRegValid((uint8_t)reg)) {
    return ERR_PARAM;
  }

  return st25r3916WriteMultipleRegisters((uint8_t)reg, values, len);
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipReadReg(uint16_t reg, uint8_t *values, uint8_t len)
{
  if (!st25r3916IsRegValid((uint8_t)reg)) {
    return ERR_PARAM;
  }

  return st25r3916ReadMultipleRegisters((uint8_t)reg, values, len);
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipExecCmd(uint16_t cmd)
{
  if (!st25r3916IsCmdValid((uint8_t)cmd)) {
    return ERR_PARAM;
  }

  return st25r3916ExecuteCommand((uint8_t) cmd);
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipWriteTestReg(uint16_t reg, uint8_t value)
{
  return st25r3916WriteTestRegister((uint8_t)reg, value);
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipReadTestReg(uint16_t reg, uint8_t *value)
{
  return st25r3916ReadTestRegister((uint8_t)reg, value);
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipChangeRegBits(uint16_t reg, uint8_t valueMask, uint8_t value)
{
  if (!st25r3916IsRegValid((uint8_t)reg)) {
    return ERR_PARAM;
  }

  return st25r3916ChangeRegisterBits((uint8_t)reg, valueMask, value);
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipChangeTestRegBits(uint16_t reg, uint8_t valueMask, uint8_t value)
{
  st25r3916ChangeTestRegisterBits((uint8_t)reg, valueMask, value);
  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipSetRFO(uint8_t rfo)
{
  return st25r3916ChangeRegisterBits(ST25R3916_REG_TX_DRIVER, ST25R3916_REG_TX_DRIVER_d_res_mask, rfo);
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipGetRFO(uint8_t *result)
{
  ReturnCode ret;

  ret = st25r3916ReadRegister(ST25R3916_REG_TX_DRIVER, result);

  (*result) = ((*result) & ST25R3916_REG_TX_DRIVER_d_res_mask);

  return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipMeasureAmplitude(uint8_t *result)
{
  ReturnCode err;
  uint8_t reg_opc, reg_mode, reg_conf1, reg_conf2;

  /* Save registers which will be adjusted below */
  st25r3916ReadRegister(ST25R3916_REG_OP_CONTROL, &reg_opc);
  st25r3916ReadRegister(ST25R3916_REG_MODE, &reg_mode);
  st25r3916ReadRegister(ST25R3916_REG_RX_CONF1, &reg_conf1);
  st25r3916ReadRegister(ST25R3916_REG_RX_CONF2, &reg_conf2);

  /* Set values as per defaults of DS. These regs/bits influence receiver chain and change amplitude */
  /* Doing so achieves an amplitude comparable over a complete polling cylce */
  st25r3916WriteRegister(ST25R3916_REG_OP_CONTROL, (reg_opc & ~ST25R3916_REG_OP_CONTROL_rx_chn));
  st25r3916WriteRegister(ST25R3916_REG_MODE, ST25R3916_REG_MODE_om_iso14443a
                         | ST25R3916_REG_MODE_targ_init
                         | ST25R3916_REG_MODE_tr_am_ook
                         | ST25R3916_REG_MODE_nfc_ar_off);
  st25r3916WriteRegister(ST25R3916_REG_RX_CONF1, (reg_conf1 & ~ST25R3916_REG_RX_CONF1_ch_sel_AM));
  st25r3916WriteRegister(ST25R3916_REG_RX_CONF2, ((reg_conf2 & ~(ST25R3916_REG_RX_CONF2_demod_mode | ST25R3916_REG_RX_CONF2_amd_sel))
                                                  | ST25R3916_REG_RX_CONF2_amd_sel_peak));

  /* Perform the actual measurement */
  err = st25r3916MeasureAmplitude(result);

  /* Restore values */
  st25r3916WriteRegister(ST25R3916_REG_OP_CONTROL, reg_opc);
  st25r3916WriteRegister(ST25R3916_REG_MODE, reg_mode);
  st25r3916WriteRegister(ST25R3916_REG_RX_CONF1, reg_conf1);
  st25r3916WriteRegister(ST25R3916_REG_RX_CONF2, reg_conf2);

  return err;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipMeasurePhase(uint8_t *result)
{
  st25r3916MeasurePhase(result);

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipMeasureCapacitance(uint8_t *result)
{
  st25r3916MeasureCapacitance(result);

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3916Class::rfalChipMeasurePowerSupply(uint8_t param, uint8_t *result)
{
  *result = st25r3916MeasurePowerSupply(param);

  return ERR_NONE;
}

void RfalRfST25R3916Class::setISRPending(void)
{
  isr_pending = true;
}

bool RfalRfST25R3916Class::isBusBusy(void)
{
  return bus_busy;
}


/*******************************************************************************/
extern uint8_t invalid_size_of_stream_configs[(sizeof(struct st25r3916StreamConfig) == sizeof(struct iso15693StreamConfig)) ? 1 : (-1)];
