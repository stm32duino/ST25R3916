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
 *  \brief ST25R3916 Interrupt handling
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "rfal_rfst25r3916.h"
#include "st25r3916_interrupt.h"
#include "st25r3916_com.h"
#include "st25r3916.h"
#include "st_errno.h"
#include "nfc_utils.h"

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

/*! Length of the interrupt registers       */
#define ST25R3916_INT_REGS_LEN          ( (ST25R3916_REG_IRQ_TARGET - ST25R3916_REG_IRQ_MAIN) + 1U )

/*
 ******************************************************************************
 * LOCAL DATA TYPES
 ******************************************************************************
 */


/*
******************************************************************************
* GLOBAL VARIABLES
******************************************************************************
*/


/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
void RfalRfST25R3916Class::st25r3916InitInterrupts(void)
{
  st25r3916interrupt.callback     = NULL;
  st25r3916interrupt.prevCallback = NULL;
  st25r3916interrupt.status       = ST25R3916_IRQ_MASK_NONE;
  st25r3916interrupt.mask         = ST25R3916_IRQ_MASK_NONE;
}


/*******************************************************************************/
void RfalRfST25R3916Class::st25r3916Isr(void)
{
  if (!isBusBusy()) {
    st25r3916CheckForReceivedInterrupts();

    // Check if callback is set and run it
    if (NULL != st25r3916interrupt.callback) {
      st25r3916interrupt.callback();
    }
  } else {
    setISRPending();
  }
}


/*******************************************************************************/
void RfalRfST25R3916Class::st25r3916CheckForReceivedInterrupts(void)
{
  uint8_t  iregs[ST25R3916_INT_REGS_LEN];
  uint32_t irqStatus;

  /* Initialize iregs */
  irqStatus = ST25R3916_IRQ_MASK_NONE;
  ST_MEMSET(iregs, (int32_t)(ST25R3916_IRQ_MASK_ALL & 0xFFU), ST25R3916_INT_REGS_LEN);


  /* In case the IRQ is Edge (not Level) triggered read IRQs until done */
  while (digitalRead(int_pin) == HIGH) {
    st25r3916ReadMultipleRegisters(ST25R3916_REG_IRQ_MAIN, iregs, ST25R3916_INT_REGS_LEN);

    irqStatus |= (uint32_t)iregs[0];
    irqStatus |= (uint32_t)iregs[1] << 8;
    irqStatus |= (uint32_t)iregs[2] << 16;
    irqStatus |= (uint32_t)iregs[3] << 24;
  }

  /* Forward all interrupts, even masked ones to application */
  st25r3916interrupt.status |= irqStatus;
}


/*******************************************************************************/
void RfalRfST25R3916Class::st25r3916ModifyInterrupts(uint32_t clr_mask, uint32_t set_mask)
{
  uint8_t  i;
  uint32_t old_mask;
  uint32_t new_mask;


  old_mask = st25r3916interrupt.mask;
  new_mask = ((~old_mask & set_mask) | (old_mask & clr_mask));
  st25r3916interrupt.mask &= ~clr_mask;
  st25r3916interrupt.mask |= set_mask;

  for (i = 0; i < ST25R3916_INT_REGS_LEN; i++) {
    if (((new_mask >> (8U * i)) & 0xFFU) == 0U) {
      continue;
    }

    st25r3916WriteRegister(ST25R3916_REG_IRQ_MASK_MAIN + i, (uint8_t)((st25r3916interrupt.mask >> (8U * i)) & 0xFFU));
  }
  return;
}


/*******************************************************************************/
uint32_t RfalRfST25R3916Class::st25r3916WaitForInterruptsTimed(uint32_t mask, uint16_t tmo)
{
  uint32_t tmrDelay;
  uint32_t status;

  tmrDelay = timerCalculateTimer(tmo);

  /* Run until specific interrupt has happen or the timer has expired */
  do {
    status = (st25r3916interrupt.status & mask);
  } while ((!timerIsExpired(tmrDelay) || (tmo == 0U)) && (status == 0U));

  status = st25r3916interrupt.status & mask;

  st25r3916interrupt.status &= ~status;

  return status;
}


/*******************************************************************************/
uint32_t RfalRfST25R3916Class::st25r3916GetInterrupt(uint32_t mask)
{
  uint32_t irqs;

  irqs = (st25r3916interrupt.status & mask);
  if (irqs != ST25R3916_IRQ_MASK_NONE) {
    st25r3916interrupt.status &= ~irqs;
  }

  return irqs;
}


/*******************************************************************************/
void RfalRfST25R3916Class::st25r3916ClearAndEnableInterrupts(uint32_t mask)
{
  st25r3916GetInterrupt(mask);
  st25r3916EnableInterrupts(mask);
}


/*******************************************************************************/
void RfalRfST25R3916Class::st25r3916EnableInterrupts(uint32_t mask)
{
  st25r3916ModifyInterrupts(mask, 0);
}


/*******************************************************************************/
void RfalRfST25R3916Class::st25r3916DisableInterrupts(uint32_t mask)
{
  st25r3916ModifyInterrupts(0, mask);
}

/*******************************************************************************/
void RfalRfST25R3916Class::st25r3916ClearInterrupts(void)
{
  uint8_t iregs[ST25R3916_INT_REGS_LEN];

  st25r3916ReadMultipleRegisters(ST25R3916_REG_IRQ_MAIN, iregs, ST25R3916_INT_REGS_LEN);

  st25r3916interrupt.status = ST25R3916_IRQ_MASK_NONE;

  return;
}

/*******************************************************************************/
void RfalRfST25R3916Class::st25r3916IRQCallbackSet(void (*cb)(void))
{
  st25r3916interrupt.prevCallback = st25r3916interrupt.callback;
  st25r3916interrupt.callback     = cb;
}

/*******************************************************************************/
void RfalRfST25R3916Class::st25r3916IRQCallbackRestore(void)
{
  st25r3916interrupt.callback     = st25r3916interrupt.prevCallback;
  st25r3916interrupt.prevCallback = NULL;
}
