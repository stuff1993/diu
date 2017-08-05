/*
 * can.c
 *
 *  Created on: 28 Apr 2015
 *      Author: Stuart G
 */

#include "lpc17xx.h"
#include "type.h"
#include "can.h"
#include "dash.h"
#include "struct.h"
#include "inttofloat.h"

CAN_MSG can_rx1_buf;
CAN_MSG can_rx2_buf;
extern volatile uint8_t can_rx1_done;

volatile uint32_t CANStatus;
#if CAN_RX_COUNTERS
volatile uint32_t CAN1RxCount = 0;
volatile uint32_t CAN2RxCount = 0;
#endif
#if CAN_TX_COUNTERS
volatile uint32_t CAN1TxCount = 0;
volatile uint32_t CAN2TxCount = 0;
#endif
#if CAN_ERR_COUNTERS
uint16_t CAN1ErrCount = 0;
uint16_t CAN2ErrCount = 0;
#endif

#if CAN_WAKEUP
volatile uint32_t CANActivityInterruptFlag = 0;
#endif


/// Default unpacker
__attribute__ ((weak)) void default_can_unpacker(CAN_MSG *_msg);

/// Must implement
void can1_unpack(CAN_MSG *_msg) __attribute__ ((weak, alias ("default_can_unpacker")));
void can2_unpack(CAN_MSG *_msg) __attribute__ ((weak, alias ("default_can_unpacker")));

/******************************************************************************
 ** Function name: CAN_ISR_Rx1
 **
 ** Description:   CAN Rx1 interrupt handler
 **
 ** Parameters:      None
 ** Returned value:  None
 **
 ******************************************************************************/
void CAN_ISR_Rx1(void)
{
	uint32_t * pDest = (uint32_t *) &can_rx1_buf;

	*pDest = LPC_CAN1->RFS; // Frame
	pDest++;

	*pDest = LPC_CAN1->RID; // ID
	pDest++;

	*pDest = LPC_CAN1->RDA; // DataA
	pDest++;

	*pDest = LPC_CAN1->RDB; // DataB
	pDest++;

	can1_unpack(&can_rx1_buf);
	LPC_CAN1->CMR = 0x4; // Release Receive Buffer
}

/******************************************************************************
 ** Function name:   CAN_ISR_Rx2
 **
 ** Description:     CAN Rx2 interrupt handler
 **
 ** Parameters:      None
 ** Returned value:  None
 **
 ******************************************************************************/
void CAN_ISR_Rx2(void)
{
	uint32_t *pDest;

	/* initialize destination pointer	*/
	pDest = (uint32_t *) &can_rx2_buf;
	*pDest = LPC_CAN2->RFS; /* Frame	*/

	pDest++;
	*pDest = LPC_CAN2->RID; /* ID	*/

	pDest++;
	*pDest = LPC_CAN2->RDA; /* Data A	*/

	pDest++;
	*pDest = LPC_CAN2->RDB; /* Data B	*/

	can2_unpack(&can_rx2_buf);
	LPC_CAN2->CMR = 0x4; /* release receive buffer */
	return;
}

/*****************************************************************************
 ** Function name:   CAN_Handler
 **
 ** Descriptions:    CAN interrupt handler
 **
 ** parameters:      None
 ** Returned value:  None
 **
 *****************************************************************************/
void CAN_IRQHandler(void)
{
	CANStatus = LPC_CANCR->CANRxSR;
	if (CANStatus & (1 << 8))
	{
#if CAN_RX_COUNTERS
		CAN1RxCount++;
#endif
		CAN_ISR_Rx1();
	}
	if (CANStatus & (1 << 9))
	{
#if CAN_RX_COUNTERS
		CAN2RxCount++;
#endif
		CAN_ISR_Rx2();
	}
#if CAN_ERR_COUNTERS
	if ( LPC_CAN1->GSR & (1 << 6))
	{
		// The error count includes both TX and RX
		CAN1ErrCount = LPC_CAN1->GSR >> 16;
	}
	if ( LPC_CAN2->GSR & (1 << 6))
	{
		// The error count includes both TX and RX
		CAN2ErrCount = LPC_CAN2->GSR >> 16;
	}
#endif
	return;
}

#if CAN_WAKEUP
/******************************************************************************
 ** Function name:   CANActivity_IRQHandler
 **
 ** Descriptions:    Wake up from CAN handler
 **
 ** parameters:      None
 ** Returned value:  None
 **
 ******************************************************************************/
void CANActivity_IRQHandler (void)
{
	can_rx2_done = TRUE;
	CANActivityInterruptFlag = 1;

	LPC_SC->CANSLEEPCLR = (0x1<<1)|(0x1<<2);
	LPC_CAN1->MOD = LPC_CAN2->MOD &= ~(0x1<<4);
	LPC_SC->CANWAKEFLAGS = (0x1<<1)|(0x1<<2);
	return;
}
#endif

/******************************************************************************
 ** Function name:   can1_init
 **
 ** Descriptions:    Initialize CAN, install CAN interrupt handler
 **
 ** parameters:      bitrate
 ** Returned value:  true or false, false if initialization failed.
 **
 ******************************************************************************/
uint32_t can1_init(uint32_t can_btr)
{
	can_rx1_done = FALSE;

	LPC_SC->PCONP |= (1 << 13); /* Enable CAN1 clock */

	LPC_PINCON->PINSEL1 |= (1 << 13) | (1 << 12) | (1 << 11) | (1 << 10);

	LPC_CAN1->MOD = 1; /* Reset CAN */
	LPC_CAN1->IER = 0; /* Disable Receive Interrupt */
	LPC_CAN1->GSR = 0; /* Reset error counter when CANxMOD is in reset	*/

	LPC_CAN1->BTR = can_btr;
	LPC_CAN1->MOD = 0x0; /* CAN in normal operation mode */

	NVIC_EnableIRQ(CAN_IRQn);

	LPC_CAN1->IER = 0x01; /* Enable receive interrupts */
	return ( TRUE);
}

/******************************************************************************
 ** Function name:   can2_init
 **
 ** Descriptions:    Initialize CAN, install CAN interrupt handler
 **
 ** parameters:      bitrate
 ** Returned value:  true or false, false if initialization failed.
 **
 ******************************************************************************/
uint32_t can2_init(uint32_t can_btr)
{
	LPC_SC->PCONP |= (1 << 14); /* Enable CAN2 clock */

	LPC_PINCON->PINSEL0 |= (1 << 9) | (1 << 11);

	LPC_CAN2->MOD = 1; /* Reset CAN */
	LPC_CAN2->IER = 0; /* Disable Receive Interrupt */
	LPC_CAN2->GSR = 0; /* Reset error counter when CANxMOD is in reset	*/

	LPC_CAN2->BTR = can_btr;
	LPC_CAN2->MOD = 0x0; /* CAN in normal operation mode */

	NVIC_EnableIRQ(CAN_IRQn);

	LPC_CAN2->IER = 0x01; /* Enable receive interrupts */
	return ( TRUE);
}

/******************************************************************************
 ** Function name:   CAN_SetACCF_Lookup
 **
 ** Descriptions:    Initialize CAN, install CAN interrupt handler
 **
 ** parameters:      bitrate
 ** Returned value:  true or false, false if initialization failed.
 **
 ******************************************************************************/
void CAN_SetACCF_Lookup(void)
{
  uint32_t address = 0;
  uint32_t i;
  uint32_t ID_high, ID_low;

  /* Set explicit standard Frame */
  LPC_CANAF->SFF_sa = address;
  for ( i = 0; i < ACCF_IDEN_NUM; i += 2 )
  {
  ID_low = (i << 29) | (MPPT1_BASE << 16);
  ID_high = ((i+1) << 13) | (MPPT1_BASE << 0);
  *((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low | ID_high;
  address += 4;
  }

  /* Set group standard Frame */
  LPC_CANAF->SFF_GRP_sa = address;
  for ( i = 0; i < ACCF_IDEN_NUM; i += 2 )
  {
  ID_low = (i << 29) | (GRP_STD_ID << 16);
  ID_high = ((i+1) << 13) | (GRP_STD_ID << 0);
  *((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low | ID_high;
  address += 4;
  }

  /* Set explicit extended Frame */
  LPC_CANAF->EFF_sa = address;
  for ( i = 0; i < ACCF_IDEN_NUM; i++  )
  {
  ID_low = (i << 29) | (EXP_EXT_ID << 0);
  *((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low;
  address += 4;
  }

  /* Set group extended Frame */
  LPC_CANAF->EFF_GRP_sa = address;
  for ( i = 0; i < ACCF_IDEN_NUM; i++  )
  {
  ID_low = (i << 29) | (GRP_EXT_ID << 0);
  *((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low;
  address += 4;
  }

  /* Set End of Table */
  LPC_CANAF->ENDofTable = address;
  return;
}

/******************************************************************************
** Function name:   CAN_SetACCF
**
** Descriptions:    Set acceptance filter and SRAM associated with
**
** parameters:      ACMF mode
** Returned value:  None
**
**
******************************************************************************/
void CAN_SetACCF( uint32_t ACCFMode )
{
  switch ( ACCFMode )
  {
  case ACCF_OFF:
    LPC_CANAF->AFMR = ACCFMode;
    LPC_CAN1->MOD = LPC_CAN2->MOD = 1;  // Reset CAN
    LPC_CAN1->IER = LPC_CAN2->IER = 0;  // Disable Receive Interrupt
    LPC_CAN1->GSR = LPC_CAN2->GSR = 0;  // Reset error counter when CANxMOD is in reset
  break;

  case ACCF_BYPASS:
    LPC_CANAF->AFMR = ACCFMode;
  break;

  case ACCF_ON:
  case ACCF_FULLCAN:
    LPC_CANAF->AFMR = ACCF_OFF;
    CAN_SetACCF_Lookup();
    LPC_CANAF->AFMR = ACCFMode;
  break;

  default:
  break;
  }
  return;
}

/******************************************************************************
** Function name:   can1_send_message
**
** Descriptions:    Send message block to CAN1
**
** parameters:      pointer to the CAN message
** Returned value:  true or false, if message buffer is available,
**                  message can be sent successfully, return TRUE,
**                  otherwise, return FALSE.
**
******************************************************************************/
uint32_t can1_send_message( CAN_MSG *pTxBuf )
{
  uint32_t CANStatus;

#if CAN_TX_COUNTERS
  CAN1TxCount++;
#endif
  CANStatus = LPC_CAN1->SR;
  if ( CANStatus & 0x00000004 )
  {
  LPC_CAN1->TFI1 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN1->TID1 = pTxBuf->MsgID;
  LPC_CAN1->TDA1 = pTxBuf->DataA;
  LPC_CAN1->TDB1 = pTxBuf->DataB;
  LPC_CAN1->CMR |= 0x21;
  return ( TRUE );
  }
  else if ( CANStatus & 0x00000400 )
  {
  LPC_CAN1->TFI2 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN1->TID2 = pTxBuf->MsgID;
  LPC_CAN1->TDA2 = pTxBuf->DataA;
  LPC_CAN1->TDB2 = pTxBuf->DataB;
  LPC_CAN1->CMR |= 0x41;
  return ( TRUE );
  }
  else if ( CANStatus & 0x00040000 )
  {
  LPC_CAN1->TFI3 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN1->TID3 = pTxBuf->MsgID;
  LPC_CAN1->TDA3 = pTxBuf->DataA;
  LPC_CAN1->TDB3 = pTxBuf->DataB;
  LPC_CAN1->CMR |= 0x81;
  return ( TRUE );
  }
  return ( FALSE );
}

/******************************************************************************
** Function name:   can2_send_message
**
** Descriptions:    Send message block to CAN2
**
** parameters:      pointer to the CAN message
** Returned value:  true or false, if message buffer is available,
**                  message can be sent successfully, return TRUE,
**                  otherwise, return FALSE.
**
******************************************************************************/
uint32_t can2_send_message( CAN_MSG *pTxBuf )
{
  uint32_t CANStatus;

#if CAN_TX_COUNTERS
  CAN2TxCount++;
#endif
  CANStatus = LPC_CAN2->SR;
  if ( CANStatus & 0x00000004 )
  {
  LPC_CAN2->TFI1 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN2->TID1 = pTxBuf->MsgID;
  LPC_CAN2->TDA1 = pTxBuf->DataA;
  LPC_CAN2->TDB1 = pTxBuf->DataB;
  LPC_CAN2->CMR |= 0x21;
  return ( TRUE );
  }
  else if ( CANStatus & 0x00000400 )
  {
  LPC_CAN2->TFI2 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN2->TID2 = pTxBuf->MsgID;
  LPC_CAN2->TDA2 = pTxBuf->DataA;
  LPC_CAN2->TDB2 = pTxBuf->DataB;
  LPC_CAN2->CMR |= 0x41;
  return ( TRUE );
  }
  else if ( CANStatus & 0x00040000 )
  {
  LPC_CAN2->TFI3 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN2->TID3 = pTxBuf->MsgID;
  LPC_CAN2->TDA3 = pTxBuf->DataA;
  LPC_CAN2->TDB3 = pTxBuf->DataB;
  LPC_CAN2->CMR |= 0x81;
  return ( TRUE );
  }
  return ( FALSE );
}

__attribute__ ((section(".after_vectors")))
void default_can_unpacker(CAN_MSG *_msg)
{ if(1) {}
}
