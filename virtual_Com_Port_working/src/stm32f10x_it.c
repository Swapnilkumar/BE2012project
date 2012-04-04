 #define STM32F10X_CL


/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : stm32f10x_it.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Main Interrupt Service Routines.
*                      This file provides template for all exceptions handler
*                      and peripherals interrupt service routine.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "hw_config.h"
#include "platform_config.h"
#include "stm32_eval.h"
#include "symtab.h"
#include "printf.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/

/* end address for the .data section. defined in linker script */
extern unsigned long _edata;
extern unsigned long  _etext;
extern unsigned end;
/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/*******************************************************************************
* Function Name  : NMI_Handler
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMI_Handler(void)
{
}

/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : MemManage_Handler
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned *lowestaddr = &end;

void MemManage_Handler(void)
{
	extern unsigned end, end_of_ram;
	unsigned *src, *dest, *xnaddr, *link_reg_addr, *xnaddr1, *stack_pointer;
	int func_size, offset;
	int i,j;

	asm(
	"TST LR, #4;"
	"ITE EQ;"
	"MRSEQ R0, MSP;"
	"MRSNE R0, PSP;"
	);

	asm("mov %0, r0" : "=r"(stack_pointer));			
	xnaddr = (unsigned *) stack_pointer[15];
	xnaddr1 = (unsigned *) stack_pointer[16];
	offset = ((int)xnaddr)-(int)0xc0000000;
		

	if( ((int)xnaddr1 | 1) == 0xc0000001)
	{

		xnaddr1  = stack_pointer[17];	
		i=0;
		while( &_edata > &symbols[i])
		{
			if ( symbols[i].usedbit == 1 )
			{
				if ( xnaddr1 > symbols[i].load_time_address )
					if(xnaddr1 < symbols[i].load_time_address + symbols[i].fun_size)
					{
						symbols[i].usedbit = 0;
						break;
					}
			}
			i++;
		}
		

		stack_pointer[16] = &_etext;
                stack_pointer[18] = &_etext;
		return;	
	}
//#ifdef SYMTAB
	i = 0;
	while( &_edata > &symbols[i])
	{
		if ( (symbols[i].fun_start_address | 0x01) == ( (int)xnaddr | 1) )
		{
			func_size = symbols[i].fun_size;
			break;
		}
		i++;
	}
//#endif
	/* check if section is already preset in the RAM */
	if ( symbols[i].presentbit == 0 )
	{

		if ( (lowestaddr + func_size) < stack_pointer )
		{
			symbols[i].load_time_address =(uint32_t) lowestaddr;
			dest = lowestaddr;

		
			/*  copy required section in RAM */
			for( src = &_etext+(offset/4); 
				src <= &_etext +(func_size/4)+(offset/4); 
					src++, dest++)
			{	
				*dest=*src;
			}

			symbols[i].presentbit = 1;
			symbols[i].usedbit = 1;	
			link_reg_addr = lowestaddr;

			lowestaddr=dest;

		}
		else
		{
			/* Replace the section which is not used i.e. usedbit == 0 */
			j=i;
			i=0;
			while( &_edata > &symbols[i])
			{
				if ( (symbols[i].presentbit == 1) && (symbols[i].usedbit == 0) )
				{
					if ( func_size <= symbols[i].fun_size  )
					{
						symbols[i].presentbit = 0;
						symbols[j].load_time_address =(uint32_t) symbols[i].load_time_address;
			                        dest = symbols[i].load_time_address;


                        			/*  copy required section in RAM */
			                        for( src = &_etext+(offset/4);
                        		        	src <= &_etext +(func_size/4)+(offset/4);
                                        			src++, dest++)
			                        {
                        			        *dest=*src;
                        			}

			                        symbols[j].presentbit = 1;
                        			symbols[j].usedbit = 1;
			                        link_reg_addr = symbols[j].load_time_address;
						break;
					}
				}
				i++;
			}
		
			
		}

	}
	else
	{
		//symbols[i].load_time_address =(uint32_t) lowestaddr;
		dest = symbols[i].load_time_address;

		
		/*  copy required section in RAM */
		for( src = &_etext+(offset/4); 
			src <= &_etext +(func_size/4)+(offset/4); 
				src++, dest++)
		{	
			*dest=*src;
		}

		symbols[i].presentbit = 1;
		symbols[i].usedbit = 1;	
		//lowestaddr=dest;

		/* section is already in RAM */
		link_reg_addr = symbols[i].load_time_address;

	}
	//printf("\r\nValue of PC : 0x%x\r\n", link_reg_addr);

	      asm(
	"TST LR, #4;"
	"ITE EQ;"
	"MRSEQ R0, MSP;"
	"MRSNE R0, PSP;"
	);

	/* save link register value */
	asm("mov %0, r0" : "=r"(stack_pointer));			
		stack_pointer[15] = link_reg_addr;
		stack_pointer[18] = link_reg_addr;
	
	return;	

}


/*
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
/*  while (1)
  {
  }
}
*/

/*******************************************************************************
* Function Name  : BusFault_Handler
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : UsageFault_Handler
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : SVC_Handler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVC_Handler(void)
{
}

/*******************************************************************************
* Function Name  : DebugMon_Handler
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMon_Handler(void)
{
}

/*******************************************************************************
* Function Name  : PendSV_Handler
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSV_Handler(void)
{
}

/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

#ifndef STM32F10X_CL
/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  USB_Istr();
}
#endif /* STM32F10X_CL */

/*******************************************************************************
* Function Name  : EVAL_COM1_IRQHandler
* Description    : This function handles EVAL_COM1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EVAL_COM1_IRQHandler(void)
{
  if (USART_GetITStatus(EVAL_COM1, USART_IT_RXNE) != RESET)
  {
    /* Send the received data to the PC Host*/
    USART_To_USB_Send_Data();
  }

  /* If overrun condition occurs, clear the ORE flag and recover communication */
  if (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_ORE) != RESET)
  {
    (void)USART_ReceiveData(EVAL_COM1);
  }
}


#ifdef STM32F10X_CL
/*******************************************************************************
* Function Name  : OTG_FS_IRQHandler
* Description    : This function handles USB-On-The-Go FS global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OTG_FS_IRQHandler(void)
{
  STM32_PCD_OTG_ISR_Handler(); 
}
#endif /* STM32F10X_CL */

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/*******************************************************************************
* Function Name  : PPP_IRQHandler
* Description    : This function handles PPP interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void PPP_IRQHandler(void)
{
}*/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

