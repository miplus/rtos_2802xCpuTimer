// TI File $Revision: /main/5 $
// Checkin $Date: October 6, 2010   14:42:18 $
//###########################################################################
//
// FILE:    Example_2802xCpuTimer.c
//
// TITLE:   f2802x Device Getting Started Program.
//
// ASSUMPTIONS:
//
//
// DESCRIPTION:
//
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#include "FreeRTOS.h"
#include "task.h"


extern void vTaskDelay( portTickType xTicksToDelay );
extern void vPortPreemptiveTickISR(void);
extern void vPortCooperativeTickISR(void);
extern void vPortYieldTrap(void);


// Prototype statements for functions found within this file.
__interrupt void adc_isr(void);	//MP adc integration
void Adc_Config(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);

__interrupt void sciaTxFifoIsr(void);
__interrupt void sciaRxFifoIsr(void);
void scia_fifo_init(void);
void error(void);

void BackgroundTask(void *pvParameters);
void CounterTask(void *pvParameters);
void BlinkLEDTask(void *pvParameters);
unsigned int TaskCounter;

/* needed FreeRTOS hooks - dummy hooks*/
void vApplicationMallocFailedHook( void ){}
void vApplicationIdleHook(void){}

/* assert function called by RTOS - not very useful*/
Uint16 AssertCtr = 0;
void appASSERT(){
	AssertCtr++;
}

/*
 * This function is called when vTaskStartScheduler() is executed.
 */
void vApplicationSetupTimerInterrupt(void){
	// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM
}

/*
 * This function is called at each kernel tick and handles interrupt timer related flags.
 */
void vApplicationTickHook(void){
	   CpuTimer2.InterruptCount++;
}

// Global variables used in this example:
Uint16 LoopCount;
/*! ADC varables*/
Uint16 ConversionCount;
Uint16 Voltage1[10];
Uint16 Voltage2[10];

/*! task variables*/
Uint16 ACounter = 0;
int32 LongVariable = 0;

/*! SCI variables*/
uint16_t sdataA[2];    // Send data for SCI-A
uint16_t rdataA[2];    // Received data for SCI-A
uint16_t rdata_pointA; // Used for checking the received data

void main(void)
{
uint16_t i; //Temp variable for SCI

// WARNING: Always ensure you call memcpy before running any functions from RAM
// InitSysCtrl includes a call to a RAM based function and without a call to
// memcpy first, the processor will go "into the weeds"
   #ifdef _FLASH
	   memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
   #endif

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the f2802x_SysCtrl.c file.
   InitSysCtrl();


   TaskCounter = 0;
// Step 2. Initalize GPIO:
// This example function is found in the f2802x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
   InitGpio();  // Added for SCI /*MP*/
   InitSciaGpio();
   // Configure GPIO 0-3 as outputs
   EALLOW;
   GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO0=1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO1=1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO2=1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO3=1;
   EDIS;

   // Configure GPIO 0-3 initial level
//   GpioDataRegs.GPASET.bit.GPIO0 = 1;
   GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
   GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
//   GpioDataRegs.GPASET.bit.GPIO2 = 1;
   GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
//   GpioDataRegs.GPASET.bit.GPIO3 = 1;
   GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the f2802x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in f2802x_DefaultIsr.c.
// This function is found in f2802x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.ADCINT1 = &adc_isr; /*MP ADC*/
   PieVectTable.TINT0 = &cpu_timer0_isr;
   PieVectTable.TINT1 = &cpu_timer1_isr;
//SCIA interrupts
   PieVectTable.SCIRXINTA = &sciaRxFifoIsr;
   PieVectTable.SCITXINTA = &sciaTxFifoIsr;
//   PieVectTable.TINT2 = &cpu_timer2_isr; /*FreeRTOS port*/
	#if configUSE_PREEMPTION == 1
		PieVectTable.TINT2 = &vPortPreemptiveTickISR; //&Cpu0TimerInterrupt;
	#else
		PieVectTable.TINT2 = &vPortCooperativeTickISR; //&Cpu0TimerInterrupt;
	#endif
	PieVectTable.USER12 = &vPortYieldTrap;
  EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the Device Peripheral. This function can be
//         found in f2802x_CpuTimers.c
//  InitAdc();  // For this example, init the ADC /*! MP bug function wrong calling hierarchy!*/

    scia_fifo_init();  // Init SCI-A

#if 1 == 2 /* this part executed above in InitSysCtrl */
//  EALLOW;
//  SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
//  (*Device_cal)();
//  EDIS;
#endif
  EALLOW;
  AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG
  AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
  AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC
  AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
  AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select interal BG
  EDIS;




  InitCpuTimers();   // For this example, only initialize the Cpu Timers

#if (CPU_FRQ_60MHZ)
// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 60MHz CPU Freq, 1 second Period (in uSeconds)

   ConfigCpuTimer(&CpuTimer0, 60, 1000000);
   ConfigCpuTimer(&CpuTimer1, 60, 1000000);
//   ConfigCpuTimer(&CpuTimer2, 60, 1000000);
/*! timer 2 configured period 1 mili second*/
   ConfigCpuTimer(&CpuTimer2, 60, 10000); /*FreeRTOS port*/
#endif
#if (CPU_FRQ_50MHZ)
// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 50MHz CPU Freq, 1 second Period (in uSeconds)

   ConfigCpuTimer(&CpuTimer0, 50, 1000000);
   ConfigCpuTimer(&CpuTimer1, 50, 1000000);
   ConfigCpuTimer(&CpuTimer2, 50, 1000000);
#endif
#if (CPU_FRQ_40MHZ)
// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 40MHz CPU Freq, 1 second Period (in uSeconds)

   ConfigCpuTimer(&CpuTimer0, 40, 1000000);
   ConfigCpuTimer(&CpuTimer1, 40, 1000000);
   ConfigCpuTimer(&CpuTimer2, 40, 1000000);
#endif
// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in f2802x_CpuTimers.h), the
// below settings must also be updated.

   CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
   CpuTimer1Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
   CpuTimer2Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0 /*FreeRTOS port*/

// Step 5. User specific code, enable interrupts:

   // Init send data.  After each transmission this data
   // will be updated for the next transmission
      for(i = 0; i<2; i++)
      {
         sdataA[i] = i;
      }

      rdata_pointA = sdataA[0];
   // Enable interrupts required for this example
   //   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
      PieCtrlRegs.PIEIER9.bit.INTx1=1;     // PIE Group 9, INT1
      PieCtrlRegs.PIEIER9.bit.INTx2=1;     // PIE Group 9, INT2

// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
// which is connected to CPU-Timer 1, and CPU int 14, which is connected
// to CPU-Timer 2:
   IER |= M_INT1;	//	CPU-Timer 0 PIE: Group 1 interrupt 7
   IER |= M_INT9;	// SCI-A RX and TX int
   IER |= M_INT13;	//	CPU-Timer 1
   IER |= M_INT14;	//	CPU-Timer 2	/*FreeRTOS port*/

// Enable ADCINT1 in PIE
   PieCtrlRegs.PIEIER1.bit.INTx1 = 1;	// Enable INT 1.1 in the PIE
// Enable TINT0 in the PIE: Group 1 interrupt 7
  PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

// Enable global Interrupts and higher priority real-time debug events:
     EINT;   // Enable Global interrupt INTM
     ERTM;   // Enable Global realtime interrupt DBGM

   // Configure ADC
   LoopCount = 0;
   ConversionCount = 0;
   // Configure ADC

//Note: Channel ADCINA4  will be double sampled to workaround the ADC 1st sample issue for rev0 silicon errata
   	EALLOW;
   	AdcRegs.ADCCTL1.bit.INTPULSEPOS	= 1;	//ADCINT1 trips after AdcResults latch
   	AdcRegs.INTSEL1N2.bit.INT1E     = 1;	//Enabled ADCINT1
   	AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;	//Disable ADCINT1 Continuous mode
   	AdcRegs.INTSEL1N2.bit.INT1SEL	= 2;	//setup EOC2 to trigger ADCINT1 to fire
   	AdcRegs.ADCSOC0CTL.bit.CHSEL 	= 4;	//set SOC0 channel select to ADCINA4
   	AdcRegs.ADCSOC1CTL.bit.CHSEL 	= 4;	//set SOC1 channel select to ADCINA4
   	AdcRegs.ADCSOC2CTL.bit.CHSEL 	= 2;	//set SOC1 channel select to ADCINA2
   	AdcRegs.ADCSOC0CTL.bit.TRIGSEL 	= 5;	//set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
   	AdcRegs.ADCSOC1CTL.bit.TRIGSEL 	= 5;	//set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
   	AdcRegs.ADCSOC2CTL.bit.TRIGSEL 	= 5;	//set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
   	AdcRegs.ADCSOC0CTL.bit.ACQPS 	= 6;	//set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
   	AdcRegs.ADCSOC1CTL.bit.ACQPS 	= 6;	//set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
   	AdcRegs.ADCSOC2CTL.bit.ACQPS 	= 6;	//set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
   	EDIS;

// Assumes ePWM1 clock is already enabled in InitSysCtrl();
      EPwm1Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
      EPwm1Regs.ETSEL.bit.SOCASEL	= 4;		// Select SOC from from CPMA on upcount
      EPwm1Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event
      EPwm1Regs.CMPA.half.CMPA 	= 0x0080;	// Set compare A value
      EPwm1Regs.TBPRD 				= 0xFFFF;	// Set period for ePWM1
      EPwm1Regs.TBCTL.bit.CTRMODE 	= 0;		// count up and start

   (void)xTaskCreate(BackgroundTask, "Background", 100, NULL, 1, NULL);
   (void)xTaskCreate(BlinkLEDTask, "LED", 100, NULL, 2, NULL);
   (void)xTaskCreate(CounterTask, "Counter", 100, NULL, 3, NULL);

	vTaskStartScheduler();
	// Step 6. IDLE loop. Just sit and loop forever (optional):
	for(;;);

}

void error(void)
{
    __asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}
/*! \bug freertos in the context stores only IER |= M_INT14;
 * Any other interrupt has to by allowed from inside of the OS
 */

/*! \todo interrupts walk around
 * all IER bits has to be set in the FreeRTOS configuration h file
 */

__interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;
   GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1;
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}

__interrupt void cpu_timer1_isr(void)
{
   CpuTimer1.InterruptCount++;
   GpioDataRegs.GPATOGGLE.bit.GPIO2 = 1;
   // The CPU acknowledges the interrupt.
   EDIS;
}

__interrupt void  adc_isr(void)
{

  Voltage1[ConversionCount] = AdcResult.ADCRESULT1;  //discard ADCRESULT0 as part of the workaround to the 1st sample errata for rev0
  Voltage2[ConversionCount] = AdcResult.ADCRESULT2;

  // If 20 conversions have been logged, start over
  if(ConversionCount == 9)
  {
     ConversionCount = 0;
  }
  else ConversionCount++;

  AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		//Clear ADCINT1 flag reinitialize for next SOC
  PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;   // Acknowledge interrupt to PIE

//  return; //MP why?
}

#if 1 == 1	//SCIA interrupts
__interrupt void sciaTxFifoIsr(void)
{
    uint16_t i;
    for(i=0; i< 2; i++)
    {
 	   SciaRegs.SCITXBUF=sdataA[i];     // Send data
	}

    for(i=0; i< 2; i++)                 //Increment send data for next cycle
    {
 	   sdataA[i] = (sdataA[i]+1) & 0x00FF;
	}

	SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;	// Clear SCI Interrupt flag
	PieCtrlRegs.PIEACK.all|=0x100;      // Issue PIE ACK
}

__interrupt void sciaRxFifoIsr(void)
{
    uint16_t i;
	for(i=0;i<2;i++)
	{
	   rdataA[i]=SciaRegs.SCIRXBUF.all;	 // Read data
	}
	for(i=0;i<2;i++)                     // Check received data
	{
	   if(rdataA[i] != ( (rdata_pointA+i) & 0x00FF) ) error();
	}
	rdata_pointA = (rdata_pointA+1) & 0x00FF;

	SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
	SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

	PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}
#endif	//SCIA interrupts

void scia_fifo_init()
{
   SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                  // No parity,8 char bits,
                                  // async mode, idle-line protocol
   SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                  // Disable RX ERR, SLEEP, TXWAKE
   SciaRegs.SCICTL2.bit.TXINTENA =1;
   SciaRegs.SCICTL2.bit.RXBKINTENA =1;
//   SciaRegs.SCIHBAUD = 0x0000;
//   SciaRegs.SCILBAUD = SCI_PRD;
#if (CPU_FRQ_60MHZ)
    SciaRegs.SCIHBAUD    =0x0000;  // 9600 baud @LSPCLK = 15MHz (60 MHz SYSCLK).
    SciaRegs.SCILBAUD    =0x00C2;
#elif (CPU_FRQ_50MHZ)
    SciaRegs.SCIHBAUD     =0x0000;  // 9600 baud @LSPCLK = 12.5 MHz (50 MHz SYSCLK)
    SciaRegs.SCILBAUD    =0x00A1;
#elif (CPU_FRQ_40MHZ)
    SciaRegs.SCIHBAUD    =0x0000;  // 9600 baud @LSPCLK = 10MHz (40 MHz SYSCLK).
    SciaRegs.SCILBAUD    =0x0081;
#endif
    SciaRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset

    SciaRegs.SCIFFTX.all=0xE040;	// Initalize the SCI FIFO
    SciaRegs.SCIFFRX.all=0x2044;
    SciaRegs.SCIFFCT.all=0x0;

#if 1 == 1 //test
   SciaRegs.SCICCR.bit.LOOPBKENA =1; // Enable loop back
   SciaRegs.SCIFFTX.all=0xC022;
   SciaRegs.SCIFFRX.all=0x0022;
   SciaRegs.SCIFFCT.all=0x00;
   SciaRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
   SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
   SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
#endif
}

#if 1 == 2
__interrupt void cpu_timer2_isr(void)
{
   EALLOW;
   DINT;
   CpuTimer2.InterruptCount++;
   // The CPU acknowledges the interrupt.
   Sch_TaskScheduler();
   EDIS;
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

}

#endif


/* Blinks LED and "increments" enumerator for viewing */
void BlinkLEDTask(void *pvParameters){
	portTickType xLastWakeTime;
	vTaskDelay( 100 );						/*FreeRTOS: delay from start 100 ticks*/
	// this is needed to prepare for vTaskDelayUntil() call
	xLastWakeTime = xTaskGetTickCount();

//	IER |= M_INT1;	//	CPU-Timer 0 PIE: Group 1 interrupt 7
//	IER |= M_INT13;	//	CPU-Timer 1

	for(;;){
		GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
//		GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1;
//		GpioDataRegs.GPATOGGLE.bit.GPIO2 = 1;
		GpioDataRegs.GPATOGGLE.bit.GPIO3 = 1;
		TaskCounter++;

		vTaskDelayUntil(&xLastWakeTime, 100); /*FreeRTOS: period 100 ticks*/
	}
}

/* Simple counter for demo purposes */
void CounterTask(void *pvParameters){
	portTickType xLastWakeTime;
	// this is needed to prepare for vTaskDelayUntil() call
	xLastWakeTime = xTaskGetTickCount();

	for(;;){
		ACounter++;
		vTaskDelayUntil(&xLastWakeTime, 10);
	}
}

/* Lowest priority background task. */
void BackgroundTask(void *pvParameters){
	for(;;){
		LongVariable++;
		taskYIELD(); // only needed if other task at same priority
	}
}

//===========================================================================
// No more.
//===========================================================================
