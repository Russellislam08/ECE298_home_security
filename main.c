#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include <string.h>
#include <stdio.h>

/*
 * This project contains some code samples that may be useful.
 *
 */

char ADCState = 0; //Busy state of the ADC
int adc_results[3] = {0};
int16_t ADCResult = 0; //Storage for the ADC conversion result

char arm_time[4];
int arm_time_counter;
int arming;
int entered;
int disarm_mode = 0;

int tripped[3] = {0}; // For buzzer
int zone_status[4][3]; // Status, start time, end time

// For RTC
volatile unsigned int HRS;
volatile unsigned int MINS;
volatile unsigned int SEC;

// Function declaration
void zone_reed_switch();
void print_status();
void sound_alarms();

void sound_alarms()
{
  int j = 0;
  int triggered = 0;
  for (j = 0; j < 4; j++)
  {
    if (tripped[j] == 1)
    {
      triggered = 1;
    }
  }
  if (triggered == 1)
  {
    Timer_A_outputPWM(TIMER_A0_BASE, &param); //Turn on PWM
  }
  else
  {
    Timer_A_stop(TIMER_A0_BASE); //Shut off PWM signal
  }
}

// Print status of each zone
void print_status()
{
  EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\r\n");

  char buffer[256];
  memset(buffer, 0, 256);
  sprintf(buffer, "ZONE 1 : ARM TIME SET FOR %d | DISARM TIME SET FOR %d. \r\n", zone_status[0][1], zone_status[0][2]);
  EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, buffer);

  memset(buffer, 0, 256);
  sprintf(buffer, "ZONE 2 : ARM TIME SET FOR %d | DISARM TIME SET FOR %d. \r\n", zone_status[1][1], zone_status[1][2]);
  EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, buffer);

  memset(buffer, 0, 256);
  sprintf(buffer, "ZONE 3 : ARM TIME SET FOR %d | DISARM TIME SET FOR %d. \r\n", zone_status[2][1], zone_status[2][2]);
  EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, buffer);

  memset(buffer, 0, 256);
  sprintf(buffer, "ZONE 4 : ARM TIME SET FOR %d | DISARM TIME SET FOR %d. \r\n", zone_status[3][1], zone_status[3][2]);
  EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, buffer);
  EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "===============================================================\r\n");

}

/*
 * PORT2 Interrupt Service Routine
 * Reset all the alarm NOT okay statuses and turn off alarm sounds
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
  // Clear Interrupt
  P1IFG &= ~BIT2; // P1.2 IFG cleared

  // Turn all NOT OKAY LEDs off
  GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Set ZONE0 Status to NOT OKAY
  GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3); // Set ZONE1 Status to NOT OKAY
  GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Set ZONE2 Status to NOT OKAY
  GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0); // Set ZONE3 Status to NOT OKAY

  // Reset all sound alarm checks to 0
  int j = 0;
  for (j = 0; j < 4; j++)
  {
    tripped[j] = 0;
  }

  P4OUT ^= BIT0; // Turn LED1 On
}

//// RTC interrupt service routine
#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR(void)
{
  if (RTCIV & RTCIV_RTCIF)
  { // RTC Overflow
    P1OUT ^= BIT0;
    SEC++;
    if (SEC == 60)
    {
      MINS++;
      SEC = 0;
    }
    if (MINS == 60)
    {
      HRS++;
      MINS = 0;
    }
    if (HRS == 24)
    {
      HRS = 0;
    }
  }

  showChar(SEC % 10 + '0', pos5);
  showChar((SEC / 10) % 10 + '0', pos4);
  showChar(MINS % 10 + '0', pos3);
  showChar((MINS / 10) % 6 + '0', pos2);
}

void main(void)
{
  char buttonState = 0; //Current button press state (to allow edge detection)

  /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */

  //Turn off interrupts during initialization
  __disable_interrupt();

  //Stop watchdog timer unless you plan on using it
  WDT_A_hold(WDT_A_BASE);

  // Initializations - see functions for more detail
  Init_GPIO();  //Sets all pins to output low as a default
  Init_PWM();   //Sets up a PWM output
  Init_ADC();   //Sets up the ADC to sample
  Init_Clock(); //Sets up the necessary system clocks
  Init_UART();  //Sets up an echo over a COM port
  Init_LCD();   //Sets up the LaunchPad LCD display

  // Some more initializations
  arm_time_counter = 0;
  arming = 0;
  entered = 0;

  int k = 0;
  for (k = 0; k < 4; k++)
  {
    zone_status[k][0] = 0;
    zone_status[k][1] = -1;
    zone_status[k][2] = -1;
  }

  //  Cycle = HIGH_COUNT / TIMER_A_PERIOD)

  /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
  PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

  //All done initializations - turn interrupts back on.
  __enable_interrupt();

  P1DIR |= 0x01;

  // RTC stuff
  do
  {
    CSCTL7 &= ~(XT1OFFG | DCOFFG); // Clear XT1 and DCO fault flag
    SFRIFG1 &= ~OFIFG;
  } while (SFRIFG1 & OFIFG); // Test oscillator fault flag

  RTCCTL = RTCSS__XT1CLK | RTCSR | RTCPS__1024 | RTCIE;

  RTCMOD = 32 - 1;

  P1IE |= BIT2;           // P1.2 interrupt enabled
  P1IFG &= ~BIT2;         // P1.2 IFG cleared
  __bis_SR_register(GIE); // Enter LPM3, enable interrupt
  while (1)
  {

    // Zone 1
    if (((MINS % 10) >= zone_status[0][1]) && ((MINS % 10) < zone_status[0][2]))
    {
      if (zone_status[0][0] == 2)
      {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);

        if (ADCState == 0)
        {

          ADCState = 1; //Set flag to indicate ADC is busy - ADC ISR (interrupt) will clear it
          ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);

          // Sliding average
          adc_results[2] = adc_results[1];
          adc_results[1] = adc_results[0];
          adc_results[0] = (int)ADCResult;

          if (((adc_results[0] + adc_results[1] + adc_results[2]) / 3) > 950) // Microphone
          {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
            tripped[0] = 1;
          }
        }

        if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1) == 1) // Reed switch
        {
          tripped[0] = 1;
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
        }
      }
    }
    else
    {
      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
      tripped[0] = 0;
    }

    // Zone 2
    if (((MINS % 10) >= zone_status[1][1]) && ((MINS % 10) < zone_status[1][2]))
    {
      if (zone_status[1][0] == 2)
      {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 1) // Reed switch
        {
          tripped[1] = 1;
          GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
          // Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
        }
      }
    }
    else
    {
      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
      GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
      tripped[1] = 0;
    }

    // Zone 3
    if (((MINS % 10) >= zone_status[2][1]) && ((MINS % 10) < zone_status[2][2]))
    {
      if (zone_status[2][0] == 2)
      {
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
        if (GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 1)
        {
          tripped[2] = 1;
          GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);
          // Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
        }
      }
    }
    else
    {
      GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);
      GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
      tripped[2] = 0;
    }

    // Zone 4
    if (((MINS % 10) >= zone_status[3][1]) && ((MINS % 10) < zone_status[3][2]))
    {
      if (zone_status[3][0] == 2)
      {
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
        if (GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) == 1)
        {
          tripped[3] = 1;
          GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
          // Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
        }
      }
    }
    else
    {
      GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
      GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
      tripped[3] = 0;
    }

    // Sound the alarms, if required
    sound_alarms();

  }
}

void Init_GPIO(void)
{
  // Set all GPIO pins to output low to prevent floating input and reduce power consumption
  GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);

  GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);

  //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
  GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
  GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

  //Set LED1 and LED2 as outputs
  //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
  GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);

  GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN3);
  GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN2);
  GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN5);
  GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN1);

  GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3);
  GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
  GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3);
  GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);

  GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);
  GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);
  GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);
  GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0);

  GPIO_setAsInputPin(ADC_IN_PORT, ADC_IN_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
  /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

  /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
  //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

  // Set external clock frequency to 32.768 KHz
  CS_setExternalClockSource(32768);
  // Set ACLK = XT1
  CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
  // Initializes the XT1 crystal oscillator
  CS_turnOnXT1LF(CS_XT1_DRIVE_1);
  // Set SMCLK = DCO with frequency divider of 1
  CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
  // Set MCLK = DCO with frequency divider of 1
  CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
  /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

  //Configure UART pins, which maps them to a COM port over the USB cable
  //Set P1.0 and P1.1 as Secondary Module Function Input.
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

  /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

  //SMCLK = 1MHz, Baudrate = 9600
  //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
  EUSCI_A_UART_initParam param = {0};
  param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
  param.clockPrescalar = 6;
  param.firstModReg = 8;
  param.secondModReg = 17;
  param.parity = EUSCI_A_UART_NO_PARITY;
  param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
  param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
  param.uartMode = EUSCI_A_UART_MODE;
  param.overSampling = 1;

  if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
  {
    return;
  }

  EUSCI_A_UART_enable(EUSCI_A0_BASE);

  EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

  // Enable EUSCI_A0 RX interrupt
  EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector = USCI_A0_VECTOR
__interrupt void EUSCIA0_ISR(void)
{
  uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

  EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

  RTCCTL = RTCSS__XT1CLK | RTCSR | RTCPS__1024 | RTCIE;

  RTCMOD = 32 - 1;

  __bis_SR_register(GIE); // Enter LPM3, enable interrupt

  uint8_t key_pressed;
  memset(&key_pressed, 0, sizeof(uint8_t));
  key_pressed = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);

  if (RxStatus)
  {

    entered = 0;

    if (key_pressed == 'S' || key_pressed == 's') // List status of all zones
    {
      print_status();
      entered = 1;
    }

    if (key_pressed == 'D' || key_pressed == 'd') // List status of all zones
    {
      disarm_mode = 1;
      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "PLEASE ENTER ZONE TO DISARM (1,2,3,4)\n\r");
      entered = 1;
    }

    if (disarm_mode == 1)
    {
      if (key_pressed == '1')
      {
        if (zone_status[0][0] == 0)
        {
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (1) IS NOT ARMED. ABORTING...\n\r");
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
          disarm_mode = 0;
          return;
        }
        zone_status[0][0] = 0;
        zone_status[0][1] = -1;
        zone_status[0][2] = -1;
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (1) HAS BEEN DISARMED\n\r");
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
        disarm_mode = 0;
        return;
      }
      else if (key_pressed == '2')
      {
        if (zone_status[1][0] == 0)
        {
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (2) IS NOT ARMED. ABORTING...\n\r");
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
          disarm_mode = 0;
          return;
        }
        zone_status[1][0] = 0;
        zone_status[1][1] = -1;
        zone_status[1][2] = -1;
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (2) HAS BEEN DISARMED\n\r");
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
        disarm_mode = 0;
        return;
      }
      else if (key_pressed == '3')
      {
        if (zone_status[2][0] == 0)
        {
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (3) IS NOT ARMED. ABORTING...\n\r");
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
          disarm_mode = 0;
          return;
        }
        zone_status[2][0] = 0;
        zone_status[2][1] = -1;
        zone_status[2][2] = -1;
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (3) HAS BEEN DISARMED\n\r");
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
        disarm_mode = 0;
        return;
      }
      else if (key_pressed == '4')
      {
        if (zone_status[3][0] == 0)
        {
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (4) IS NOT ARMED. ABORTING...\n\r");
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
          disarm_mode = 0;
          return;
        }
        zone_status[3][0] = 0;
        zone_status[3][1] = -1;
        zone_status[3][2] = -1;
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (4) HAS BEEN DISARMED\n\r");
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
        disarm_mode = 0;
        return;
      }
    }

    if (key_pressed == 'A' || key_pressed == 'a') // I am going to arm a zone
    {
      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "Select Zone (1), (2), (3) or (4)\n\r");
      arming = 1;
    }
    // Checking to see which zone you want to arm
    if (arming)
    {
      if (key_pressed == '1')
      {
        if (zone_status[0][0] != 0)
        {
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (1) IS ALREADY ARMED. PLEASE DISARM TO RE-ARM. ABORTING...\n\r");
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
          return;
        }
        zone_status[0][0] = 1;
        arming = 0;
        entered = 1;
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "FOR WHAT TIME DO YOU WANT TO START ALARM FOR ZONE (1)? ENTER (0-9)\n\r");
      }
      else if (key_pressed == '2')
      {
        if (zone_status[1][0] != 0)
        {
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (2) IS ALREADY ARMED. PLEASE DISARM TO RE-ARM. ABORTING...\n\r");
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
          return;
        }
        zone_status[1][0] = 1;
        arming = 0;
        entered = 1;
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "FOR WHAT TIME DO YOU WANT TO START ALARM FOR ZONE (2)? ENTER (0-9)\n\r");
      }
      else if (key_pressed == '3')
      {
        if (zone_status[2][0] != 0)
        {
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (3) IS ALREADY ARMED. PLEASE DISARM TO RE-ARM. ABORTING...\n\r");
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
          return;
        }
        zone_status[2][0] = 1;
        arming = 0;
        entered = 1;
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "FOR WHAT TIME DO YOU WANT TO START ALARM FOR ZONE (3)? ENTER (0-9)\n\r");
      }
      else if (key_pressed == '4')
      {
        if (zone_status[3][0] != 0)
        {
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "ZONE (4) IS ALREADY ARMED. PLEASE DISARM TO RE-ARM. ABORTING...\n\r");
          EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
          return;
        }
        zone_status[3][0] = 1;
        arming = 0;
        entered = 1;
        EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "FOR WHAT TIME DO YOU WANT TO START ALARM FOR ZONE (4)? ENTER (0-9)\n\r");
      }
    }

    // Start alarms
    if ((zone_status[0][0] == 1) && (zone_status[0][1] == -1) && (entered == 0))
    {
      zone_status[0][1] = (int)key_pressed - 48;
      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "FOR WHAT TIME DO YOU WANT TO END ALARM FOR ZONE (1)? ENTER (1-9)\n\r");
      entered = 1;
    }

    if ((zone_status[1][0] == 1) && (zone_status[1][1] == -1) && (entered == 0))
    {
      zone_status[1][1] = (int)key_pressed - 48;
      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "FOR WHAT TIME DO YOU WANT TO END ALARM FOR ZONE (2)? ENTER (1-9)\n\r");
      entered = 1;
    }

    if ((zone_status[2][0] == 1) && (zone_status[2][1] == -1) && (entered == 0))
    {
      zone_status[2][1] = (int)key_pressed - 48;
      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "FOR WHAT TIME DO YOU WANT TO END ALARM FOR ZONE (3)? ENTER (1-9)\n\r");
      entered = 1;
    }

    if ((zone_status[3][0] == 1) && (zone_status[3][1] == -1) && (entered == 0))
    {
      zone_status[3][1] = (int)key_pressed - 48;
      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "FOR WHAT TIME DO YOU WANT TO END ALARM FOR ZONE (4)? ENTER (1-9)\n\r");
      entered = 1;
    }

    // End alarms
    if ((zone_status[0][0] == 1) && (zone_status[0][1] != -1) && (zone_status[0][2] == -1) && (entered == 0))
    {
      zone_status[0][2] = (int)EUSCI_A_UART_receiveData(EUSCI_A0_BASE) - 48;
      zone_status[0][0] = 2;

      char temp[256];
      memset(temp, 0, 256);
      sprintf(temp, "ARMED ZONE (1) AT START TIME OF (%d) AND END TIME OF (%d)\n\r", zone_status[0][1], zone_status[0][2]);

      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, temp);
      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
    }

    if ((zone_status[1][0] == 1) && (zone_status[1][1] != -1) && (zone_status[1][2] == -1) && (entered == 0))
    {
      zone_status[1][2] = (int)EUSCI_A_UART_receiveData(EUSCI_A0_BASE) - 48;
      zone_status[1][0] = 2;

      char temp[256];
      memset(temp, 0, 256);
      sprintf(temp, "ARMED ZONE (2) AT START TIME OF (%d) AND END TIME OF (%d)\n\r", zone_status[1][1], zone_status[1][2]);

      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, temp);
      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
    }

    if ((zone_status[2][0] == 1) && (zone_status[2][1] != -1) && (zone_status[2][2] == -1) && (entered == 0))
    {
      zone_status[2][2] = (int)EUSCI_A_UART_receiveData(EUSCI_A0_BASE) - 48;
      zone_status[2][0] = 2;

      char temp[256];
      memset(temp, 0, 256);
      sprintf(temp, "ARMED ZONE (3) AT START TIME OF (%d) AND END TIME OF (%d)\n\r", zone_status[2][1], zone_status[2][2]);

      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, temp);
      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
    }

    if ((zone_status[3][0] == 1) && (zone_status[3][1] != -1) && (zone_status[3][2] == -1) && (entered == 0))
    {
      zone_status[3][2] = (int)EUSCI_A_UART_receiveData(EUSCI_A0_BASE) - 48;
      zone_status[3][0] = 2;

      char temp[256];
      memset(temp, 0, 256);
      sprintf(temp, "ARMED ZONE (4) AT START TIME OF (%d) AND END TIME OF (%d)\n\r", zone_status[3][1], zone_status[3][2]);

      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, temp);
      EUSCI_A_UART_transmitMultipleData(EUSCI_A0_BASE, "================================================================\n\r");
    }
  }
}

/* PWM Initialization */
void Init_PWM(void)
{
  /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
  //Generate PWM - Timer runs in Up-Down mode
  param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
  param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
  param.timerPeriod = TIMER_A_PERIOD; //Defined in main.h
  param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
  param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
  param.dutyCycle = HIGH_COUNT; //Defined in main.h

  //PWM_PORT PWM_PIN (defined in main.h) as PWM output
  GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
  /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

  //Set ADC_IN to input direction
  GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

  //Initialize the ADC Module
  /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
  ADC_init(ADC_BASE,
           ADC_SAMPLEHOLDSOURCE_SC,
           ADC_CLOCKSOURCE_ADCOSC,
           ADC_CLOCKDIVIDER_1);

  ADC_enable(ADC_BASE);

  /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
  ADC_setupSamplingTimer(ADC_BASE,
                         ADC_CYCLEHOLD_16_CYCLES,
                         ADC_MULTIPLESAMPLESDISABLE);

  //Configure Memory Buffer
  /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
  ADC_configureMemory(ADC_BASE,
                      ADC_IN_CHANNEL,
                      ADC_VREFPOS_AVCC,
                      ADC_VREFNEG_AVSS);

  ADC_clearInterrupt(ADC_BASE,
                     ADC_COMPLETED_INTERRUPT);

  //Enable Memory Buffer interrupt
  ADC_enableInterrupt(ADC_BASE,
                      ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR(void)
{
  uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

  ADC_clearInterrupt(ADC_BASE, ADCStatus);

  if (ADCStatus)
  {
    ADCState = 0; //Not busy anymore
    ADCResult = ADC_getResults(ADC_BASE);
  }
}
