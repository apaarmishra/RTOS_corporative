// RTOS Framework - Fall 2016
// J Losh

// Student Name: Apaar Mishra

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 4 Pushbuttons and 4 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for green and yellow LEDs (temporary to guarantee compilation)
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32)))
#define GREEN_LED     (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))
#define YELLOW_LED    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))
#define PB0			 (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define PB1			 (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define PB2			 (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define PB3			 (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))
//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------
//Preprocessors for task
#define MAX_QUEUE_SIZE 10
#define STATE_INVALID    0 // no task
#define STATE_READY      1 // ready to run
#define STATE_BLOCKED    2 // has run, but now blocked by semaphore
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define MAX_TASKS 9      // maximum number of valid tasks

// function pointer
typedef void (*_fn)();

// semaphore
struct semaphore {
	unsigned int count;
	unsigned int queueSize;
	unsigned int processQueue[MAX_QUEUE_SIZE]; // store task index here
}*s, keyPressed, keyReleased, flashReq;

/* Global Variables*/
uint8_t task = 0;   	// index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
int z;
int j = 0;
char input;
char check[30];
int charcount = 0;
char output[30];
char type1[30];
int fieldposition[20];
int command();
int skipcounter = 0;
int psvariable=0;
int diference[MAX_TASKS];
uint32_t ADCVals[3]={'\0'};
struct _tcb {
	uint8_t state;                 // see STATE_ values above
	void *pid;                     // used to uniquely identify thread
	void *sp;                      // location of stack pointer for thread
	uint8_t priority;              // 0=highest, 7=lowest
	uint8_t currentPriority;       // used for priority inheritance
	uint32_t ticks;                // ticks until sleep complete
	char name[15];                 // name of task used in ps command
	void *semaphore;     // pointer to the semaphore that is blocking the thread
	int semacount;
	int skipcount;
	int starttime;
	int endtime;
}tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];

//-----------------------------------------------------------------------------
// RTOS Kernel
//-----------------------------------------------------------------------------

void rtosInit() {
	uint8_t i;
	// no tasks running
	taskCount = 0;
	// clear out tcb records
	for (i = 0; i < MAX_TASKS; i++) {
		tcb[i].state = STATE_INVALID;
		tcb[i].pid = 0;
	}
	//systick for 1ms system timer
}

int rtosScheduler() {
	// prioritization to 16 levels
	bool ok;
	//int i=0;
	static uint8_t task = 0xFF;
	ok = false;
	tcb[task].endtime += psvariable;
	while (!ok) {
		task++;
		if (task >= MAX_TASKS)
			task = 0;
		if(tcb[task].skipcount == 0){
		ok = (tcb[task].state == STATE_READY);

		tcb[task].skipcount = tcb[task].currentPriority;
		}
		else
			tcb[task].skipcount--;
	}
	psvariable=0;
	tcb[task].starttime += psvariable;
	return task;
}

bool createThread(_fn fn, char name[15], int priority) {
	bool ok = false;
	uint8_t i = 0;
	uint8_t j = 0;
	bool found = false;
	// steps to ensure a task switch cannot occur
	// save starting address if room in task list
	if (taskCount <= MAX_TASKS) {
		// make sure fn not already in list (prevent reentrancy)
		while (!found && (i < MAX_TASKS)) {
			found = (tcb[i].pid == fn);
			if (found == 1 && tcb[i].state == 0) {
				tcb[i].state = STATE_READY;
			}
			i++;
		}
		if (!found) {
			// find first available tcb record
			i = 0;
			while (tcb[i].state != STATE_INVALID) {
				i++;
			}
			tcb[i].state = STATE_READY;
			tcb[i].pid = fn;
			// REQUIRED: preload stack to look like the task had run before
			stack[i][255] = 1; //XPSR
			stack[i][254] = (int) fn; //LR
			stack[i][253] = (int) fn; //LR
			stack[i][252] = 12; //R12
			stack[i][251] = 3; //R3
			stack[i][250] = 2; //R2
			stack[i][249] = 1; //R1
			stack[i][248] = 0; //R0
			stack[i][247] = 11; //R11
			stack[i][246] = 10; //R10
			stack[i][245] = 9; //R9
			stack[i][244] = 8; //R8
			stack[i][243] = 7; //R7
			stack[i][242] = 6; //R6
			stack[i][241] = 5; //R5
			stack[i][240] = 4; //R4
            
			tcb[i].sp = &stack[i][240]; // REQUIRED: + offset as needed for the pre-loaded stack
			tcb[i].priority = priority;
			tcb[i].skipcount = priority;
			tcb[i].currentPriority = priority;
			tcb[i].semacount=0;
			for (j = 0; j <= strlen(name); j++) {
				tcb[i].name[j] = name[j];
			}
			//  tcb[i].name = &name ;
			// increment task count
			taskCount++;
			ok = true;
		}
	}
	return ok;
}

// function to destroy a process
void destroyProcess(_fn fn) {
	static int count2 = 0;
	int num1;

	for (j = 0; j <= MAX_TASKS; j++) {
		if (tcb[j].pid == fn)
			break;
	}
	tcb[j].state = STATE_INVALID;
	if (tcb[j].pid == tcb[1].pid) {
		num1 = flashReq.queueSize;
		if (num1 > 0) {
			while (num1 >= count2) {
				flashReq.processQueue[count2] =
						flashReq.processQueue[count2 + 1];
				num1--;
				count2++;
			}
			flashReq.queueSize--;
		}
	}
	if (tcb[j].pid == tcb[4].pid) {

		num1 = keyReleased.queueSize;
		if (num1 > 0) {
			while (num1 >= count2) {
				keyReleased.processQueue[count2] =
						keyReleased.processQueue[count2 + 1];
				num1--;
				count2++;
			}
			keyReleased.queueSize--;
		}
	}
	if (tcb[j].pid == tcb[5].pid) {

		num1 = keyPressed.queueSize;
		if (num1 > 0) {
			while (num1 >= count2) {
				keyPressed.processQueue[count2] = keyPressed.processQueue[count2
						+ 1];
				num1--;
				count2++;
			}
			keyPressed.queueSize--;
		}
	}

}

void rtosStart() {
	//  code to call the first task to be run, restoring the preloaded context
	_fn fn;
	TIMER1_CTL_R |= TIMER_CTL_TAEN;
	task = rtosScheduler();

	// Add code to initialize the SP with tcb[task_current].sp;
	void *psp1;

	psp1 = tcb[task].sp;

	__asm(" MOV R13,R0 ");
	__asm(" POP {R4} ");
	__asm(" POP {R5} ");
	__asm(" POP {R6} ");
	__asm(" POP {R7} ");
	__asm(" POP {R8} ");
	__asm(" POP {R9} ");
	__asm(" POP {R10} ");
	__asm(" POP {R11} ");
	__asm(" POP {R0} ");
	__asm(" POP {R1} ");
	__asm(" POP {R2} ");
	__asm(" POP {R3} ");
	__asm(" POP {R12} ");
	__asm(" POP {LR} ");
	__asm(" POP {LR} ");
	__asm(" POP {XPSR} ");
	__asm(" BX LR ");
	// Restore the stack to run the first process
}

void init(void* p, int count) {
	s = p;
	s->count = count;
	s->queueSize = 0;
}

void stackp(int point2) {
	__asm(" add SP, #8");
	__asm(" STR SP,[R0]");
}
void previousstack(int *point) {
	__asm(" MOV SP,R0");
}
// REQUIRED: modify this function to yield execution back to scheduler
void yield() {
	// push registers, call scheduler, pop registers, return to new function
	
	__asm(" POP {R3} ");
	__asm(" POP {LR} ");
	__asm(" PUSH {XPSR} ");
	__asm(" PUSH {LR} ");
	__asm(" PUSH {LR} ");
	__asm(" PUSH {R12} ");
	__asm(" PUSH {R3} ");
	__asm(" PUSH {R2} ");
	__asm(" PUSH {R1} ");
	__asm(" PUSH {R0} ");
	__asm(" PUSH {R11} ");
	__asm(" PUSH {R10} ");
	__asm(" PUSH {R9} ");
	__asm(" PUSH {R8} ");
	__asm(" PUSH {R7} ");
	__asm(" PUSH {R6} ");
	__asm(" PUSH {R5} ");
	__asm(" PUSH {R4} ");
    
    //Saving the stack pointer value
    stackp(&tcb[task].sp);

	///FInd the task2 to be scheduled from rtosScheduler
	task = rtosScheduler();
	//tcb[task].taskcount++;

	void *p2 = tcb[task].sp;
	previousstack(p2);

	__asm(" sub.w  sp, sp, #8");
	__asm(" POP {R4} ");
	__asm(" POP {R5} ");
	__asm(" POP {R6} ");
	__asm(" POP {R7} ");
	__asm(" POP {R8} ");
	__asm(" POP {R9} ");
	__asm(" POP {R10} ");
	__asm(" POP {R11} ");
	__asm(" POP {R0} ");
	__asm(" POP {R1} ");
	__asm(" POP {R2} ");
	__asm(" POP {R3} ");
	__asm(" POP {R12} ");
	__asm(" POP {LR} ");
	__asm(" POP {LR} ");
	__asm(" POP {XPSR} ");
	__asm(" BX LR ");

}

// function to support 1ms system timer
// execution yielded back to scheduler until time elapses
void sleep(uint32_t tick) {
	// push registers, set state to delayed, store timeout, call scheduler, pop registers,
	// return to new function (separate unrun or ready processing)
	
    tcb[task].ticks = tick;
	tcb[task].state = STATE_DELAYED;
	
    __asm(" POP {R1} ");
	__asm(" POP {R2} ");
	__asm(" POP {R3} ");
	__asm(" POP {LR} ");

	__asm(" PUSH {XPSR} ");
	__asm(" PUSH {LR} ");
	__asm(" PUSH {LR} ");
	__asm(" PUSH {R12} ");
	__asm(" PUSH {R3} ");
	__asm(" PUSH {R2} ");
	__asm(" PUSH {R1} ");
	__asm(" PUSH {R0} ");
	__asm(" PUSH {R11} ");
	__asm(" PUSH {R10} ");
	__asm(" PUSH {R9} ");
	__asm(" PUSH {R8} ");
	__asm(" PUSH {R7} ");
	__asm(" PUSH {R6} ");
	__asm(" PUSH {R5} ");
	__asm(" PUSH {R4} ");

	stackp(&tcb[task].sp);

	///FInd the task2 to be scheduled from rtosScheduler
	task = rtosScheduler();

    void *p2 = tcb[task].sp;
	previousstack(p2);

	__asm(" sub.w  sp, sp, #8");
	__asm(" POP {R4} ");
	__asm(" POP {R5} ");
	__asm(" POP {R6} ");
	__asm(" POP {R7} ");
	__asm(" POP {R8} ");
	__asm(" POP {R9} ");
	__asm(" POP {R10} ");
	__asm(" POP {R11} ");
	__asm(" POP {R0} ");
	__asm(" POP {R1} ");
	__asm(" POP {R2} ");
	__asm(" POP {R3} ");
	__asm(" POP {R12} ");
	__asm(" POP {LR} ");
	__asm(" POP {LR} ");
	__asm(" POP {XPSR} ");
	__asm(" BX LR ");

}

// function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler
void wait(void* pSemaphore) {
	s = pSemaphore;
	if ((s->count) > 0) {
		s->count--;
		tcb[task].semacount++;
	}

	else {
		__asm(" POP {R1} ");
		__asm(" POP {R2} ");
		__asm(" POP {R3} ");
		__asm(" POP {LR} ");

		__asm(" PUSH {XPSR} ");
		__asm(" PUSH {LR} ");
		__asm(" PUSH {LR} ");
		__asm(" PUSH {R12} ");
		__asm(" PUSH {R3} ");
		__asm(" PUSH {R2} ");
		__asm(" PUSH {R1} ");
		__asm(" PUSH {R0} ");
		__asm(" PUSH {R11} ");
		__asm(" PUSH {R10} ");
		__asm(" PUSH {R9} ");
		__asm(" PUSH {R8} ");
		__asm(" PUSH {R7} ");
		__asm(" PUSH {R6} ");
		__asm(" PUSH {R5} ");
		__asm(" PUSH {R4} ");

		stackp(&tcb[task].sp);
		tcb[task].state = STATE_BLOCKED;
		s->queueSize++;
		s->processQueue[j] = task;
		///FInd the task2 to be scheduled from rtosScheduler
		task = rtosScheduler();

		void *p2 = tcb[task].sp;
		previousstack(p2);
		__asm(" sub.w  sp, sp, #8");
		__asm(" POP {R4} ");
		__asm(" POP {R5} ");
		__asm(" POP {R6} ");
		__asm(" POP {R7} ");
		__asm(" POP {R8} ");
		__asm(" POP {R9} ");
		__asm(" POP {R10} ");
		__asm(" POP {R11} ");
		__asm(" POP {R0} ");
		__asm(" POP {R1} ");
		__asm(" POP {R2} ");
		__asm(" POP {R3} ");
		__asm(" POP {R12} ");
		__asm(" POP {LR} ");
		__asm(" POP {LR} ");
		__asm(" POP {XPSR} ");
		__asm(" BX LR ");

	}
}

// function to signal a semaphore is available
void post(void* pSemaphore) {
	s = pSemaphore;
	s->count++;
	int n = s->queueSize;
	int cnt = 0;
	if (s->queueSize > 0) {

		tcb[s->processQueue[0]].state = STATE_READY;
		s->queueSize--;
		if (s->queueSize > 0) {
			while (n >= cnt) {
				s->processQueue[cnt] = s->processQueue[cnt + 1];
				n--;
				cnt++;
			}
		}

	}
}

// function to add support for the system timer
void systickIsr() {
	int j;
	for (j = 0; j <= MAX_TASKS; j++) {
		if (tcb[j].state == STATE_DELAYED) {
			tcb[j].ticks--;
			if (tcb[j].ticks == 0) {
				tcb[j].state = STATE_READY;
			}
		}
	}
}

// function to add support for the service call
void svcCallIsr() {
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void systickHw() {
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE
			| NVIC_ST_CTRL_CLK_SRC;
	NVIC_ST_RELOAD_R = 40000 - 1;
	NVIC_ST_CURRENT_R = 0;
}
// Initialize Hardware
void initHw() {
	// REQUIRED: Add initialization for orange, red, green, and yellow LEDs
	//           4 pushbuttons, and uart
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
			| SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

	// Set GPIO ports to use APB (not needed since default configuration -- for clarity)
	// Note UART on port A must use APB
	SYSCTL_GPIOHBCTL_R = 0;

	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF
			| SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD| SYSCTL_RCGC2_GPIOE;

	GPIO_PORTC_DIR_R = 0x00;  // bits 1 and 3 are outputs, other pins are inputs
	//GPIO_PORTC_DR2R_R = 0xF0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTC_DEN_R = 0xF0;  // enable LEDs and pushbuttons
	GPIO_PORTC_PUR_R = 0xF0;  // enable internal pull-up for push button

	GPIO_PORTD_DIR_R = 0x0F;
	GPIO_PORTD_DR2R_R = 0x0F;
	GPIO_PORTD_DEN_R = 0x0F;

	// Configure UART0 pins
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
	GPIO_PORTA_DEN_R |= 3;                         // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                       // default, added for clarity
	GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
	UART0_CTL_R = 0;                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
	UART0_IBRD_R = 21;   // r = 40 MHz / (Nx38.4kHz), set floor(r)=65, where N=1
	UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
	UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
	UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

	// Configure Timer 1 as the time base
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
	TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
	TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
	TIMER1_TAILR_R = 0x61A80; // set load value to 40e6 for 1 Hz interrupt rate
	TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
	NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);     // turn-on interrupt 37 (TIMER1A)
	TIMER1_CTL_R |= TIMER_CTL_TAEN;


	// Configure AN0 as an analog input
	SYSCTL_RCGCADC_R |= 1;                      // turn on ADC module 0 clocking
	GPIO_PORTE_AFSEL_R |= 0x0E; // select alternative functions for AIN0,1,2 (PE3,2,1)
	GPIO_PORTE_DEN_R &= ~0x0E;      // turn off digital operation on pin PE3,2,1
	GPIO_PORTE_AMSEL_R |= 0x0E;
	ADC0_CC_R = ADC_CC_CS_SYSPLL; // select PLL as the time base (not needed, since default value)
	ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1; // disable sample sequencer 1 (SS1) for programming
	ADC0_EMUX_R = ADC_EMUX_EM1_PROCESSOR; // select SS1 bit in ADCPSSI as trigger
	ADC0_SSMUX1_R |= 0x210;           // set first sample to AIN0, AIN1 and AIN2
	ADC0_SSCTL1_R = ADC_SSCTL1_END2;             // mark third sample as the end
	ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;                 // enable SS1 for operation
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us) {
	// Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");
	// 1
	__asm("WMS_LOOP1:   SUB  R1, #1");
	// 6
	__asm("             CBZ  R1, WMS_DONE1");
	// 5+1*3
	__asm("             NOP");
	// 5
	__asm("             B    WMS_LOOP1");
	// 5*3
	__asm("WMS_DONE1:   SUB  R0, #1");
	// 1
	__asm("             CBZ  R0, WMS_DONE0");
	// 1
	__asm("             B    WMS_LOOP0");
	// 1*3
	__asm("WMS_DONE0:");
	// ---
	// 40 clocks/us + error
}
void readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS1;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY){
    	yield();
    }// wait until SS3 is not busy
    int i=0;
    while(!(ADC0_SSFSTAT1_R & ADC_SSFSTAT1_EMPTY))
    {
        ADCVals[i]= ADC0_SSFIFO1_R;                    // get single result from the FIFO
        i++;
    }
}
timerISR(){
	psvariable++;
	TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}
// REQUIRED: add code to return a value from 0-15 indicating which of 4 PBs are pressed
uint8_t readPbs() {
	if (PB0 == 0)
		return 1;
	else if (PB1 == 0)
		return 2;
	else if (PB2 == 0)
		return 4;
	else if (PB3 == 0)
		return 8;
	else
		return 0;

}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle() {
	while (true) {
		ORANGE_LED = 1;
		waitMicrosecond(1000);
		ORANGE_LED = 0;
		yield();
	}
}

void flash4Hz() {
	while (true) {
		GREEN_LED ^= 1;
		sleep(125);
	}
}

void oneshot() {
	while (true) {
		wait(&flashReq);
		YELLOW_LED = 1;
		sleep(1000);
		YELLOW_LED = 0;
	}
}

void partOfLengthyFn() {
	// represent some lengthy operation
	waitMicrosecond(1000);
	// give another process a chance
	yield();
}

void lengthyFn() {
	uint16_t i;
	while (true) {
		for (i = 0; i < 4000; i++) {
			partOfLengthyFn();
		}
		RED_LED ^= 1;
	}
}

void readKeys() {
	uint8_t buttons;
	while (true) {
		wait(&keyReleased);
		buttons = 0;
		while (buttons == 0) {
			buttons = readPbs();
			yield();
		}
		post(&keyPressed);
		if ((buttons & 1) != 0) {
			YELLOW_LED ^= 1;
			RED_LED = 1;
		}
		if ((buttons & 2) != 0) {
			post(&flashReq);
			RED_LED = 0;
		}
		if ((buttons & 4) != 0) {
			createThread(flash4Hz, "Flash_4hz", 0);
		}
		if ((buttons & 8) != 0) {
			destroyProcess(flash4Hz);
		}

		yield();
	}
}

void debounce() {
	uint8_t count;
	while (true) {
		wait(&keyPressed);
		count = 10;
		while (count != 0) {
			sleep(10);
			if (readPbs() == 0)
				count--;
			else
				count = 10;
		}
		post(&keyReleased);
	}
}

void uncooperative() {
	while (true) {
		while (readPbs() == 8) {

		}
		yield();
	}
}

void putcUart0(char c) {
	while (UART0_FR_R & UART_FR_TXFF)
		;
	UART0_DR_R = c;
}
void putsUart0(char* str) {
	int i;
	for (i = 0; i < strlen(str); i++) {
		putcUart0(str[i]);
	}
}

char getcUart0() {
	while (UART0_FR_R & UART_FR_RXFE) {
		yield();
	}
	return UART0_DR_R & 0xFF;
}
bool iscommand(char fieldname[10]) {
	bool answer;
	answer = (strcmp(fieldname, &check[fieldposition[0]]) == 0);
	return answer;
}

int getnumber(int index) {
	int num;
	strcpy(num, &check[fieldposition[index]]);
	int num2;
	num2 = atoi(num);
	return num2;

}
void shell() {
	// REQUIRED: add processing for the shell commands here
			int i=0;
			while(1)
			{
				i= 0;charcount=0;int cnt1=0;j=0;int no_of_fields=0;int index=0;
				putsUart0("\r\n\Plss enter valid shell commands\r\n");
			while(i<32)
			{
				input = getcUart0();
				if((input>96 && input<123) | (input>47 && input<58)|(input==32)|(input>32 && input<48)|(input>57 && input<65)|(input>90 && input<97) )
				{
					i++;
					check[i-1]=input;
					charcount++;
					output[i-1]=input-32;

				}

			if((input==8) & (i>0))
				i--;

			if((input>96 && input<123))
				type1[i-1]='a';//type1=1 implies it is an alphabet

			if((input>47 && input<58))
				type1[i-1]='n'; //type1=0 implies it is a number


				if((input>32 && input<48)|(input>57 && input<65)|(input>90 && input<97)|(input==32))
			{
				type1[i-1]='s';//type1=2 implies it is a special charachter
				check[i-1]=0;
			}

			if (input==13)
			{
				break;
			}

			}
			while(cnt1<32)
			{

			if((type1[cnt1] == 's' && type1[cnt1 + 1] == 'a') || (type1[cnt1] == 's' && type1[cnt1 + 1] == 'n') )
			{
				fieldposition[j]=cnt1+1;
				j++;
				no_of_fields++;
			}
			if(type1[cnt1]=='a'&& cnt1==0)
			{
				fieldposition[j]=cnt1;
				no_of_fields++;
				j++;
			}
			cnt1++;
			}
			command();
			yield();
	}

}
int command() {

	char string1[10];
	int name = 0;
	if (iscommand("ps")) {
		putsUart0("Process Status Entered\r\n");
		int total = 0;
		int i = 0;
		int tasktime = 0;
		for (i = 0; i <= MAX_TASKS; i++) {
			diference[i] = tcb[i].endtime - tcb[i].starttime  ;
		}
		for (i = 0; i <= MAX_TASKS; i++) {
					total = total + diference[i] ;
				}
		for (i = 0; i <= MAX_TASKS; i++) {
			tasktime = (diference[i] * 100) / total;
			putsUart0(tcb[i].name);
			putsUart0("\t");
			sprintf(string1, "%u", tasktime);
			putsUart0(string1);
			putsUart0("\n\r");
		}


	}


	if (iscommand("kill")) {
		int num = getnumber(1);
		if (num == tcb[1].pid) {
			putsUart0("Killing flash4Hz\r\n");
			destroyProcess(flash4Hz);
		}

	}
	if (iscommand("reboot")) {
		putsUart0("Rebooting the system \r\n");
		ResetISR();
	}
	if (iscommand("flash4hz")) {
		putsUart0("Creating flash4Hz\r\n");
		createThread(flash4Hz, "Flash_4hz", 0);
	}
	if (iscommand("pidoflash4hz")) {
				putsUart0("Process ID of flash4Hz is : \r\n");
				name = tcb[1].pid;
				sprintf(string1, "%u", name);
				putsUart0(string1);
			} else if (iscommand("pidofidle")) {
				putsUart0("Process ID of Idle is : \r\n");
				name = tcb[0].pid;
				sprintf(string1, "%u", name);
				putsUart0(string1);
			} else if (iscommand("pidoflengthyfn")) {
				putsUart0("Process ID of Lengthyfn is : \r\n");
				name = tcb[2].pid;
				sprintf(string1, "%u", name);
				putsUart0(string1);
			}

	if (iscommand("ipcs")) {
		int i=0;
		putsUart0("Displaying Semaphore Usage\r\n");
		for(i=0;i<=MAX_TASKS;i++){
			name = tcb[i].semacount;
			putsUart0(tcb[i].name);
			putsUart0("\t");
			sprintf(string1,"%u",name);
			putsUart0(string1);
			putsUart0("\n\r");
		}
	}

}

void magnet() {
	while (1) {
		// Read sensor
		readAdc0Ss3();
		uint8_t i = 0;
		uint16_t diff10 = 0;
		uint16_t diff21 = 0;
		diff10 = abs(ADCVals[1] - ADCVals[0]);
		diff21 = abs(ADCVals[2] - ADCVals[1]);
		/*if ((ADCVals[0] < 1285) && (ADCVals[1] < 1285) && (ADCVals[2] < 1285)) {
		 putsUart0("Magnet not found\r\n");
		 }*/
		if ((ADCVals[1] > 1315) && (35 < diff10 < 60)) {
			putsUart0("Centre\r\n");
		}
		if ((ADCVals[0] > 1315) && (35 < diff10 < 60)) {
			putsUart0("20mm ");
			putsUart0("Left\r\n");
		}
		if ((ADCVals[2] > 1315) && (35 < diff21 < 60)) {
			putsUart0("20mm ");
			putsUart0("Right\r\n");
		}
		if ((1295 < ADCVals[1]) && (1295 < ADCVals[0])) {
			if (2 < diff10 < 15) {
				putsUart0("10mm ");
				putsUart0("Left\r\n");
			}
		}
		if ((1295 < ADCVals[1]) && (1295 < ADCVals[2])) {
			if (2 < diff21 < 15) {
				putsUart0("10mm ");
				putsUart0("Right\r\n");
			}
		}
		waitMicrosecond(1000);
		yield();
	} // REQUIRED: add code to read the Hall effect sensors that maintain lane centering and report the lane position relative to center

}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void) {
	bool ok;

	// Initialize hardware
	initHw();
	systickHw();
	rtosInit();

	// Power-up flash
	RED_LED = 1;
	waitMicrosecond(250000);
	RED_LED = 0;
	waitMicrosecond(250000);

	// Initialize semaphores
	init(&keyPressed, 1);
	init(&keyReleased, 0);
	init(&flashReq, 5);

	// Add required idle process
	ok = createThread(idle, "Idle", 15);

	// Add other processes
	ok &= createThread(flash4Hz, "Flash_4hz", 0);
	ok &= createThread(lengthyFn, "Lengthy_fn", 10);
	ok &= createThread(oneshot, "One_shot", 4);
	ok &= createThread(readKeys, "Read_keys", 2);
	ok &= createThread(debounce, "Debounce", 6);
	ok &= createThread(uncooperative, "Uncoop", 7);
	ok &= createThread(shell, "Shell", 12);
	ok &= createThread(magnet, "Magnet", 8);

	// Start up RTOS
	if (ok)
		rtosStart(); // never returns
	else
		RED_LED = 1;

	return 0;
	// don't delete this unreachable code
	// if a function is only called once in your code, it will be
	// accessed with two goto instructions instead of call-return,
	// so any stack-based code will not function correctly
	yield();
	sleep(0);
	wait(0);
	post(0);
}

