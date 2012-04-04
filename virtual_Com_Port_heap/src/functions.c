
#include "printf.h"
unsigned *program_counter, pc;

__attribute__((section(".dummyfunction")))
void dummyfunction()
{
}


__attribute__((section(".xnfunction")))
volatile int increment(int var)
{
	//int var;
	/* Getting PC and printing */
	asm(
	"MOV R0, PC;"
	);	
	asm("mov %0, r0" : "=r"(program_counter));
//	pc = program_counter;

	printf("\r\n\r\nIncrement FUNCTION called PC : 0x%x", program_counter);
//	var = 0;	
//	main();
	asm(
	"movw	ip, #1\n"
	"movt	ip, #49152\n"
	"blx	ip\n"
	);
	return var+1;

}

__attribute__((section(".xnfunction")))
volatile int decrement(int var)
{
//	unsigned *program_counter, pc;
//	int var;
	asm(
	"MOV R0, PC;"
	);	
	asm("mov %0, r0" : "=r"(program_counter));
//	pc = program_counter;

	printf("\r\n\r\nDecrement FUNCTION called PC : 0x%x", program_counter);
//	var = 0;
	asm(
	"movw	ip, #1\n"
	"movt	ip, #49152\n"
	"blx	ip\n"
	);
	
	return var-1;
}



__attribute__((section(".xnfunction")))
volatile int substraction(int num1, int num2)
{
//	unsigned *program_counter, pc;
//	int var;
	asm(
	"MOV R0, PC;"
	);	
	asm("mov %0, r0" : "=r"(program_counter));
//	pc = program_counter;

	printf("\r\n\r\nDecrement FUNCTION called PC : 0x%x", program_counter);
//	var = 0;
	asm(
	"movw	ip, #1\n"
	"movt	ip, #49152\n"
	"blx	ip\n"
	);
	
	return num1 - num2;
}



__attribute__((section(".xnfunction")))
volatile int addition(int num1, int num2)
{
//	unsigned *program_counter, pc;
//	int var;
	asm(
	"MOV R0, PC;"
	);	
	asm("mov %0, r0" : "=r"(program_counter));
//	pc = program_counter;

	printf("\r\n\r\nDecrement FUNCTION called PC : 0x%x", program_counter);
//	var = 0;
	asm(
	"movw	ip, #1\n"
	"movt	ip, #49152\n"
	"blx	ip\n"
	);
	
	return num1 + num2;
}




