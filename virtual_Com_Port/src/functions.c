
#include "printf.h"
static int a;

__attribute__((section(".xnfunction")))
volatile int sum()
{
	unsigned *program_counter, pc;
	asm(
	"MOV R0, PC;"
	"MOV R0, PC;"
	);	
	asm("mov %0, r0" : "=r"(program_counter));
	pc = program_counter;
	printf("\r\n\r\nSUM FUNCTION called PC : 0x%x", pc);
	return 2;

}

__attribute__((section(".xnfunction")))
int sub()
{
	unsigned *program_counter, pc;
	asm(
	"MOV R0, PC;"
	"MOV R0, PC;"
	);	
	asm("mov %0, r0" : "=r"(program_counter));
	pc = program_counter;
	printf("\r\n\r\nSUB FUNCTION called PC : 0x%x", pc);
	return 1;
}

