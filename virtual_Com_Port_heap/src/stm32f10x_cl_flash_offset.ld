/* default stack sizes. */

__Stack_Size = 1024 ;

PROVIDE ( _Stack_Size = __Stack_Size ) ;

__Stack_Init = _estack  - __Stack_Size ;

/*"PROVIDE" allows to easily override these values from an object file or the commmand line.*/
PROVIDE ( _Stack_Init = __Stack_Init ) ;

/*
There will be a link error if there is not this amount of RAM free at the end.
*/
_Minimum_Stack_Size = 0x100 ;


/* Memory Spaces Definitions */

MEMORY
{
  	RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 64K
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
	xn (rwx) : ORIGIN = 0xC0000000, LENGTH = 512K
}

/* higher address of the user mode stack */
_estack = 0x20010000;



/* include the sections management sub-script for FLASH mode */

/* Sections Definitions */

SECTIONS
{
    /* for Cortex devices, the beginning of the startup code is stored in the .isr_vector section, which goes to FLASH */
    .isr_vectorsflash :
    {
	. = ALIGN(4);
        KEEP(*(.isr_vectorsflash))            /* Startup code */
	. = ALIGN(4);
    } >FLASH
 
   
    /* the program code is stored in the .text section, which goes to Flash */
    .text :
    {
	    . = ALIGN(4);
	*(.handlerfunctions)	   
	*(Reset_Handler*) 
         *(.text)                    /* remaining code */
         *(.text.*)                    /* remaining code */
        *(.rodata)                 /* read-only data (constants) */
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
/*	*(.xnfunction)*/
	    . = ALIGN(4);
   	 _etext = . ;
	    /* This is used by the startup in order to initialize the .data secion */
   	 _sidata = _etext;
    } >FLASH
    
    .xntext : AT(_etext) {
		*(.dummyfunction*)	
		*(.xnfunction*)		
	
	_exn = . ; 	
    } > xn
 

	_sidata = _etext + SIZEOF(.xntext);
    /* This is the initialized data section
    The program executes knowing that the data is in the RAM
    but the loader puts the initial values in the FLASH (inidata).
    It is one task of the startup to copy the initial values from FLASH to RAM. */
    .data  : AT ( _sidata )
    {
	    . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _sdata = . ;
        
        *(.data)
        *(.data.*)
	*(.symtab);
	    . = ALIGN(4);
	    /* This is used by the startup in order to initialize the .data secion */
   	 _edata = . ;
    } >RAM
    
    

    /* This is the uninitialized data section */
    .bss :
    {
	    . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;
        
        *(.bss)
        *(COMMON)
        
	    . = ALIGN(4);
	    /* This is used by the startup in order to initialize the .bss secion */
   	 _ebss = . ;
    } >RAM
    
    PROVIDE ( end = _ebss );
    PROVIDE ( _end = _ebss );
    
    /* This is the user stack section 
    This is just to check that there is enough RAM left for the User mode stack
    It should generate an error if it's full.
     */
    ._usrstack :
    {
	    . = ALIGN(4);
        _susrstack = . ;
        
        . = . + _Minimum_Stack_Size ;
        
	    . = ALIGN(4);
        _eusrstack = . ;
    } >RAM
    

    /* after that it's only debugging information. */
    
    /* remove the debugging information from the standard libraries */
    DISCARD :
    {
     libc.a ( * )
     libm.a ( * )
     libgcc.a ( * )
     }
	
}
