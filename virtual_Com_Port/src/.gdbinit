echo .gdbinit is a hidden file. Press Ctrl-H in the current working directory to see it.\n
# Connect to OpenOCD
<<<<<<< HEAD
target remote localhost:3333
#monitor reset halt
=======
	target remote localhost:3333
monitor reset halt

define bpl
    info breakpoints
end
document bpl
	List all breakpoints.
end
>>>>>>> fd9b2f8d3dee4a198fa5bbdc7fed615f53bc80b6
