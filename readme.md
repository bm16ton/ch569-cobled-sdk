An attempt at usb cdcacm on wch569. The code is in samples/gud. Currently basic uart rx/tx works for usb 2, on usb3 It (sorta) works for usb to uart but uart to usb doesnt show anything. Still havent gotten a grasp on order of operations or even the required ops for usb3 stuff yet. Also line coding is anissue, i see the data in wireshark but only the basic brequest/length etc can i find in the buffer, everything after that, the bytes conatining the encoding data, is static and not the data. Sure its a stupid simple mistake but have examined everybyte in buffer and dont see it.

So Ive added the coding struct and the various functions for setting/reading the uart encoding but its currently untested and useless till I figure out how to read the packet/buffer correctly to get the settings.

Edited the ld script to show ram usage like previous toolchain versions used to.

Added genric board support files (generic based on the official board design from wch)

Will probly drop the teenyusb side of things and just simply ass the cdc stuff to the hydrausb files.

My ld/startup.S assembly knowledge is non-existent, but I want to look into where the code is actually copying from flash to xram, it wasnt obvious to me.

I compiled the tooolchain myself but I dont think thats the reason the missing symbols caused by nosyslibs are present (same as on arm) but I am surprised that no stubs where include in hydrausb code, I added them.

I think thats about it. Once I figure out how to properly initiate a transfer from mcu to host, and how/where to read the cdc encoding data the rest should fall in-line fairly quickly. The linux kernels usb gadget for cdcacm seems to have usb3 superspeed support so i belive the usbcdc driver should be all set. After this time permitting Ill hopefully give adding gud a driveby.

Oh and adding makefile options for genric/hydrausb and gnu toolchain/wch toolchain switching easier
