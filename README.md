# rfnm_gnuradio_sink
A custom sink module and preliminary tests for the rfnm.com SDR 

This uses the beta firmware for the rfnm module which has to be downloaded and flashed seperatly via a USB drive. Information about this can be obtained in the rfnm discord.

The rfnm operates with two parts. Firstly there is the i.MX8 that handles the USB, Ethernet or local data handling and applications. 
In the beta branch a armbian embedded linux is used which allows for convient update and installation of packages like gnuradio.
The SDR part is handled by the NXP LA9310 and the chosen add-on board. To communicate with the LA9310 the kernel level driver listens to some
sysfs files and syscalls. The data transfer is essentially just a ring buffer in the DDR4 memory that is triggered by a syscall. The LA9310 gets this data then via PCIe.

The module generates a synchronous gnuradio block that will throttle the data rate automatically.