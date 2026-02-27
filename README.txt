INSTRUCTIONS FOR SERIAL PORT PROTOCOL
=====================================

Project Structure
-----------------

- bin/: Compiled binaries.
- src/: Source code for the implementation of the link-layer and application layer protocols. Students should edit these files to implement the project.
- cable/: Virtual cable program to help test the serial port. This file must not be changed.
- Makefile: Makefile to build the project and run the application.
- penguin.gif: Example file to be sent through the serial port.

Instructions to Run the Project
-------------------------------

1. Compile the application and the virtual cable program using the provided Makefile.
2. Run the virtual cable program (either by running the executable manually or using the Makefile target).
   Note that the virtual cable program requires the installation of "socat".
    (Option 1) $ sudo ./bin/cable_app
    (Option 2) $ sudo make run_cable

3. Test the protocol without cable disconnections and noise
    3.1 Run the receiver (either by running the executable manually or using the Makefile target):
        (Option 1) $ ./bin/main /dev/ttyS11 9600 rx penguin-received.gif
        (Option 2) $ make run_rx

    3.2 Run the transmitter (either by running the executable manually or using the Makefile target):
        (Option 1) $ ./bin/main /dev/ttyS10 9600 tx penguin.gif
        (Option 2) $ make run_tx

    3.3 Check if the file received matches the file sent, using the diff Linux command or using the Makefile target:
        (Option 1) $ diff -s penguin.gif penguin-received.gif
        (Option 2) $ make check_files
