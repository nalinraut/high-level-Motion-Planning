CONTENTS:
This folder contains the files needed for the Baxter remote start device:
	ethernet_relay/ethernet_relay.ino -- The Arduino code for receiving ethernet messages and controlling the relay that turns Baxter on
	remote_start.py -- Sends ethernet commands to the Arduino for turning Baxter on and off.

HARDWARE DESCRIPTION:
The remote start hardware replaces the button used to turn Baxter on and off. Baxter
will turn on or off whenever the two control wires are shorted, as normally done by
the button.

For this remote start, we replace the button with a relay that we control over
ethernet using an Arduino with a Ethernet shield. When the proper message is sent
to the Arduino's IP, the relay will close for half a second, simulating a button press.

Also, the device is covered in electrical tape on each side to help prevent accidental 
short circuits. Without proper insulation, the metal housing of the Baxter's base can
cause a short circuit and turn the Baxter on and off unexpectedly.

UPDATING IP:
If the Arduino's IP changes, you can find it by opening the Arduino IDE and opening
File->Examples->Ethernet->DhcpAddressPrinter. Upload this to the Arduino and open the
Serial Monitor in the IDE to get the new IP address. Make sure to re-upload the remote
start code after doing this.

TODO:
1. Wire a button in parallel with the remote start device so Baxter can be started
	manually.
2. Create a new side panel and a case for the remote start device and button. 