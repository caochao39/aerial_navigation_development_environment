## Example udev rule for Linux

When running this driver on Linux, it is possible to get a more user-friendly, "permanent" name for the device by utilizing a udev rule.
The example udev rule here is for the Logitech F710 controller, but should work with minor modifications for most devices.
To figure out the correct name for any controller, first plug the controller in and notice which /dev/input/event* device it is on.
Supposing it is on /dev/input/event7, the following command can be run:

$ udevadm info -a -n /dev/input/event7

This will print out a lot of information, but only the first two entries are interesting.
Find the line that says ATTRS{name}, and copy the value from there into 99-logitech-f710.rules.
Then rename the SYMLINK to whatever you want.  Copy that file into /etc/udev/rules.d/99-<whatever-name>.rules.
Run:

$ udevadm control --reload-rules

Now unplug and replug your device, and the symlink you've chosen should show up in /dev.
