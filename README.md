# alx standalone development tree

The puropose of this development tree is to enable us to do development
on alx for both BSD and Linux with a single unified repository. This enables
us to synchronize fixes for both BSD and Linux.

# Linux support

Linux support targets the alx driver to be built in synch with
linux-next.git as the base development tree. Backport kernel support
is provided by utilizing the compat-drivers framework.

To synch to the latest compat-drivers clone compat-drivers and run:

./refresh-compat

This will copy over all code needed to build the driver for Linux.

# BSD support

TBD
