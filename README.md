# sample_scurvegen
Verilog S-curve generator for stepper control

This is a snippet of a larger FPGA design and C++ program to drive it. These parts are concerned with
the stepper step/dir generation logic and the code to calculate the timing of the move.

This is only "fyi" reference code, provided as an example. It's not intended to be a complete solution
and it's heavily geared to my specific OpenPnP build. Your mileage will definitely vary.

The sums are derived from the work by Mark, Jaroslaw and other contributors on this thread:
https://groups.google.com/g/openpnp/c/bEVZvYoXO98/m/hl14FcspBwAJ
And on the course notes here:
http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf
Thanks for all that hard work.
