rosc_entropy
============

Digital entropy source based on on jitter between multiple, digital ring
oscillators. The entropy source is used in the TRNG as one of several
entropy sources feeding the mixer.


## Functionality ##

The digital oscillators are created using adders. The carry out from the
adder are inverted and fed back into the adder as carry in. In
combination with operand values we basically get an inverted signal (the
carry out) that toggles reapeatedly. By having the operands externally
defined, synthesis tools in general will not optimize them away.

The carry out signal is sampled with a clock that toggles at a much
slower rate than the intrinsic toggle rate of the carry out signal. In a
modern FPGA, the toggle rate may be 400+ MHz while the sample rate might
be 10 kHz. This sample time allows the differences in intrinsic toggle
frequency between separate oscillators to drift apart after sampling.

The entropy source contains 32 separate oscillators. The outputs from
the oscillators are XOR-combined to create a single entropy bit. Entropy
bits are collected into 32-bit words which are provided to entropy
consumers.


## Implementation Results ##

The entropy source has been implemented, tested and shown to produce
good quality entropy (using the ent estimation tool etc) in Altera
Cyclone-IV and Cyclone-V devices as well as in Xilinx Spartan-6.

The Xilinx synthesis tool will try to optimize the combinational loop
away. (More specifically, it claims that the oscillator sample registers
will have a fixed value.). There is therefore an attribute in the source
code to force the tool to preserve the register.
