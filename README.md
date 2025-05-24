# Sine wave WAV generator

This is just a silly little program I wrote for fun. Use at your own risk!

Requires some recent version of NASM to build (I'm using 2.16, but something recent should be fine). x64 only, of
course, and it uses AVX instructions, so it needs a CPU that isn't too old (mid 2000s should be fine).

Build it with `make`. `make clean` and `make debug` do what you expect.

* * *

## Usage

This program outputs to standard output, so use something like `> foobar.wav` to capture the output into a file, or
pipe it into a program that will play it or convert it to another format.

Command-line arguments are either options or wave specifications. The program will output a sound file that contains
the result of adding all the specified waves together (clamped to the valid output range).

Each wave specification must contain a frequency, in Hertz. Any positive number is accepted here: even fractional
frequencies. Additionally, the amplitude of the wave and/or the initial phase of the wave may be specified by
following the frequency with `a` or `p` and the corresponding value, without spaces; the initial phase is given in
fractions of a full period (e.g., 0.25 indicates a wave that starts a quarter period later, making it a cosine wave
instead of a sine wave). The defaults for amplitude and phase are 1 and 0 respectively.

Since the only reasonable values for the phase are between 0 and 1, and the amplitude will almost always be between 0
and 1 as well, the initial `0.` may be omitted: if the value contains no decimal dots, the program will assume that
it is a fractional part. For example, `240a25` and `240a0.25` specify the same wave (a 240 Hz wave with an amplitude
of 0.25).

Valid options are:

- `-b`, `-w`, `-q`: set the output format to 8-bit PCM, 16-bit PCM or 32-bit floats, respectively. (Default: 16-bit
  PCM.)
- `-f frequency`: set the output frequency in Hertz (default: 44100). This value must be an integer.
- `-l length`, `-s samples`: set the output length (default: 5 seconds). `-l` sets the value in seconds, while `-s`
  sets the value in samples (which must be an integer).
- `-v volume`: set the overall volume for the output, in dB. This value cannot be positive: if the value does not
  begin with a minus sign, a minus sign is implicitly added. (Therefore, `-v 12` and `-v -12` both set the output
  volume to -12 dB.)
- `-h`: show usage instructions and exit.
