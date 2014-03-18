#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "micros.h"


#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (1)

// the whole number of milliseconds per timer0 overflow
#define MICROS_INC MICROSECONDS_PER_TIMER0_OVERFLOW

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_micros = 0;
static unsigned char timer0_fract = 0;

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_micros;
	m += MICROS_INC;
	timer0_micros = m;
	timer0_overflow_count++;
}

unsigned long micros() {
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_micros or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_micros)
	cli();
	m = timer0_micros;
	SREG = oldSREG;
	return m;
}
