/* Simple Sinewave DDS Routine
 * Speedy version -- using bit-shifts in the accumulator.
 * See https://github.com/hexagon5un/barking_dogs_dds_demo
 * Public Domain
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include "scale16.h"
#include "sineWav.h"

#define clear_bit(sfr, bit)   (_SFR_BYTE(sfr) &= ~_BV(bit))
#define set_bit(sfr, bit)     (_SFR_BYTE(sfr) |= _BV(bit))
#define toggle_bit(sfr, bit)  (_SFR_BYTE(sfr) ^= _BV(bit))

#define SAMPLE_RATE       24000UL
#define ACCUMULATOR_STEPS 2048

/* Init functions, defined at the bottom of the file */
static inline void setup_pwm_audio_timer(void);
static inline void setup_sample_timer(void);

struct DDS {
  uint16_t increment = ACCUMULATOR_STEPS;
  uint16_t position = 0;
  uint16_t accumulator = 0;
};
volatile struct DDS sineTone;

const uint16_t sineTone_size = sizeof(ui8SineLUT);

ISR(TIMER1_COMPA_vect)
{
  PORTB |= (1 << PB1); // debug, toggle pin PB1, Arduino D9

  OCR2A = (int8_t)pgm_read_byte_near(ui8SineLUT + sineTone.position);

  if (sineTone.position < sineTone_size)
  {    /* playing */
      sineTone.accumulator += sineTone.increment;
      sineTone.position += sineTone.accumulator / ACCUMULATOR_STEPS;
      sineTone.accumulator = sineTone.accumulator % ACCUMULATOR_STEPS;
  } else
  {  /*  done playing, reset and wait  */
      sineTone.position = 0;
  }

  PORTB &= ~(1 << PB1); // debug, toggle pin
}

void setup()
{
  setup_sample_timer();
  setup_pwm_audio_timer();
  set_bit(DDRB, PB1); // debugging

  sineTone.increment = 1;
}

void loop()
{
}


static inline void setup_sample_timer()
{
  // Undo Arduino defaults:
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  // Set up in count-till-clear mode
  set_bit(TCCR1B, WGM12);
  // Sync to CPU clock: fast
  set_bit(TCCR1B, CS10);
  // Enable output-compare interrupt
  set_bit(TIMSK1, OCIE1A);
  OCR1A = F_CPU / SAMPLE_RATE - 1;
  sei();
}

static inline void setup_pwm_audio_timer()
{
  // Undo Arduino defaults:
  TCCR2A = 0;
  TCCR2B = 0;
  // Set fast PWM mode
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  // With output on OC2A / Arduino pin 11
  set_bit(TCCR2A, COM2A1);
  // Set fastest clock speed: ~62kHz @ 16MHz
  set_bit(TCCR2B, CS20);
  // Signal is symmetric around 127
  OCR2A = 127;
  // output on pin 11 -- OC2A
  set_bit(DDRB, PB3);  /* or pinMode(11, OUTPUT) in Arduinese */
}
