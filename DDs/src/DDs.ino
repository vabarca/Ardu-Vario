#include <Arduino.h>

/* Random Dogs Barking
 * a direct-digital synthesis "tutorial"
 * for the AVR, but will compile on an Arduino
 * Released public domain, because it's absurd, frankly.
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include "scale16.h"
#include "sineWav.h"

#define clear_bit(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define set_bit(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))
#define LED_ON              set_bit(PORTB, PB5) // debug, pin PB1, Arduino D9
#define LED_OFF             clear_bit(PORTB, PB5)
#define LED_INIT            set_bit(DDRB, PB5) /* debugging -- this pin toggled in ISR  */

  /* Note: If ACCUMULATOR_STEPS is a power of two,
   * the math works fast and all is well.
   * If not, the division and modulo by ACCUMULATOR_STEPS kills it.
   * */
#define ACCUMULATOR_STEPS 2048
#define SAMPLE_RATE       8000UL
#define NUM_TONES         1

/* Init functions, defined at the bottom of the file */
static inline void setup_pwm_audio_timer(void);
static inline void setup_sample_timer(void);
static inline void init_ouput_pins(void);

const uint16_t ui16SineLUT_size = sizeof(ui8SineLUT);

struct structTone
{
  uint16_t increment = ACCUMULATOR_STEPS;
  uint16_t position = 0;
  uint16_t accumulator = 0;
};
volatile struct structTone Tone;

ISR(TIMER1_COMPA_vect)
{
  LED_ON; // debug, toggle pin

  OCR2A = (int8_t)pgm_read_byte_near(ui8SineLUT + Tone.position);

  if (Tone.position < ui16SineLUT_size)
  {    /* playing */
    Tone.accumulator += Tone.increment;
    Tone.position += Tone.accumulator / ACCUMULATOR_STEPS;
    Tone.accumulator = Tone.accumulator % ACCUMULATOR_STEPS;
  }
  else
  {  /*  done playing, reset and wait  */
    Tone.position = 0;
    Tone.increment = 0;
  }

  LED_OFF; // debug, toggle pin
}

// These constants are defined in "scale16.h"
uint16_t scale[] = {NOTE_C1, NOTE_E1, NOTE_G1, NOTE_C2, NOTE_E2, NOTE_G2, NOTE_C3};
const uint8_t scale_max = sizeof(scale)/sizeof(scale[0]);

void setup()
{
  setup_sample_timer();
  setup_pwm_audio_timer();

  LED_INIT;
}

void loop()
{
  /* Demo the entire scale */
  for (uint8_t i=0; i < scale_max; i++)
  {
    Tone.increment = scale[i];
    set_bit(PORTC, PC0);   /* Turn on LED for light show. */
    _delay_ms(2000);
  }
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
