/*
 * This code is responsible for controlling the solenoid valves on InSpace's
 * CR25H hybrid rocket during flight.
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
 * (c) Carleton University InSpace 2025
 *
 * Please feel free to contact me with questions about this code if you are an
 * InSpace member from future years.
 *
 * This code does the following:
 *
 * 1. Wait for the continuity line to read low (disconnected)
 * 2. Turn on the MOSFET switches that output 24V battery power on the solenoid
 *    power lines
 * 3. Begin a timer for the flight duration (configurable, somewhere in the
 *    minutes range)
 * 4. Once the timer ends, turn off the MOSFET switches
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>

/* Pre-processor warning helpers */

#define STR(x) #x
#define XSTR(x) STR(x)

/* Logic parameters */

#if F_CPU != 8000000
#error "This program was written with the assumption that CPU_FREQ is 8MHz"
#endif

/* Nominal flight time of CR25H in seconds */

#define NOMINAL_FLIGHT_DUR 72

/*
 * WARNING: The MCU is powered from an unregulated LiPo or Li-Ion battery,
 * which can be anywhere from 4.2V to 2.4V throughout its charge lifespan.
 * Within these ranges, the internal RC oscillator can be between 6-8MHz
 * (datasheet Table 21-2). This timer is calibrated for the fastest scenario
 * of 8MHz, but a slower frequency of 6MHz will result in a longer timer
 * delay. This is preferred over a faster delay, since the recovery team can
 * simply wait longer to recover the rocket, but we cannot finish our flight
 * any faster (if all goes well, anyways).
 */

#if FTIME < NOMINAL_FLIGHT_DUR
#error                                                                         \
    "The flight timer duration cannot be less than the nominal flight duration"
#endif

#define FTIME_MS ((uint32_t)(FTIME) * 1000)
#define MIN_FTIME_MS (FTIME_MS * 1.024)
#define MAX_FTIME_MS (FTIME_MS * 1.36)

#pragma message "Flight time is set to " XSTR(FTIME_MS) " milliseconds"
#pragma message "Minimum flight time is " XSTR(MIN_FTIME_MS) " milliseconds"
#pragma message "Maximum flight time is " XSTR(MAX_FTIME_MS) " milliseconds"

#define DEBOUNCE_DELAY_MS 10

/* Pin definitions */

#define XV4_DIR DDB0
#define XV6_DIR DDB1
#define XV7_DIR DDB2

#define XV4_OUT PB2
#define XV6_OUT PB1
#define XV7_OUT PB0

#define CONT_DIR DDB3
#define CONT_IN PB3
#define CONT_INT_PIN PCINT3

/* Helpers */

#define SOLENOID_MASK (_BV(XV4_OUT) | _BV(XV6_OUT) | _BV(XV7_OUT))
#define solenoids_on() PORTB |= (SOLENOID_MASK)
#define solenoids_off() PORTB &= ~(SOLENOID_MASK)
#define solenoids_are_on ((PORTB & SOLENOID_MASK) == SOLENOID_MASK)

/* Type definitions */

typedef enum {
    STATE_GROUNDED, /* Connected to ground systems */
    STATE_LAUNCHED, /* Disconnected, just launched */
    STATE_FLIGHT,   /* In flight */
    STATE_LANDED,   /* Flight time over, presumed landing */
} state_e;

/* Global variables */

state_e g_state = STATE_GROUNDED;
uint32_t g_time_ms = 0;

/* Function prototypes */

static inline void start_timer(void);
static inline void stop_timer(void);
static inline uint8_t disconnected(void);

/* This is the interrupt vector for detecting a loss of continuity in the
 * continuity line. It puts high voltage on the signal lines that drive the
 * MOSFET switches, giving 24V power the solenoid valves. This interrupt handler
 * is called when the continuity line experiences a change in voltage.
 *
 * PCINT0 is used to detect pin changes. The pin with a change must be verified.
 */
ISR(PCINT0_vect) {

    /* Make sure that the continuity line actually reads low, with debouncing as
     * treat. */

    if (disconnected()) {

        /* We just launched if there was a falling edge detected, which also
         * means the solenoids are no longer ground powered and need to be
         * turned on from the batteries.
         *
         * Only perform this logic if we're coming from the grounded state. This
         * prevents us accidentally returning to the `STATE_LAUNCHED` state
         * because of a second falling edge interrupt, which might happen if the
         * continuity wire ends dangling from the rocket touch each other again
         * by pure chance.
         */

        if (g_state == STATE_GROUNDED) {
            g_state = STATE_LAUNCHED;
            solenoids_on();
        }
    }

    /* NOTE: PCIE interrupt flag is cleared when this ISR returns */
}

/* Timer interrupt vector
 *
 * This is the interrupt vector for detecting a compare match with OCR0A. We
 * will use it to count the number of compare matches, and therefore determine
 * how much time has passed.
 *
 * NOTE: The timer is set up for a period of ~1ms. The nominal flight
 * time for CR25H is 72 seconds according to simulation. The configured flight
 * duration in the `FLIGHT_TIME` should be a good margin larger to account for
 * any unknowns and variability in the CPU clock frequency, since even a 5
 * minute duration provides enough extra margin while still safing the rocket
 * before the recovery team retrieves it.
 */
ISR(TIMER0_COMPA_vect) {
    g_time_ms++; /* Increment the time count in ms by one more */

    /* If the flight time is depleted, then we have landed */

    if (g_time_ms >= FTIME_MS) {
        g_state = STATE_LANDED;
    }

    /* NOTE: OCF0A interrupt flag is cleared when this ISR returns */
}

int main(void) {

    /* Disable all interrupts */

    cli();

    /* When we power on, we assume that we are currently grounded. */

    g_state = STATE_GROUNDED;

    /* Set up continuity line pin */

    DDRB &= ~_BV(CONT_DIR); /* Set continuity pin as input */
    PORTB &= _BV(CONT_IN);  /* Turn off the pull-up resistor */

    /* Set up solenoid valve output pins */

    DDRB &= ~(_BV(XV4_DIR) | _BV(XV6_DIR) |
              _BV(XV7_DIR)); /* Set solenoids as outputs */
    solenoids_off();         /* Set both to low */

    /* Configure the flight timer.
     *
     * The flight timer is configured in count-up mode, with:
     * Clock source: CLK_IO (8MHz)
     * Divider: 256
     * MAX: 31
     *
     * This will generate an overflow interrupt every 1.024ms. This is good
     * enough for our purposes, and we will consider it 1ms.
     *
     * NOTE: if the CPU frequency drops to 6MHz because of the voltage changes,
     * then the clock resolution is 1.36ms. This makes our flight duration 36%
     * longer, which is fine.
     */

    stop_timer(); /* Stop timer before we configure things  */

    TCCR0B = _BV(CS02);  /* Divide clock frequency by 256 */
    TCCR0A = _BV(WGM01); /* Put timer in CTC mode (count up from 0 to the value
                            in OCR0A) */
    TCNT0 = 0;           /* Clear timer counter */
    OCR0A = 31;          /* Set the MAX value for the counter */

    /* Clear pending interrupts */

    GIFR |= _BV(PCIF); /* Clears external PCIE interrupts */

    /* Enable interrupts for continuity detect and timer */

    TIMSK |= _BV(OCIE0A);       /* Enables timer 0 compare A interrupt */
    PCMSK |= _BV(CONT_INT_PIN); /* Enables PCIE interrupt for continuity pin */
    GIMSK |= _BV(PCIE);         /* Enables PCIE interrupt */
    sei(); /* Enable interrupts globally (must be last step) */

    /* Loop forever, let interrupt routines handle state transitions  */

    for (;;) {

        switch (g_state) {

        case STATE_GROUNDED:

            /* We are still connected to ground systems, nothing to be done.
             * Just make sure to keep the solenoids turned off.
             */

            if (solenoids_are_on) solenoids_off();

            /* As an extra precaution, here we check the status of the
             * continuity line. If we were to lose power to the MCU in
             * flight, we'd miss the falling edge interrupt and would
             * therefore fail to power the solenoids in flight.
             *
             * This check ensures that if we regain power in this scenario,
             * we can still detect that we've launched and can turn the
             * solenoids on again.
             *
             * The PCIE interrupt is still important to have, since it will have
             * a (marginally) quicker response time to a disconnect. This is
             * just an extra precaution.
             */

            if (disconnected()) {
                g_state = STATE_LAUNCHED;
            }

            break;

        case STATE_LAUNCHED:

            /* We must have just disconnected from the ground systems.
             *
             * Set up the flight duration timer.
             * Turn on the solenoids if they aren't on yet
             * Indicate that we are now in flight.
             */

            if (!solenoids_are_on) solenoids_on();
            start_timer();
            g_state = STATE_FLIGHT;

            break;

        case STATE_FLIGHT:

            /* We are flying, the timer is running. Nothing to do.
             * Just make sure we keep the solenoids on.
             */

            if (!solenoids_are_on) solenoids_on();

            break;

        case STATE_LANDED:

            /* We have landed. Time to turn off the solenoids and the flight
             * timer. Otherwise do nothing.
             */

            stop_timer();
            if (solenoids_are_on) solenoids_off();

            break;
        }
    }
}

/*
 * This function stops the flight duration timer from counting by holding it in
 * reset.
 */
static inline void stop_timer(void) {
    /* Keep the value in PSR0 bit asserted forever */

    GTCCR |= _BV(TSM);

    /* Keep the timer 0 pre-scaler reset, hence the clock halted */

    GTCCR |= _BV(PSR0);

    return;
}

/*
 * This function starts the flight duration timer from 0.
 */
static inline void start_timer(void) {
    g_time_ms = 0; /* Reset program millisecond counter */
    TCNT0 = 0;     /* Reset timer counter */
    GTCCR &= ~(_BV(TSM) |
               _BV(PSR0)); /* Turn off timer reset assert (timer starts) */
    return;
}

/*
 * Checks (with debouncing) if the continuity line is disconnected.
 *
 * WARNING: This function returns false if the continuity line is not 0 when it
 * is first called. This might fail if we are so unlucky as to call this
 * function during some transient 'high' spike. This can be alleviated by adding
 * some delay before calling this function to let things settle.
 *
 * NOTE: debounce duration is configured by macro `DEBOUNCE_DELAY_MS`
 *
 * Returns 0 for connected, 1 for disconnected.
 */
static inline uint8_t disconnected(void) {

    /* Check for a disconnect */

    if (!(PINB & _BV(CONT_IN))) {

        /* Wait some delay before checking again */

        _delay_ms(DEBOUNCE_DELAY_MS);

        /* If still low, then we are definitely disconnected */

        if (!(PINB & _BV(CONT_IN))) {
            return 1;
        }
    }

    return 0;
}
