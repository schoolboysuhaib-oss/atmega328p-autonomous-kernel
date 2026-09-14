/*
=============================================================================
PROJECT:    BARE-METAL INTELLIGENT AUTONOMOUS RECOVERY ENGINE (V2.7.2-PROD)
TARGET:     ATmega328P (Arduino Uno R3 / Custom Target Hardware)
PARADIGM:   Ultra-Low Overhead, Direct Hardware Register Manipulation, 
            Asynchronous Non-Blocking Pulse-Width Timing, System Clock Synthesis, 
            and 32kHz Ultrasonic Silent Phase-Correct PWM Motor Control.
=============================================================================
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdbool.h>

// --- Custom Low-Overhead Utility Macros ---
#define PROD_CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// --- Static FSM Operational States ---
#define STATE_FORWARD_CRUISE     0
#define STATE_DYNAMIC_AVOIDANCE   1
#define STATE_STUCK_RECOVERY     2
#define STATE_EMERGENCY_BRAKE    3

// --- Calibration Thresholds (Integer Optimization) ---
const int SAFETY_SETPOINT_CM   = 25;
const int CRITICAL_THRESHOLD_CM = 8;
const int KP                    = 12;

// --- Global Volatile Variables for ISR Safety ---
volatile unsigned long systemTicks = 0;
volatile uint8_t currentState = STATE_FORWARD_CRUISE; 
volatile unsigned long stateChangeTimestamp = 0;

// High-precision synthesized timestamps in microseconds
volatile unsigned long sonar1_start_us = 0;
volatile unsigned long sonar1_echo_us  = 0;
volatile unsigned long sonar2_start_us = 0;
volatile unsigned long sonar2_echo_us  = 0;

int currentDistanceRight     = 100;
int currentDistanceLeft      = 100;
unsigned long lastSonarTick  = 0;
unsigned long lastPITick     = 0;
unsigned long lastTelemetryTick = 0;
bool alternateSonarToggle    = false;

// --- Custom Microsecond Assembly Delay ---
static inline void delayMicrosecondsCustom(unsigned int us) {
    unsigned int loops = us << 2;
    __asm__ __volatile__ (
        "1: sbiw %0, 1" "\n\t"
        "brne 1b"
        : "=w" (loops)
        : "0" (loops)
    );
}

// --- High-Performance Microsecond Timing Core ---
static inline unsigned long getSystemMicrosRaw(void) {
    unsigned long ticks = systemTicks;
    uint8_t tcnt = TCNT2;
    if ((TIFR2 & (1 << OCF2A)) && (tcnt < 249)) {
        ticks++;
    }
    return (ticks * 1000UL) + (tcnt * 4UL);
}

// --- Native UART Hardware Serial Register Drivers ---
void uart_init_115200(void) {
    UBRR0H = 0;
    UBRR0L = 16; // 16MHz clock with Double Speed Enabled = 115200 Baud
    UCSR0A |= (1 << U2X0);
    UCSR0B = (1 << TXEN0); 
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}

void uart_transmit_byte(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0))); 
    UDR0 = data;
}

void uart_print_str(const char* str) {
    while (*str) {
        uart_transmit_byte((uint8_t)*str++);
    }
}

// Fixed zero-array recursive integer printer with clean unsigned negation casting
void uart_print_int_helper(unsigned int val) {
    if (val / 10) {
        uart_print_int_helper(val / 10);
    }
    uart_transmit_byte((val % 10) + '0');
}

void uart_print_int(int value) {
    if (value == 0) {
        uart_transmit_byte('0');
        return;
    }
    unsigned int uval;
    if (value < 0) {
        uart_transmit_byte('-');
        uval = -((unsigned int)value); // Fixed INT_MIN undefined behavior negation bug
    } else {
        uval = (unsigned int)value;
    }
    uart_print_int_helper(uval);
}

// --- Hardware Initialization Core ---
void setupBareMetalHardware(void) {
    cli();
    
    // Direct Register Configuration for Pin Outputs
    DDRD |= (1 << DDD7) | (1 << DDD2); // Sonar Triggers
    DDRB |= (1 << DDB1) | (1 << DDB2); // Motors (Pin 9 & 10)

    // Pin Inputs
    DDRB &= ~(1 << DDB3); // Echo 1
    DDRD &= ~(1 << DDD5); // Echo 2

    // Pin Change Interrupt Masks (PCINT)
    PCICR  |= (1 << PCIE0) | (1 << PCIE2);
    PCMSK0 |= (1 << PCINT3);
    PCMSK2 |= (1 << PCINT21);

    // Timer2 Clock Configuration: 1ms Heartbeat Clock CTC Mode
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22); // Prescaler 64
    OCR2A  = 249;
    TIMSK2 |= (1 << OCIE2A);

    // Timer1 Architecture: 32kHz Silent Phase-Correct PWM Engine (Mode 10)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << CS10); // Prescaler 1 (Mode 10 perfectly targeted)
    ICR1   = 250; 

    OCR1A = 0;
    OCR1B = 0;
    
    uart_init_115200();
    sei();
}

// --- Interrupt Service Routines ---
ISR(TIMER2_COMPA_vect) {
    systemTicks++;
}

ISR(PCINT0_vect) {
    if (PINB & (1 << PINB3)) {
        sonar1_start_us = getSystemMicrosRaw();
    } else {
        sonar1_echo_us = getSystemMicrosRaw() - sonar1_start_us;
    }
}

ISR(PCINT2_vect) {
    if (PIND & (1 << PIND5)) {
        sonar2_start_us = getSystemMicrosRaw();
    } else {
        sonar2_echo_us = getSystemMicrosRaw() - sonar2_start_us;
    }
}

void triggerAsynchronousSonar(bool rightSensor) {
    if (rightSensor) {
        PORTD &= ~(1 << PORTD7); delayMicrosecondsCustom(2);
        PORTD |= (1 << PORTD7);  delayMicrosecondsCustom(10);
        PORTD &= ~(1 << PORTD7);
    } else {
        PORTD &= ~(1 << PORTD2); delayMicrosecondsCustom(2);
        PORTD |= (1 << PORTD2);  delayMicrosecondsCustom(10);
        PORTD &= ~(1 << PORTD2);
    }
}

void executePControlLoop(void) {
    int errorRight = SAFETY_SETPOINT_CM - currentDistanceRight;
    int errorLeft  = SAFETY_SETPOINT_CM - currentDistanceLeft;

    int outputRight = errorRight * KP;
    int outputLeft  = errorLeft * KP;

    int targetSpeedRight = PROD_CONSTRAIN(200 + outputLeft, 40, 240);
    int targetSpeedLeft  = PROD_CONSTRAIN(200 + outputRight, 40, 240);

    OCR1A = targetSpeedRight;
    OCR1B = targetSpeedLeft;
}

void transitionToState(uint8_t newState) {
    currentState = newState;
    stateChangeTimestamp = systemTicks;
}

void setup(void) {
    setupBareMetalHardware();
    transitionToState(STATE_FORWARD_CRUISE);
}

void loop(void) {
    unsigned long currentMillis = systemTicks;

    // 60ms Window: Asynchronous Metric Isolation Engine
    if (currentMillis - lastSonarTick >= 60) {
        lastSonarTick = currentMillis;
        unsigned long local_echo_right;
        unsigned long local_echo_left;

        // Custom Atomic tracking state preservation block
        uint8_t sreg_save = SREG; 
        cli();
        local_echo_right = sonar1_echo_us;
        local_echo_left  = sonar2_echo_us;
        SREG = sreg_save;

        currentDistanceRight = (int)((local_echo_right / 2.0) / 29.15);
        currentDistanceLeft  = (int)((local_echo_left / 2.0) / 29.15);

        if (currentDistanceRight <= 0 || currentDistanceRight > 400) currentDistanceRight = 400;
        if (currentDistanceLeft  <= 0 || currentDistanceLeft > 400)  currentDistanceLeft  = 400;

        triggerAsynchronousSonar(alternateSonarToggle);
        alternateSonarToggle = !alternateSonarToggle;
    }

    // Global Safety Intercept Watchdog Check
    if (currentState != STATE_EMERGENCY_BRAKE) {
        if (currentDistanceRight < CRITICAL_THRESHOLD_CM || currentDistanceLeft < CRITICAL_THRESHOLD_CM) {
            transitionToState(STATE_EMERGENCY_BRAKE);
        }
    }

    // FSM Execution Tree
    switch (currentState) {
        case STATE_FORWARD_CRUISE:
            OCR1A = 215;
            OCR1B = 215;
            if (currentDistanceRight < SAFETY_SETPOINT_CM || currentDistanceLeft < SAFETY_SETPOINT_CM) {
                transitionToState(STATE_DYNAMIC_AVOIDANCE);
            }
            break;

        case STATE_DYNAMIC_AVOIDANCE:
            if (currentMillis - lastPITick >= 30) {
                lastPITick = currentMillis;
                executePControlLoop();
            }
            if (currentMillis - stateChangeTimestamp > 3500) {
                transitionToState(STATE_STUCK_RECOVERY);
            } else if (currentDistanceRight >= SAFETY_SETPOINT_CM && currentDistanceLeft >= SAFETY_SETPOINT_CM) {
                transitionToState(STATE_FORWARD_CRUISE);
            }
            break;

        case STATE_STUCK_RECOVERY:
            if (currentMillis - stateChangeTimestamp < 1000) {
                OCR1A = 100; OCR1B = 0;
            } else if (currentMillis - stateChangeTimestamp < 2200) {
                OCR1A = 160; OCR1B = 0;
            } else {
                transitionToState(STATE_FORWARD_CRUISE);
            }
            break;

        case STATE_EMERGENCY_BRAKE:
            OCR1A = 0;
            OCR1B = 0;
            if (currentDistanceRight > SAFETY_SETPOINT_CM && currentDistanceLeft > SAFETY_SETPOINT_CM) {
                transitionToState(STATE_FORWARD_CRUISE);
            }
            break;
    }

    // 150ms Low-Overhead Hardware Serial Telemetry Pipeline
    if (currentMillis - lastTelemetryTick >= 150) {
        lastTelemetryTick = currentMillis;
        uart_print_str("[BARE_STATE:");     uart_print_int(currentState);
        uart_print_str("] | R_CM:");       uart_print_int(currentDistanceRight);
        uart_print_str(" | L_CM:");        uart_print_int(currentDistanceLeft);
        uart_print_str("\r\n");
    }
}

// Standalone Bare-Metal Toolchain Entry point execution hook
int main(void) {
    setup();
    while (1) {
        loop();
    }
    return 0;
}

