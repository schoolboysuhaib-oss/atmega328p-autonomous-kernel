# Autonomous Search-and-Recovery Kinetic Architecture (V2.7.1)

A production-grade, bare-metal AVR operating kernel written in C++ optimizing the ATmega328P hardware layer. This architecture completely bypasses standard Arduino library bloat to maximize CPU cycle efficiency.

## 🛠️ Core Engineering Highlights
* **Asynchronous Task Scheduler:** Engineered a 1ms system heartbeat timer via Timer2 registers to replace blocking delays, ensuring deterministic execution loops.
* **Direct Register Manipulation:** Configured native DDR and PORT registers for immediate bitwise pin-state changes, minimizing processing latency.
* **Thread-Safe Atomic Isolation:** Programmed custom low-overhead assembly atomic block macros to control the CPU status register (`SREG`), isolating memory vectors from data corruption race conditions.
* **32kHz Silent Phase-Correct PWM:** Tuned 16-bit Timer1 hardware counters to a 32kHz carrier frequency to achieve silent, fluid motor acceleration tracking curves.
* **Native UART Pipeline:** Wrote custom baud register polling drivers to stream telemetry packets bypassing high-level serial wrapping utilities.
