/*
 * process — the tester operation sequence (see ~/moto/tester_emul_logic.md).
 * Power-up homing of motor 3/4, then the 21-step per-panel cycle looping on the
 * SMEMA handshake. Runs in its own thread; composes motion / solenoid / io and
 * narrates each step via evlog.
 */
#ifndef INFEED_PROCESS_H_
#define INFEED_PROCESS_H_

#include <stdbool.h>

int process_init(void);   /* starts the sequence thread */
void process_start(void); /* release the unconfigured safe-hold (CLI `infeed start`) */

/* Commissioning/test mode: cylinder confirms wait indefinitely (no alarm) so an
 * operator can hand-step the cycle (L4 or sim) without being rushed. Runtime
 * only, defaults OFF (production = 10 s cylinder alarm). */
void process_set_test_mode(bool on);
bool process_test_mode(void);

#endif /* INFEED_PROCESS_H_ */
