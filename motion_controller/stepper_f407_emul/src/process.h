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
void process_start(void); /* release safe-hold / (re)run + leave manual mode (CLI `infeed start`) */

/* Manual commissioning commands: run one portion of the cycle from the CLI.
 * Each parks the auto-cycle (manual mode) until `infeed start` resumes it.
 * Return 0 on success, -EBUSY (fault), -EAGAIN (auto cycle running), -EINTR
 * (aborted by fault/`infeed start` mid-move). Blocking (wait on sensors/motion). */
int process_table_home(void);  /* #1  home the table (up to Photo 10) */
int process_load(void);        /* #4..#9  panel in (Laser1) -> convey -> near (Laser2) -> creep */
int process_table_down(void);  /* #10  table down to test (6 rev) */
int process_unload(void);      /* #25..#27  convey out -> in position (Laser3) -> stop */

/* Commissioning/test mode: cylinder confirms wait indefinitely (no alarm) so an
 * operator can hand-step the cycle (L4 or sim) without being rushed. Runtime
 * only, defaults OFF (production = 10 s cylinder alarm). */
void process_set_test_mode(bool on);
bool process_test_mode(void);

#endif /* INFEED_PROCESS_H_ */
