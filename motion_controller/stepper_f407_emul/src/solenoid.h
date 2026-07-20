/*
 * solenoid — 3 double-solenoid pneumatic valves (SMC SY3220-5LOU, 5/2 bistable).
 * Each valve has two coils (A/B); PULSE a coil to shift, the valve holds its
 * position with both coils de-energized. We never energize both coils, and the
 * pulse auto-releases via a delayed work item.
 */
#ifndef INFEED_SOLENOID_H_
#define INFEED_SOLENOID_H_

enum solenoid_id  { SOLENOID1, SOLENOID2, SOLENOID3, SOLENOID_COUNT };
enum solenoid_pos { SOLENOID_A, SOLENOID_B };

int  solenoid_init(void);
/* Pulse the chosen coil to shift the valve (it then holds with coils off). */
int  solenoid_set(enum solenoid_id s, enum solenoid_pos pos);
/* De-energize all coils (valves HOLD last position) — safe state on fault. */
void solenoid_all_off(void);

#endif /* INFEED_SOLENOID_H_ */
