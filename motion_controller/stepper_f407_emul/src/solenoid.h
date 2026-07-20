/*
 * solenoid — 3 double-solenoid pneumatic valves (SMC SY3220-5LOU).
 * Each valve has two coils (A/B). LEVEL-driven: solenoid_set() holds the chosen
 * coil energized and de-energizes the opposite; the coil stays on until the
 * next call. We never energize both coils at once.
 */
#ifndef INFEED_SOLENOID_H_
#define INFEED_SOLENOID_H_

enum solenoid_id  { SOLENOID1, SOLENOID2, SOLENOID3, SOLENOID_COUNT };
enum solenoid_pos { SOLENOID_A, SOLENOID_B };

int  solenoid_init(void);
/* Hold the chosen coil energized (level), de-energize the opposite. */
int  solenoid_set(enum solenoid_id s, enum solenoid_pos pos);
/* De-energize all coils — safe state on fault. */
void solenoid_all_off(void);

#endif /* INFEED_SOLENOID_H_ */
