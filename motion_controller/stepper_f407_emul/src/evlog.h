/*
 * evlog — cycle event log on the console. Timestamps are SSSS.S (tenths of a
 * second) relative to the last evlog_cycle_start() (called at step 2, when the
 * 32F asserts SMEMA up-Ready). Owns the cycle clock + the line format.
 */
#ifndef INFEED_EVLOG_H_
#define INFEED_EVLOG_H_

void evlog_cycle_start(unsigned cycle);   /* reset the clock + print header */
void evlog_cycle_end(unsigned cycle);     /* print footer with total time */

/* dir: '>' = 32F output/command, '<' = input received, '.' = note. */
void evlog(int step, char dir, const char *fmt, ...);

#endif /* INFEED_EVLOG_H_ */
