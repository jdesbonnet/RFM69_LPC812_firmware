/*
 * sleep.h
 *
 *  Created on: 1 Jul 2013
 *      Author: joe
 */

#ifndef SLEEP_H_
#define SLEEP_H_

void sleep_condition_for_powerdown(void);
void sleep_condition_after_wake (void);
void prepareForPowerDown (void);
void prepareForPowerDown2 (void); // works
void prepareForDeepPowerDown (void);


#endif /* SLEEP_H_ */
