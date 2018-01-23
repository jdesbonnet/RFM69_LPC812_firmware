/*
 * sleep.h
 *
 *  Created on: 1 Jul 2013
 *      Author: joe
 */

#ifndef SLEEP_H_
#define SLEEP_H_

void sleep_set_pins_for_powerdown(void);
void sleep_set_pins_for_wake (void);
void sleep_prepare (void);
void prepareForPowerDown2 (void); // works
void prepareForDeepPowerDown (void);


#endif /* SLEEP_H_ */
