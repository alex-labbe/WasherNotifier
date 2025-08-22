/*
 * notifier_interface.h
 *
 *  Created on: Aug 21, 2025
 *      Author: alexl
 */

#ifndef INC_NOTIFIER_INTERFACE_H_
#define INC_NOTIFIER_INTERFACE_H_

#pragma once
#include <cstdint>


class INotifier { //TODO: implement the huart version of notifying. no light - unarmed, blinking, armed/running, done - solid
public:
	virtual ~INotifier() = default;

	virtual void notify(const char* s) = 0;

	virtual bool notifyHigh(const char* s) = 0;
};



#endif /* INC_NOTIFIER_INTERFACE_H_ */
