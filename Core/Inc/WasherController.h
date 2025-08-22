/*
 * WasherController.h
 *
 *  Created on: Aug 21, 2025
 *      Author: alexl
 */

#ifndef INC_WASHERCONTROLLER_H_
#define INC_WASHERCONTROLLER_H_


#pragma once
#include <cstdint>
#include <string>

#include "motion_interface.h"
#include "notifier_interface.h"
#include "ArmButton.h"

enum class WasherState { Idle, Armed, Monitering, Done, Error };

struct WasherParams {
	uint32_t still_timeout_ms = 120000; // how long of “still” => done (e.g., 2 min)
	uint32_t start_motion_grace_ms = 30000; // time window to see motion after arming
};

class WasherController{
public:
	WasherController(ArmButton& button, IMotion& motion, INotifier& notif, const WasherParams& p);

	void tick(uint32_t now_ms);

	WasherState state() const { return state_; }

private: //methods
	void uartPrint(const char* s);

	// helper to get state name
	void setState(WasherState new_state, uint32_t now_ms);
private: //vars

	//current state
	ArmButton& button_;
	IMotion& motion_;
	INotifier& notif_;
	WasherParams cfg_;

	WasherState state_{WasherState::Idle};
	uint32_t state_since_ms_{0};

};


#endif /* INC_WASHERCONTROLLER_H_ */
