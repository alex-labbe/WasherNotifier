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

enum class WasherState { Idle, Armed, Done, Error };

struct WasherParams {
	uint32_t still_timeout_ms = 120000; // how long of “still” => done (e.g., 2 min)
	uint32_t start_motion_timeout_ms = 600000;  // e.g., 10 min to see first motion after arming
	uint32_t min_run_time_ms = 600000;          // e.g., 10 min minimum run time before "Done" allowed// time window to not really see motion after arming not actualluy sure how usefil this is
};

class WasherController{
public:
	WasherController(ArmButton& button, IMotion& motion, INotifier& notif, const WasherParams& p);

	void tick(uint32_t now_ms);

	WasherState state() const { return state_; }

private: //methods
	void uartPrint(const char* s);

	void finishCycle();

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
	uint32_t last_motion_ms_{0};

	bool seen_motion_{false};

};


#endif /* INC_WASHERCONTROLLER_H_ */
