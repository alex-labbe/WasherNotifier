/*
 * WasherController.cpp
 *
 *  Created on: Aug 21, 2025
 *      Author: alexl
 */

#include "../Inc/main.h"
#include "../Inc/WasherController.h"

WasherController::WasherController(ArmButton& button, IMotion& motion, INotifier& notif, const WasherParams& params)
	: button_(button), motion_(motion), notif_(notif), cfg_(params) {
	setState(WasherState::Idle);
}

void WasherController::tick(uint32 now_ms){
	const auto ev = button_.consumeEvent();
	if (!ev.long_press) return;

	if(state_ == WasherState::Idle) {
		setState(WasherState::Armed);
	} else {
		setState(WasherState::Idle);
	}
}

void WasherController::setState(WasherState new_state, uint32_t now_ms){
	state_ = new_state;
	state_since_ms_ = HAL_GetTick();
}
