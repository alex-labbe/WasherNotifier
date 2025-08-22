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
	setState(WasherState::Idle, HAL_GetTick());
}

void WasherController::tick(uint32_t now_ms){
	const auto ev = button_.consumeEvent();
	if (ev.long_press) {
		if(state_ == WasherState::Idle) {
			setState(WasherState::Armed, now_ms);
			seen_motion_ = false;
			last_motion_ms_ = now_ms;
		} else {
			setState(WasherState::Idle, now_ms);
		}
	}


	switch (state_) {
		case WasherState::Idle: {
			//nothing. should probably reset stuff maybe
			break;
		}
		case WasherState::Armed: {
			const bool moving = motion_.isMoving();

			if(moving){
				// looking for first sign of movement
				if (!seen_motion_) seen_motion_ = true;

				// motion pushes out of stillness window
				last_motion_ms_ = now_ms;
			} else {

				if(!seen_motion_) {
					const uint32_t armed_time_elapsed = now_ms - state_since_ms_;
					if(armed_time_elapsed >= cfg_.start_motion_timeout_ms) {
						setState(WasherState::Idle, now_ms);
						break;
					}

				}
				const uint32_t no_motion = now_ms - last_motion_ms_;
				const uint32_t run_time = now_ms - state_since_ms_;

				if (run_time >= cfg_.min_run_time_ms && no_motion >= cfg_.still_timeout_ms) {
					setState(WasherState::Done, now_ms);
					//TODO call finish i suppose
				}
			}
			break;
		}
	}
}

void WasherController::setState(WasherState new_state, uint32_t now_ms){
	state_ = new_state;
	state_since_ms_ = HAL_GetTick();
}




