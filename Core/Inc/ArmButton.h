/*
 * ArmButton.hpp
 *
 *  Created on: Aug 20, 2025
 *      Author: alexl
 */

#ifndef INC_ARMBUTTON_HPP_
#define INC_ARMBUTTON_HPP_

#pragma once
#include <cstdint>

struct ButtonEvent{
	bool long_press{false};
};

class ArmButton{
public:
	explicit ArmButton(uint32_t debounce_ms, uint32_t longpress_ms);
	void onIsrEdge();
	void poll(uint32_t now_ms);
	ButtonEvent consumeEvent();
	bool isPressed() const { return pressed_; }
private:
	// ISR <-> main loop handshake
	volatile bool isr_flag_{false};

	bool last_raw_{false};
	bool pressed_{false};
	bool longpress_fired_{false};
	bool event_pending_{false};

	uint32_t press_start_ms_{0};
	uint32_t last_change_ms_{0};

	// for tuning
	uint32_t debounce_ms_;
	uint32_t longpress_ms_;
};


#endif /* INC_ARMBUTTON_HPP_ */
