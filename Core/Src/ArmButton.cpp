/*
 * ArmButton.cpp
 *
 *  Created on: Aug 20, 2025
 *      Author: alexl
 */

#include "ArmButton.hpp"
#include "../Inc/main.hpp"

namespace{
inline bool readButtonPinActiveHigh() {
	// logical pressed returns true
  return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET;
}
} // namespace


ArmButton::ArmButton(uint32_t debounce_ms, uint32_t longpress_ms)
: debounce_ms_(debounce_ms), longpress_ms_(longpress_ms) {
	last_raw_ = readButtonPinActiveHigh();
	pressed_ = last_raw_;
	if (pressed_){
		press_start_ms_ = HAL_GetTick();
		longpress_fired_ = false;
	}
}

void ArmButton::onIsrEdge() {
  isr_flag_ = true;
}

void ArmButton::poll(uint32_t now_ms) {
  // Even without a new edge, a long press may mature.
  if (!isr_flag_) {
    if (pressed_ && !longpress_fired_ &&
        (now_ms - press_start_ms_) >= longpress_ms_) {
      longpress_fired_ = true;
      event_pending_ = true;  // Fire as soon as threshold is reached.
    }
    return;
  }

  // Handle the debounced edge(s).
  isr_flag_ = false;

  const bool raw = readButtonPinActiveHigh();

  // Basic debounce on raw transitions.
  if (raw != last_raw_) {
    if ((now_ms - last_change_ms_) >= debounce_ms_) {
      last_change_ms_ = now_ms;
      last_raw_ = raw;

      if (raw && !pressed_) {
        // Debounced press began.
        pressed_ = true;
        press_start_ms_ = now_ms;
        longpress_fired_ = false;
      } else if (!raw && pressed_) {
        // Debounced release.
        pressed_ = false;
        // If released before longpress_ms_, no event.
      }
    }
  }

  // While pressed, check if we've crossed the long-press threshold.
  if (pressed_ && !longpress_fired_ &&
      (now_ms - press_start_ms_) >= longpress_ms_) {
    longpress_fired_ = true;
    event_pending_ = true;
  }
}

ButtonEvent ArmButton::consumeEvent() {
  ButtonEvent ev;
  if (event_pending_) {
    ev.long_press = true;
    event_pending_ = false;
  }
  return ev;
}


