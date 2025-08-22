/*
 * Fake_Motion.h
 *
 *  Created on: Aug 21, 2025
 *      Author: alexl
 */

#ifndef INC_FAKE_MOTION_H_
#define INC_FAKE_MOTION_H_

#pragma once
#include "motion_interface.h"

// A minimal fake you can drive from main() or tests.
// - Call setMoving(true/false) to simulate washer motion.
// - activityScore() returns a value you choose (for logs/threshold tuning).
class FakeMotion : public IMotion {
public:
	explicit FakeMotion(bool moving = false, float activity = 0.0f)
	  : moving_(moving), activity_(activity) {}

	bool begin() override { return true; }

	void update(uint32_t /*now_ms*/) override {
	// No-op: state is driven externally via setMoving().
	}

	bool isMoving() const override { return moving_; }



// Test helpers:
	void setMoving(bool moving) { moving_ = moving; }
	void setActivity(float a)   { activity_ = a; }

private:
	bool  moving_{false};
	float activity_{0.0f};
};


#endif /* INC_FAKE_MOTION_H_ */
