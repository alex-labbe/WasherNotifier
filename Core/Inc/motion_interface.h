/*
 * motion_interface.h
 *
 *  Created on: Aug 21, 2025
 *      Author: alexl
 */

#ifndef INC_MOTION_INTERFACE_H_
#define INC_MOTION_INTERFACE_H_

#pragma once
#include <cstdint>

struct MotionSample {
	float ax{0.0f}, ay{0.0f}, az{0.0f};
	float activity{0.0f};
	bool moving{false};
};

class IMotion {
public:
	virtual ~IMotion() = default;

	virtual bool begin() = 0;

	virtual void update(uint32_t now_ms) = 0;

	virtual bool isMoving() const = 0;
};



#endif /* INC_MOTION_INTERFACE_H_ */
