#ifndef __POSITIONTRACKER__
#define __POSITIONTRACKER__

#include <Pixy2SPI_SS.h>
#include "Point.h"
#include "Timer.h"

// tracks a single object saved to the red light (sig 1)
#define DEFAULT_TRACKING_SIG 1
#define PIXY_CAMERA_W_PIX 316
#define PIXY_CAMERA_H_PIX 208

#define OBJECT_SIDE_LENGTH 35 //mm
#define FOCAL_LENGTH 282.66 // px

struct TrackedObjectDynamics {
	Point position;
	Point velocity;
	bool detection_occurred;

	TrackedObjectDynamics() {
		detection_occurred = false;
	}
};

struct PixyObject { // 208 x 316
	float x;
	float y;
	int area;
	int age;

	PixyObject() {}
};

class ObjectPositionTracker {
	Pixy2SPI_SS *pixy;
	TrackedObjectDynamics tracked_object;
	PixyObject tracked_pixy_object;
	Timer timer;

public:
	ObjectPositionTracker(Pixy2SPI_SS *npixy) {
		pixy = npixy;
		timer.reset();
	}

	bool objectWasDetected() {
		return tracked_object.detection_occurred;
	}

	TrackedObjectDynamics getObjectDynamics() {
		return tracked_object;
	}

	PixyObject getPixyObject() {
		return tracked_pixy_object;
	}

	void operate() {
		float dt = timer.dt();
		timer.reset();
		pixy->ccc.getBlocks();

		if (pixy->ccc.numBlocks > 0) {
			int object_index = filterSingleBlock();

			if (object_index == -1) {
				tracked_object.detection_occurred = false;
				return;
			}

			tracked_pixy_object = processPixyBlock(object_index);

			TrackedObjectDynamics new_object;
			new_object.position = convertPixyBlockToPoint(object_index);
			if (previouslyDidNotDetectObject()) {
				new_object.velocity = POINT_ZERO; 
			} else {
				new_object.velocity = (new_object.position - tracked_object.position) / dt; 
			}
			new_object.detection_occurred = true;
			tracked_object = new_object;

		} else {
			tracked_object.detection_occurred = false;
		}
	}

private:
	bool previouslyDidNotDetectObject() {
		return !tracked_object.detection_occurred;
	}

	// correct signature
	// 1. correct age
	// 2. closest area
	int filterSingleBlock() {
		int expected_age;
		if (previouslyDidNotDetectObject()) {
			expected_age = 0;
		}
		else {
			expected_age = tracked_pixy_object.age + 1;
			if (expected_age > 255) {
				expected_age = 255;
			}
		}

		int best_index = -1;
		int min_age_difference = 255;
		for (int i = 0; i < pixy->ccc.numBlocks; i++) {
			PixyObject candidate_pixy_object = processPixyBlock(i);
			int age_difference = abs(candidate_pixy_object.age - expected_age);
			if (age_difference < min_age_difference) {
				min_age_difference = age_difference;
				best_index = i;
			}
		}
		return best_index;
	}

	PixyObject processPixyBlock(int block_index) {
		PixyObject processed_block;
		int raw_x = pixy->ccc.blocks[block_index].m_x;
		int raw_y = pixy->ccc.blocks[block_index].m_y;
		int w = pixy->ccc.blocks[block_index].m_width;
		int h = pixy->ccc.blocks[block_index].m_height;

		raw_x -= PIXY_CAMERA_W_PIX/2.0;
		raw_y -= PIXY_CAMERA_H_PIX/2.0;

		if (h >= w) {
			processed_block.area = h * h;
			processed_block.x = raw_x - w/2.0 + h/2.0;
			processed_block.y= raw_y;
		} else {
			processed_block.area = w * w;
			processed_block.x = raw_x;
			processed_block.y= raw_y - h/2.0 + w/2.0;
		}
		processed_block.age = pixy->ccc.blocks[block_index].m_age;

		return processed_block;
	}

	Point convertPixyBlockToPoint(int block_index) {
		PixyObject pixy_object = processPixyBlock(block_index);
		float world_x;
		float world_y;
		float world_z;

		world_z = convertAreaToDepth(pixy_object.area);
		world_x = - pixy_object.x * world_z/FOCAL_LENGTH;
		world_y = - pixy_object.y * world_z/FOCAL_LENGTH;

		return Point(world_x, world_y, world_z);
	}

	float convertAreaToDepth(float area) {
		float meas_side_length = sqrt(area); // px 
		return FOCAL_LENGTH * OBJECT_SIDE_LENGTH / meas_side_length;
	}

	//#ifdef DEBUG
public:
	void printPixyData() {
		Serial.print("x: "); Serial.print(tracked_pixy_object.x); Serial.print(", ");
		Serial.print("y: "); Serial.print(tracked_pixy_object.y); Serial.print(", ");
		Serial.print("Area: "); Serial.print(tracked_pixy_object.area);
		Serial.println();
	}

	void printPosition() {
		Serial.print("Position: "); tracked_object.position.print();
	}
	//#endif
};

#endif

	// // filter based on age or filter based on position
	// int filterSingleBlock(float dt) {
	// 	// extract single block out
	// 	// find the positions of each block
	// 	// the block that is closest to old_block.position + dt * old_block.velocity is the chosen block.
	// 	int closest_block_index;
	// 	int closest_distance = 100000;
	// 	Point expected_position = tracked_object.position + tracked_object.velocity * dt;
	// 	for (int i = 0; i < pixy->ccc.numBlocks; i++) {
	// 		Point candidate_position = convertPixyBlockToPoint(i);

	// 	}
	// }


	// int filterSingleBlock() {
	// 	int expected_age;
	// 	if (previouslyDidNotDetectObject()) {
	// 		expected_age = 0;
	// 	}
	// 	else {
	// 		expected_age = tracked_pixy_object.age + 1;
	// 		if (expected_age > 255) {
	// 			expected_age = 255;
	// 		}
	// 	}
	// 	//int expected_area = tracked_pixy_object.area;

	// 	int best_index = -1;
	// 	int min_age_difference = 255;
	// 	//int min_area_difference;
	// 	for (int i = 0; i < pixy->ccc.numBlocks; i++) {
	// 		PixyObject candidate_pixy_object = processPixyBlock(i);
	// 		int age_difference = abs(candidate_pixy_object.age - expected_age);
	// 		if (age_difference > min_age_difference) {
	// 			min_age_difference = age_difference;
	// 			best_index = i;
	// 		}
	// 		// if (candidate_pixy_object.age == expected_age) {
	// 		// 	if (best_index == -1) {
	// 		// 		best_index = i;
	// 		// 		min_area_difference = abs(candidate_pixy_object.area - expected_area);
	// 		// 	} else {
	// 		// 		int area_difference = abs(candidate_pixy_object.area - expected_area);
	// 		// 		if (area_difference < min_area_difference) {
	// 		// 			best_index = i;
	// 		// 			min_area_difference = area_difference;
	// 		// 		}
	// 		// 	}
	// 		// }
	// 	}
	// 	return best_index;
	// }