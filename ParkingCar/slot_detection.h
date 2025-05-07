#ifndef SLOT_DETECTION_H
#define SLOT_DETECTION_H

#include <stdbool.h>
#include <stdint.h>

#define CAR_WIDTH_CM 16
#define CAR_LENGTH_CM 30
#define MIN_SLOT_MARGIN 5  // Extra space needed
#define CAR_SPEED_CMS 15   // Car speed in cm/s

typedef struct {
  float start_position;
  float length;
  float width;
  bool is_valid;
} ParkingSlot;

void init_slot_detection(void);
void start_slot_detection(void);
bool detect_parking_slot(ParkingSlot* slot);
bool validate_parking_dimensions(ParkingSlot* slot);

#endif