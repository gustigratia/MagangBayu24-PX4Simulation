// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/DifferentialDriveSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__DIFFERENTIAL_DRIVE_SETPOINT__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__DIFFERENTIAL_DRIVE_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/DifferentialDriveSetpoint in the package px4_msgs.
typedef struct px4_msgs__msg__DifferentialDriveSetpoint
{
  uint64_t timestamp;
  float speed;
  float yaw_rate;
} px4_msgs__msg__DifferentialDriveSetpoint;

// Struct for a sequence of px4_msgs__msg__DifferentialDriveSetpoint.
typedef struct px4_msgs__msg__DifferentialDriveSetpoint__Sequence
{
  px4_msgs__msg__DifferentialDriveSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__DifferentialDriveSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__DIFFERENTIAL_DRIVE_SETPOINT__STRUCT_H_
