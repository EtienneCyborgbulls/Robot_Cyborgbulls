#include <cstdio>
// This library provides input/output operations, such as reading and writing to the console or files.

#include <span>
// This library provides a container for a contiguous range of objects with a fixed size, used to represent image data.

#include <sstream>
// This library provides operations for manipulating strings.

#include <string>
// This library provides operations for manipulating strings.

#include <thread>
// This library provides functionality for managing threads of execution

#include <frc/TimedRobot.h>
// This file provides a base class for implementing timed robot programs.

#include "cscore_oo.h"
// This library provides an object-oriented interface for accessing camera streams and images.

#include <networktables/IntegerArrayTopic.h>
// This library provides a topic for sending and receiving integer arrays over the network.

#include <networktables/NetworkTableInstance.h>
// This library provides a class for managing NetworkTables instances.

#include <cameraserver/CameraServer.h>
// This library provides a class for managing camera streams on the robot.

#include <fmt/format.h>
// This library provides a fast and safe alternative to C's printf and scanf functions for formatting and printing strings.

#include "Constants.h"
// This header file likely defines constants and configuration values used throughout the program.

#include "Robot.h"
// This header file likely defines the main class for the robot program.

#include "lib/Deadband.h"
// This header file contain Deadband function for the joystick.

#include <fmt/core.h>
// This library provides a class for measuring time intervals and waiting for a certain amount of time.

#include <frc/Timer.h>
// This library provides a class for measuring time intervals and waiting for a certain amount of time.

#include <frc/shuffleboard/Shuffleboard.h>
// This library provides a class for managing the Shuffleboard dashboard on the driver station.

#include <frc/smartdashboard/SmartDashboard.h>
// This library provides a class for managing the SmartDashboard dashboard on the driver station.

#include <frc/Joystick.h>
// This library provides a class for reading input from a joystick.

#include <frc2/command/CommandScheduler.h>
// This library provides a class for scheduling and executing autonomous and teleoperated commands.

#include <iostream>
// This library provides input/output operations, such as reading and writing to the console or files.

#include <frc/Joystick.h>
// This library provides a class for grouping multiple motor controllers together for easier control.

#include <frc/motorcontrol/MotorControllerGroup.h>
// This library provides a class for controlling Talon motor controllers.

#include <frc/motorcontrol/Talon.h>
// This library provides a class for controlling Talon motor controllers.

#include <frc/controller/PIDController.h>
// This library provides a class for implementing a PID control loop.

#include <frc/XboxController.h>
// This library provides a class for reading input from an Xbox controller.

#include <frc/Compressor.h>
// This library provides a class for controlling a pneumatic compressor.

#include <frc/DoubleSolenoid.h>
// This library provides a class for controlling a double solenoid valve.

#include <frc/Encoder.h>
// This library provides a class for reading encoder data.

#include <rev/SparkMaxAbsoluteEncoder.h>
// This library provides a class for reading data from the absolute encoder on a REV SparkMax motor controller.

#include <units/pressure.h>
// This library provides a class for representing pressure units and performing conversions between them.
#include <frc/DutyCycleEncoder.h>
// This library provides a class for reading duty cycle encoder data.

#include <frc/PneumaticsControlModule.h>
// This library provides a class for controlling a pneumatics control module.

#include <frc/AnalogInput.h>
// This library provides a class for reading analog input data.

#include <frc/Ultrasonic.h>
// This library provides a class for interfacing with ultrasonic sensors.

#include <frc2/command/PIDSubsystem.h>
// This library provides a base class for creating PID-controlled subsystems.

#include <string_view>
// This header file provides the `std::string_view` class for handling string data.

#include "frc/shuffleboard/RecordingController.h"
// This header file provides the `RecordingController` class for recording data to Shuffleboard.

#include "frc/shuffleboard/ShuffleboardEventImportance.h"
// This header file provides the `ShuffleboardEventImportance` enum for specifying the importance of Shuffleboard events.

#include "frc/shuffleboard/ShuffleboardInstance.h"
// This header file provides the `ShuffleboardInstance` class for managing the Shuffleboard GUI.

#include "frc/shuffleboard/ShuffleboardTab.h"
// This header file provides the `ShuffleboardTab` class for managing a single tab in the Shuffleboard GUI.

#include <frc/shuffleboard/Shuffleboard.h>
// This header file provides the `Shuffleboard` class for interfacing with the Shuffleboard GUI.

#include <frc/filter/MedianFilter.h>
// This library provides the `MedianFilter` class for filtering noisy sensor data.