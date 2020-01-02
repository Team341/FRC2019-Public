/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.awt.Point;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  

  public static class Drive {
  /* Drive Motor Ports */

  public static final int LEFT_DRIVE_MOTOR_PORT_A = 8; //8
  public static final int LEFT_DRIVE_MOTOR_PORT_B = 9; //9
  //public static final int LEFT_DRIVE_MOTOR_PORT_C = 10; //10
  public static final int RIGHT_DRIVE_MOTOR_PORT_A = 1; //1
  public static final int RIGHT_DRIVE_MOTOR_PORT_B = 22; //22
  //public static final int RIGHT_DRIVE_MOTOR_PORT_C = 3; //3

  public static final double DRIVE_ALPHA_FILTER_GAIN_LOW_SPEED_FORWARD = .6;
  public static final double DRIVE_ALPHA_FILTER_GAIN_LOW_SPEED_BACKWARD = .6;

  public static final double TURN_DAMPEN_CONSTANT = 0.35;
  
  public static final int LEFT_DRIVE_ENCODER_PORT_A = 0;
  public static final int LEFT_DRIVE_ENCODER_PORT_B = 1;
  public static final int RIGHT_DRIVE_ENCODER_PORT_A = 2;
  public static final int RIGHT_DRIVE_ENCODER_PORT_B = 3;

  public static final double WHEEL_DIAMETER = 6.0; //inches
  public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0; //inches
  public static final double WHEEL_CIRCUMFERENCE = 1.0 * WHEEL_RADIUS * WHEEL_RADIUS * Math.PI;

  public static final double ENCODER_DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / 1023.0;

  public static final double NEO_ENCODER_INCHES_PER_TICK = 139.0 / 78.0;
  
  public static final double DRIVE_CLIMB_SPEED = 0;
  public static final double CLIMB_DISTANCE_TO_DRIVE = 0;
  public static final double AUTO_BACKWARD_SPEED = 1.0;  
  
  public static final int MOTOR_CURRENT_LIMIT = 35;
  public static final double K_COLLISION_THRESHOLD_DELTA_G = 3.0f;
  
  public static final double AUTONOMOUS_DISTANCE_THRESHOLD = 10;
  public static final double AUTONOMOUS_DISTANCE_TO_DRIVE = 100;
  
  public static final double FIND_TARGET_TURN_SPEED = 0.35;
  public static final int MAX_VELOCITY = 10;

  }

  public static class Hab {
    /* Hab Constants */
  
    public static final int PTO_PISTON_PORT = 4; //4;
    public static final int ULTRASONIC_PORT = 9;
    public static final int ULTRASONIC_ECHO_PORT = 0;
    public static final int ULTRASONIC_TRIGGER_PORT = 0;
	  public static final double EXTEND_RACKS_DRIVE_SPEED = 0.2;
	  public static final double RETRACT_RACKS_DRIVE_SPEED = -0.4;
    public static final double DRIVE_ROTATIONS_TO_RACK_HEIGHT = 5.0; //0.06630;//0.75 / (1.2);// * Drive.WHEEL_CIRCUMFERENCE);
    public static final double RACK_STARTING_HEIGHT = -0.5;
    public static final double PLATFORM_2_HEIGHT = 8.0; //inches
    public static final double PLATFORM_3_HEIGHT = 23.0;  // inches
    public static final double PLATFORM_CLIMB_TOLERANCE = 1.5; // inches
    public static final double OVER_HAB_RANGE_THRESHOLD = 6.0; // inches

    public static final double MEASURED_VOLTAGE_TO_CM = 5.0 / (5.0/1024.0);
    public static final int DISTANCE_SENSOR_I2C_PORT = 0x52;
	  public static final int DISTANCE_BYTE_COUNT = 8;
	
  }

  public static class Elevator {
  /* Elevator Constants */
  public static final int ELEVATOR_MOTOR_PORT1 = 7; ///6
  public static final int ELEVATOR_MOTOR_PORT2 = 6; //7
  public static final int ELEVATOR_UPPER_LIMIT_SWITCH_PORT = 8;
  public static final int ELEVATOR_LOWER_LIMIT_SWITCH_PORT = 7;

  public static final double ELEVATOR_SPROCKET_DIAMETER = 1.75;
  public static final double ELEVATOR_SPROCKET_CIRCUMFERENCE = Math.PI * ELEVATOR_SPROCKET_DIAMETER;
  public static final int ELEVATOR_ENCODER_TICKS_PER_ROTATION = 1023;
  public static final double ELEVATOR_GEAR_RATIO = 20.0 / 46.0;
  public static final double ELEVATOR_ENCODER_TICKS_TO_HEIGHT_INCHES =
          ELEVATOR_SPROCKET_CIRCUMFERENCE * ELEVATOR_GEAR_RATIO
          / ELEVATOR_ENCODER_TICKS_PER_ROTATION;

  public static double ELEVATOR_MANUAL_CONTROL_SCALAR = 0.6;

  // /* Main Constants
  public static final int ELEVATOR_HATCH_SCOOP_LOAD = 2150; //5000;
  public static final int ELEVATOR_HATCH_POSITION_LOW = 2850; //3175;
  public static final int ELEVATOR_HATCH_POSITION_MIDDLE = 13200; //32500;
  public static final int ELEVATOR_HATCH_POSITION_TOP = 21500; //52200;

  public static final int ELEVATOR_POSITION_BOTTOM = 0;

  public static final int ELEVATOR_CARGO_POSITION_BOTTOM = 3000; //6850;
  public static final int ELEVATOR_CARGO_POSITION_HOLD = 8570; //8000;
  public static final int ELEVATOR_CARGO_POSITION_MIDDLE = 12975; //30500;
  public static final int ELEVATOR_CARGO_POSITION_TOP = 21500; //52200;

  public static final int ELEVATOR_UPPER_LIMIT_TICKS = 22000; //52200;
  public static final double ELEVATOR_POSITION_TOLERANCE = 100; //1500;
  // */
  public static final int MOTION_ACCELERATION_MAX = 10;
  
  public static final double VOLTAGE_LIMIT = 1.5;
  public static final double VOLTAGE_COUNT_LIMIT = 100;

  /* Twin Constants
  public static final int ELEVATOR_HATCH_SCOOP_LOAD = 5000;
  public static final int ELEVATOR_HATCH_POSITION_LOW = 10350;
  public static final int ELEVATOR_HATCH_POSITION_MIDDLE = 32500;
  public static final int ELEVATOR_HATCH_POSITION_TOP = 52200;

  public static final int ELEVATOR_POSITION_BOTTOM = 0;

  public static final int ELEVATOR_CARGO_POSITION_BOTTOM = 6850;
  public static final double ELEVATOR_CARGO_POSITION_HOLD = 8000;
  public static final int ELEVATOR_CARGO_POSITION_MIDDLE = 30500;
  public static final int ELEVATOR_CARGO_POSITION_TOP = 52200;

  public static final int ELEVATOR_UPPER_LIMIT_TICKS = 52200;
  public static final double ELEVATOR_POSITION_TOLERANCE = 1500;
  */
  }

  public static class CargoIntake {
    /* Cargo Intake Constants */

  public static final int HINGE_MOTOR_LEFT_PORT = 33; //10
  public static final int HINGE_MOTOR_RIGHT_PORT = 3; //3
  public static final int INTAKE_MOTOR_PORT = 5; //5
  public static final int CONVEYOR_MOTOR_PORT = 12; //12
  public static final int HINGE_UPPER_LIMIT_SWITCH_PORT = 6;
  public static final int HINGE_LOWER_LIMIT_SWITCH_PORT = 5;

  public static final int INTAKE_HEIGHT_LEVEL_2_START = 8;
  public static final int INTAKE_HEIGHT_LEVEL_3_START = 22;

  public static final int HINGE_STOW_POSITION = 0;
  public static final int HINGE_LOAD_POSITION = 65; //86;
  public static final int HINGE_TRANSIT_POSITION = 25; //22
  public static final int HINGE_CLIMB_POSITION = 120;
  
  public static final int POSITION_MARGIN_OF_ERROR = 10;
  public static final double HINGE_TO_FRONT_WHEEL_LENGTH = 20.3572831; // Inches?
  public static final double HINGE_HEIGHT_FROM_GROUND = 8.5;           // in 
  public static final double HINGE_DISTANCE_FROM_BACK = 6;             // in
  public static final double ENCODER_GEAR_RATIO = 1.0 / 240.0;
  public static final double ENCODER_ROTATIONS_TO_RADIANS = (2.0 * Math.PI) / (4096.0 * ENCODER_GEAR_RATIO); // 12 / 44
  public static final double ENCODER_ROTATIONS_TO_DEGREES = (360.0) / (4096.0 * ENCODER_GEAR_RATIO);
  public static final double ENCODER_CONVERSION_FACTOR = 1.29;

  public static final double HINGE_MAX_VELOCITY = (20 * ENCODER_GEAR_RATIO) / (360.0);
  public static final double HINGE_MAX_ACCELERATION = HINGE_MAX_VELOCITY;
  
  public static final double ROLLER_CLIMBING_SPEED = 0;    

  public static final double INTAKING_SPEED = 0.75;
  public static final double SPITTING_SPEED = -1.0;
  public static final double CONVEYOR_IN_SPEED = 0.8;
  public static final double CONVEYOR_OUT_SPEED = -0.8;
  public static final double HINGE_CLIMB_SPEED = 0.4;

  public static final double MANUAL_CONTROL_SCALAR = 0.2;
  
  public static final int K_TIMEOUT_MS = 30;
  public static final int HINGE_CURRENT_LIMIT = 30;
  }

  public static class CargoScore {
    /* Cargo Score Constants */

	public static final int SCORING_MOTOR_PORT = 13; //13
  public static final int BALL_SENSOR_PORT = 23;
  
  public static final double IN_SPEED = 0.8;
  public static final double SCORE_SPEED = 1.0;
  public static final double UNJAM_SPEED = -0.25;

  }

  public static class HatchIntake {
    /* Hatch Intake Constants */

	public static final int INTAKE_MOTOR_PORT = 14; //14
  public static final int LOWERER_PISTON_PORT = 3; //3
  public static final int HATCH_SENSOR_PORT = 22;

  public static final double ROLLER_IN_SPEED = 1.0;
  public static final double ROLLER_OUT_SPEED = -1.0;
  }

  public static class HatchScore {
    /* Hatch Score Constants */

  public static final int PISTON_PORT = 2;
  public static final int SLIDER_PORT_A = 0;
  public static final int SLIDER_PORT_B = 1;
  }

  public static class Controller {
  /* Controller Ports */

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_1_PORT = 1;
  public static final int OPERATOR_CONTROLLER_2_PORT = 2;

  public static final int XBOX_CONTROLLER_LEFT_STICK_X = 0;
  public static final int XBOX_CONTROLLER_LEFT_STICK_Y = 1;
  public static final int XBOX_CONTROLLER_RIGHT_STICK_X = 4;
  public static final int XBOX_CONTROLLER_RIGHT_STICK_Y = 5;
  public static final double DEADBAND = 0.25;

  public static final int OPERATOR_CONTROLLER_LEFT_AXIS = 0;
  public static final int OPERATOR_CONTROLLER_RIGHT_AXIS = 0;

  // Operator Controller 1 Ports UPDATED

  public static final int CARGO_HINGE_LOAD_PORT = 1;
  public static final int CARGO_HINGE_TRANSIT_PORT = 2;
  public static final int CARGO_HINGE_STOW_PORT = 3;
  public static final int HAB_CLIMB_LEVEL_2_PORT = 4;
  public static final int HAB_CLIMB_LEVEL_3_PORT = 5;
  public static final int ELEVATOR_CARGO_LOW_PORT = 6;
  public static final int ELEVATOR_CARGO_HOLD_PORT = 7;
  public static final int ELEVATOR_CARGO_MED_PORT = 8;
  public static final int ELEVATOR_CARGO_HIGH_PORT = 9;

  // Operator Controller 2 Ports UPDATED

  public static final int ELEVATOR_BOTTOM_PORT = 1;
  public static final int ELEVATOR_HATCH_LOW_PORT = 2;
  public static final int ELEVATOR_HATCH_MED_PORT = 3;
  public static final int ELEVATOR_HATCH_HIGH_PORT = 4;
  public static final int CARGO_INTAKE_REVERSE_PORT = 11;
  public static final int CARGO_INTAKE_RUN_PORT = 12; 
  public static final int CARGO_SCORE_RUN_PORT = 7;
  public static final int HATCH_EXPANDER_IN_PORT = 8;
  public static final int HATCH_SCOOP_LOAD_PORT = 9;
  public static final int HATCH_SLIDER_OUT_PORT = 10;
  //public static final int RESET_ALL_PORT = 11; //doesn't exist? :(

  }

  public static class Vision {
    public static final Point SCREEN_CENTRE_POINT = new Point(80, 60);
    public static final double FIELD_OF_VIEW_X = 60.0;
    public static final double FIELD_OF_VIEW_Y = 34;
	  public static final double DISTANCE_OFFSET = 0.5;
    
    public static final double GOAL_SCORE_THRESHOLD = 0.1;
    public static final double GOAL_DISTANCE_THRESHOLD = 6.0;
    public static final double GOAL_ANGLE_THRESHOLD_DEGREES = 25.0;
    public static final double GOAL_DISTANCE_SCALAR_THRESHOLD = 10.0;
    public static final double GOAL_ANGLE_THRESHOLD_RADIANS = Math.toRadians(GOAL_ANGLE_THRESHOLD_DEGREES);
    public static final long TARGET_TRACK_DROP_THRESHOLD = 2500;
    public static final double TARGET_CLOSE_DISTANCE_THRESHOLD = 2.5;

    public static final double GOAL_POSITION_ALPHA = 0.25;
    public static final double TRACK_RANGE_MEDIUM = 3.5;
    public static final double TRACK_RANGE_CLOSE = 1.75;
  }

  public static class LEDS {

    public static final int RED_PORT = 5;
    public static final int GREEN_PORT = 6;
    public static final int BLUE_PORT = 7;

    public static final int NO_COLOR = -1;
    public static final int VISION_BALL = 0;
    public static final int VISION_HATCH = 1;
    public static final int CARGO_HATCH = 2;
	  public static final int ELEVATOR_VOLTAGE = 3;
  }
}