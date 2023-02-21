// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean DEBUG = true;

  //JOYSTICKS
  public static final int LEFT_JOYSICK_ID = 0;
  public static final int RIGHT_JOYSTICK_ID = 1;

  //S = STEER, D = DRIVE, Drivetrain ConstantsS
  public static final int DRIVE_FRONT_RIGHT_S = 48;
  public static final int DRIVE_FRONT_RIGHT_D = 49;
  public static final int DRIVE_BACK_RIGHT_S = 46;
  public static final int DRIVE_BACK_RIGHT_D = 43;

  public static final int DRIVE_FRONT_LEFT_S = 50;
  public static final int DRIVE_FRONT_LEFT_D = 51;
  public static final int DRIVE_BACK_LEFT_S = 31;
  public static final int DRIVE_BACK_LEFT_D = 34;

  public static final int DRIVE_CANCODER_FRONT_RIGHT = 60;
  public static final int DRIVE_CANCODER_BACK_RIGHT = 57;
  public static final int DRIVE_CANCODER_FRONT_LEFT = 56;
  public static final int DRIVE_CANCODER_BACK_LEFT = 58;
  
  //GRIPPER
  public static final int GRIPPER_MOTOR_ID = 8;
  public static final int GRIPPER_ENCODER_ID = 0;
  public static final int ARM_ID = 35;

  //PID Constants
  public static final double DRIVE_PID_P = 1;
  public static final double DRIVE_PID_I = 0;
  public static final double DRIVE_PID_D = 0;
  public static final double DRIVE_PID_FF_S = 1;
  public static final double DRIVE_PID_FF_V = 2.8;

  public static final double STEER_PID_P = 0.3;
  public static final double STEER_PID_I = 0;
  public static final double STEER_PID_D = 0;
  public static final double STEER_PID_FF_S = 0;//0.2;
  public static final double STEER_PID_FF_V = 0;//0.8;

  public static final double ARM_PID_P = 0.1;
  public static final double ARM_PID_I = 0;
  public static final double ARM_PID_D = 0;
  public static final double ARM_PID_FF = 0;

  public static final double WHEEL_RADIUS = 0.0508;
  public static final int ENCODER_RESOLUTION = 4096;
  public static final double CHASSIS_DRIVE_GEAR_RATIO = 8.142857; // this value should be x:1
  public static final double CHASSIS_STEER_GEAR_RATIO = 12.8; // this value should be x:1

  public static final double MAX_VELOCITY = 3.0; // 3 meters per second
  public static final double MAX_ACCELERATION = 6.0;
  public static final double MAX_ANGULAR_SPEED = Math.PI * 3; // 1/2 rotation per second
  public static final double MAX_ANGULAR_ACCELERATION = Math.PI * 3;

  //autonomous values
  public static final double MAX_VELOCITY_AUTO = 1.5;
  public static final double MAX_ACCELERATION_AUTO = 3.0;
  public static final double MAX_ANGULAR_SPEED_AUTO = Math.PI * 0.5;
  public static final double MAX_ANGULAR_ACCELERATION_AUTO = Math.PI * 0.5;
  public static final double kP_THETA = 0.1;
  public static final double kP_X = 2.6; 
  public static final double kI_X = 0;
  public static final double kD_X = 0;
  public static final double kP_Y = 2.6;
  public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);


  

  public static final double ARM_CONTROLLER_CHANGE = 1;
  public static final double ARM_MAX_ANGLE = 180;

  public static final double ROBOT_WIDTH = 0.5969;
  public static final double ROBOT_LENGTH = 0.5969;
  public static final int IMU = 42;
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double BACK_RIGHT_ABS_ENCODER_ZERO = 218.76;
  public static final double FRONT_LEFT_ABS_ENCODER_ZERO = 9.0;
  public static final double BACK_LEFT_ABS_ENCODER_ZERO = 351.3;
  public static final double FRONT_RIGHT_ABS_ENCODER_ZERO = 299.13;


  public static final double GRIPPER_OPENING_SPEED = 0.5;
  public static final double GRIPPER_CLOSING_SPEED = -0.5;
  public static final double GRIPPER_TIMEOUT = 5.0;
  public static final double WANTED_TIME = 1.0;
}
