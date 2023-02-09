// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //CAN ID
  public static final int EXTENDER_MOTOR_ID = 12;


  //motor speed
  public static final double EXTENDER_SPEED = 0.5;
  

  //AIO 
  
  /*
   * public static final int EXTENDER_POTENTIOMETER = 1; 
   * public static final int EXTENDER_RANGE_OF_MOTION = 100;
   * public static final int EXTENDER_STARTING_POINT = 0; 
   * public static final double HOOD_ERROR_THRESHOLD = 1;
   */

  //TIME OUT
  public static final int EXTENDER_MOTOR_TIMEOUT = 2; 
  public static final double EXTENDER_TARGET_TIMEOUT = 1;
  public static final int EXTENDER_POSITION_ERROR = 1; 

  public static final double EXTENDER_P = 1;
  public static final double EXTENDER_I = 0.01;
  public static final double EXTENDER_D = 0;
  public static final double EXTENDER_F = 0;

}
