// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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

  //define constants used by system
  public static final double kThrottleGearRatio = 6.12; //not for sure
  public static final double kWheelDiameter = 4*0.0254; //4in wheel in meter
  public static final double kThrottleMaxSpeed = 5676 / 60 /  kThrottleGearRatio * kWheelDiameter*Math.PI; //5676 unit: rpm, output max speed in m/s
  private static final double number = 20.5*0.0254/2;

  //can id definition
  public static final int A_T_ID = 3;
  public static final int A_R_ID = 4;
  public static final int A_E_ID = 9;
  public static final double A_Offset = 0;
  public static final int B_T_ID = 60;
  public static final int B_R_ID = 7;
  public static final int B_E_ID = 3;
  public static final double B_Offset = 0;
  public static final int C_T_ID = 1;
  public static final int C_R_ID = 12;
  public static final int C_E_ID = 6;
  public static final double C_Offset = 0;
  public static final int D_T_ID = 9;
  public static final int D_R_ID = 11;
  public static final int D_E_ID = 12;
  public static final double D_Offset = 0;

  //motor place definition
  public static final Translation2d A_TRANSLATION2D = new Translation2d(number,number);
  public static final Translation2d B_TRANSLATION2D = new Translation2d(number,-number);
  public static final Translation2d C_TRANSLATION2D = new Translation2d(-number,-number);
  public static final Translation2d D_TRANSLATION2D = new Translation2d(-number,number);

  //rotation max speed(in fact i don't know why i wrote this comment)
  public static double kRotationMaxSpeed = kThrottleMaxSpeed/Math.sqrt(2*Math.pow(0.5, 2))*2;
}
