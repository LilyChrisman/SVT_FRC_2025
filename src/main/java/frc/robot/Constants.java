// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(16);
  public static final double SLOWED_SPEED = Units.feetToMeters(4.0);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  /**
   * AprilTag field layout.
   */
  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  
  // Returns true if the value is inside the list
  public static boolean contains(double[] array, double value) {
    for (double element : array) {
        if (element == value) {
            return true;
        }
    }
    return false;
  }

  public static final double inToM = 0.0254;
  
  // Holds all the positions of the April Tags
  public class AprilTagMaps {
      // Field Map Source: https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
      // A HashMap of April Tag positions: Key = ID, Value = [X, Y, Z, Yaw, Pitch]
      public static final HashMap<Integer, double[]> aprilTagMap = new HashMap<>();
      static {
          // Points are in inches, Angles are in degrees
          // RED SIDE
          aprilTagMap.put(1, new double[]{657.37, 25.80, 58.50, 126.0, 0.0});
          aprilTagMap.put(2, new double[]{657.37, 291.20, 58.50, 234.0, 0.0});
          aprilTagMap.put(3, new double[]{455.15, 317.15, 51.25, 270.0, 0.0});
          aprilTagMap.put(4, new double[]{365.20, 241.64, 73.54, 0.0, 30.0});
          aprilTagMap.put(5, new double[]{365.20, 75.39, 73.54, 0.0, 30.0});
          aprilTagMap.put(6, new double[]{530.49, 130.17, 12.13, 300.0, 0.0});
          aprilTagMap.put(7, new double[]{546.87, 158.50, 12.13, 0.0, 0.0});
          aprilTagMap.put(8, new double[]{530.49, 186.83, 12.13, 60.0, 0.0});
          aprilTagMap.put(9, new double[]{497.77, 186.83, 12.13, 120.0, 0.0});
          aprilTagMap.put(10, new double[]{481.39, 158.50, 12.13, 180.0, 0.0});
          aprilTagMap.put(11, new double[]{497.77, 130.17, 12.13, 240.0, 0.0});
          // BLUE SIDE
          aprilTagMap.put(12, new double[]{33.51, 25.80, 58.50, 54.0, 0.0});
          aprilTagMap.put(13, new double[]{33.51, 291.20, 58.50, 306.0, 0.0});
          aprilTagMap.put(14, new double[]{325.68, 241.64, 73.54, 180.0, 30.0});
          aprilTagMap.put(15, new double[]{325.68, 75.39, 73.54, 180.0, 30.0});
          aprilTagMap.put(16, new double[]{235.73, -0.15, 51.25, 90.0, 0.0});
          // Tag 17: (4.073906m, 3.306318m, 0.308102m, 4PI/3 rad, 0 rad)
          aprilTagMap.put(17, new double[]{160.39, 130.17, 12.13, 240.0, 0.0});
          // Tag 18: (3.6576m, 4.0259m, 0.308102m, PI rad, 0 rad)
          aprilTagMap.put(18, new double[]{144.00, 158.50, 12.13, 180.0, 0.0});
          aprilTagMap.put(19, new double[]{160.39, 186.83, 12.13, 120.0, 0.0});
          aprilTagMap.put(20, new double[]{193.10, 186.83, 12.13, 60.0, 0.0});
          aprilTagMap.put(21, new double[]{209.49, 158.50, 12.13, 0.0, 0.0});
          aprilTagMap.put(22, new double[]{193.10, 130.17, 12.13, 300.0, 0.0});
      }
  }

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
