// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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

  public static final Double DRIVETRAIN_ROTATIONS_TO_METERS = null;
  public static final double MAX_VELOCITY_RPS_EMPIRICAL = 0;
  public static final int DRIVETRAIN_PIGEON_ID = 0;
public static final String CANIVORE_DRIVETRAIN = null;
public static final int FL_DRIVE_MOTOR_ID = 0;
public static final int FL_STEER_MOTOR_ID = 0;
public static final int FL_STEER_ENCODER_ID = 0;
public static final Rotation2d FL_STEER_OFFSET_DEGREES = null;
public static final int FR_DRIVE_MOTOR_ID = 0;
public static final int FR_STEER_MOTOR_ID = 0;
public static final int FR_STEER_ENCODER_ID = 0;
public static final Rotation2d FR_STEER_OFFSET_DEGREES = null;
public static final int BL_STEER_MOTOR_ID = 0;
public static final int BL_DRIVE_MOTOR_ID = 0;
public static final int BL_STEER_ENCODER_ID = 0;
public static final Rotation2d BL_STEER_OFFSET_DEGREES = null;
public static final int BR_DRIVE_MOTOR_ID = 0;
public static final int BR_STEER_MOTOR_ID = 0;
public static final int BR_STEER_ENCODER_ID = 0;
public static final Rotation2d BR_STEER_OFFSET_DEGREES = null;
public static final Pose2d DRIVE_ODOMETRY_ORIGIN = new Pose2d();
public static SwerveDriveKinematics kinematics;
}
