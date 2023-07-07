// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.Key;

import com.ctre.phoenix.sensors.Pigeon2;
import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */

  private SwerveDriveKinematics kinematics = Constants.kinematics;
  private final Pigeon2 pigeon2 = new Pigeon2(Constants.DRIVETRAIN_PIGEON_ID, Constants.CANIVORE_DRIVETRAIN);

  private final SwerveModule[] modules;

  private final Rotation2d[] lastAngles;

  private final SwerveDrivePoseEstimator odometer;

  private final Field2d m_sField2d = new Field2d(); 


  public DrivetrainSubsystem() {

    Preferences.initString("FL","AUGIE");
    Preferences.initString("FR","AUGIE");
    Preferences.initString("BL","AUGIE");
    Preferences.initString("BR","AUGIE");

    SmartDashboard.putData("Field", m_sField2d);

    modules = new SwerveModule[4];
    lastAngles = new Rotation2d[]{new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()};


    modules[0] = new SwerveModule(
    "FL",
    Constants.FL_DRIVE_MOTOR_ID,
    Constants.FL_STEER_MOTOR_ID,
    Constants.FL_STEER_ENCODER_ID,
    Constants.FL_STEER_OFFSET_DEGREES,
    Constants.CANIVORE_DRIVETRAIN
    );

    modules[1] = new SwerveModule(
    "FR",
    Constants.FR_DRIVE_MOTOR_ID,
    Constants.FR_STEER_MOTOR_ID,
    Constants.FR_STEER_ENCODER_ID,
    Constants.FR_STEER_OFFSET_DEGREES,
    Constants.CANIVORE_DRIVETRAIN
    );

    modules[2] = new SwerveModule(
    "BR",
    Constants.BL_DRIVE_MOTOR_ID,
    Constants.BL_STEER_MOTOR_ID,
    Constants.BL_STEER_ENCODER_ID,
    Constants.BL_STEER_OFFSET_DEGREES,
    Constants.CANIVORE_DRIVETRAIN
    );

    modules[3] = new SwerveModule(
    "FL",
    Constants.BR_DRIVE_MOTOR_ID,
    Constants.BR_STEER_MOTOR_ID,
    Constants.BR_STEER_ENCODER_ID,
    Constants.BR_STEER_OFFSET_DEGREES,
    Constants.CANIVORE_DRIVETRAIN
    );

    odometer = new SwerveDrivePoseEstimator(kinematics, null, null, null);

  }

  public void zeroGyroscope(){
    pigeon2.setYaw(0.0);
    //odometer.resetPosition(new Rotation2d(), , null);
  }

  public void zeroGyroscope(double angleDeg){

  }

  public Rotation2d getGyroscopeRotation() {

  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (int i = 0; i <4 ; i++) positions [i] = modules[i].getPosition();
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] States = new SwerveModuleState[4];

    for (int i = 0; i <4 ; i++) States [i] = modules[i].getState();
    return States;
  }

  public Pose2d getPose() 
  {
    return odometer.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose)
  {
    zeroGyroscope(pose.getRotation().getDegrees());
    odometer.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }

  public void resetOdometry()
  {
      odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), Constants.DRIVE_ODOMETRY_ORIGIN);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
