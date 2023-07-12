// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.Key;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.fasterxml.jackson.annotation.JacksonInject.Value;
import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.Math.kinematics.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  //public static final CTREconfigs ctreConfig = new CTREConfigs;

  public SwerveDriveKinematics kinematics = Constants.kinematics;
  
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
    Constants.FL_STEER_OFFSET,
    Constants.CANIVORE_DRIVETRAIN
    );

    modules[1] = new SwerveModule(
    "FR",
    Constants.FR_DRIVE_MOTOR_ID,
    Constants.FR_STEER_MOTOR_ID,
    Constants.FR_STEER_ENCODER_ID,
    Constants.FR_STEER_OFFSET,
    Constants.CANIVORE_DRIVETRAIN
    );

    modules[2] = new SwerveModule(
    "BR",
    Constants.BL_DRIVE_MOTOR_ID,
    Constants.BL_STEER_MOTOR_ID,
    Constants.BL_STEER_ENCODER_ID,
    Constants.BL_STEER_OFFSET,
    Constants.CANIVORE_DRIVETRAIN
    );

    modules[3] = new SwerveModule(
    "FL",
    Constants.BR_DRIVE_MOTOR_ID,
    Constants.BR_STEER_MOTOR_ID,
    Constants.BR_STEER_ENCODER_ID,
    Constants.BR_STEER_OFFSET,
    Constants.CANIVORE_DRIVETRAIN
    );

    odometer = new SwerveDrivePoseEstimator(kinematics, getGyroscopeRotation(), getModulePositions(), Constants.DRIVE_ODOMETRY_ORIGIN);

  }

  public void zeroGyroscope(){
    pigeon2.setYaw(0.0);
    //odometer.resetPosition(new Rotation2d(), , null);
  }

  public void zeroGyroscope(double angleDeg){

    pigeon2.setYaw(0.0);
    odometer.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public Rotation2d getGyroscopeRotation() {
    return pigeon2.getRotation2d();
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
  
  public void pointWheelsInward()
  {
    
  }

  public void drive( ChassisSpeeds chassisSpeeds)
  {
    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    double maxSpeed = Collections.max(Arrays.asList(desiredStates)).speedMetersPerSecond;
    if (maxSpeed <= Constants.DRIVE_DEADBAND_MPS)
    {
      for(int i = 0; i < 4;  i++)
      {
        stop();
      }
    }else
    {
      setModuleStates(desiredStates);
    }
  
  }

  public void stop()
  {
    for(int i = 0; i < 4; i++)
    {
      modules[i].setDesiredState(new SwerveModuleState(0, lastAngles[i]));
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_VELOCITY_METERS_PER_SECOND);
    for (int i = 0;  i < 4; i++)
    {
      setModule(i, desiredStates[i]);
    }
  }

  private void setModule(int i, SwerveModuleState desiredState)
  {
    modules[i].setDesiredState(desiredState);
    lastAngles[i] = desiredState.angle;
  }

  private void updateOdemetry()
  {
    odometer.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation(), getModulePositions());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdemetry();
    m_sField2d.setRobotPose(odometer.getEstimatedPosition());
    
    SmartDashboard.putNumber("Robot Angle", getGyroscopeRotation().getDegrees());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());


  }
}
