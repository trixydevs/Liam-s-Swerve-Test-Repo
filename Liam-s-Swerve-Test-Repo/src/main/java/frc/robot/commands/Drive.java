// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Drive extends CommandBase {
  

  private final DrivetrainSubsystem drivetrain;
  private final BooleanSupplier robotCentricMode;
  private final DoubleSupplier translationalXsupplier;
  private final DoubleSupplier translationalYSupplier;
  private final DoubleSupplier rotationSupplier;

 
  public Drive(DrivetrainSubsystem subsystem,
    BooleanSupplier robotCentricMode,
    DoubleSupplier translationalXSupplier,
    DoubleSupplier translationalYSupplier,
    DoubleSupplier rotationSupplier) {

        this.drivetrain = subsystem;
        this.robotCentricMode = robotCentricMode;
        this.translationalXsupplier = translationalXSupplier;
        this.translationalYSupplier = translationalYSupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(robotCentricMode.getAsBoolean())
    {
        drivetrain.drive(new ChassisSpeeds(
        
            translationalXsupplier.getAsDouble() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            translationalYSupplier.getAsDouble() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            rotationSupplier.getAsDouble() * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND

        ));
      
    }
    else
    {
        drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationalXsupplier.getAsDouble() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                translationalYSupplier.getAsDouble() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                rotationSupplier.getAsDouble() * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                drivetrain.getGyroscopeRotation()
                )

        );
    }    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
