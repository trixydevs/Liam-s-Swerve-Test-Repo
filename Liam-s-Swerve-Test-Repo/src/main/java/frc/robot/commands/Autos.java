// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;


public final class Autos {
  /** Example static factory for an autonomous command. */
  private DrivetrainSubsystem drivetrain;
  public static SwerveAutoBuilder autobuilder;
  private HashMap<String, Command> eventMap;

  private static Autos autos;

  public static Autos getInstance()
  {
    if(autos == null)
    {
      autos = new Autos();
    }
    return autos;
  }

  public void autoInit(SendableChooser<Command> autoChooser, HashMap<String, Command> eventMap, DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    this.eventMap = eventMap;
    // Create the AutoBuilder. This only needs to be created once when robot code
    // starts, not every time you want to create an auto command. A good place to
    // put this is in RobotContainer along with your subsystems.
    Autos.autobuilder = new SwerveAutoBuilder(
            drivetrain::getPose, // Pose2d supplier
            drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            drivetrain.kinematics, // SwerveDriveKinematics
            new PIDConstants(
                    Constants.k_XY_P,
                    Constants.k_XY_I,
                    Constants.k_XY_D), // PID constants to correct for translation error (used to create the X
                                            // and Y PID controllers)
            new PIDConstants(
                    Constants.k_THETA_P,
                    Constants.k_THETA_I,
                    Constants.k_THETA_D), // PID constants to correct for rotation error (used to create the
                                               // rotation controller)
            drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color.
                   // Optional, defaults to true
            drivetrain // The drive subsystem. Used to properly set the requirements of path following
                       // commands
    );
    // add autos to smart dashboard.\
    autoChooser.addOption("example auto", ExampleAuto());

}


  public SequentialCommandGroup ExampleAuto()
  {
    return new SequentialCommandGroup(

    autobuilder.fullAuto(driveforward)
    );
  }



  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

 static List<PathPlannerTrajectory> driveforward = PathPlanner.loadPathGroup("My First Path", new PathConstraints(2.5, 1.75));
}
