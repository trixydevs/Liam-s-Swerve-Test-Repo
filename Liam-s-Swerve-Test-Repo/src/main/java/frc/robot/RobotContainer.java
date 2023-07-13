// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.math.MathUtil;

import java.sql.Driver;
import java.util.HashMap;

import javax.naming.AuthenticationException;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private DrivetrainSubsystem drivetrain;
  private Drive defaultDrive;
  private SendableChooser<Command> autoChooser;
  private final PS4Controller driveController;

  private Autos autos;

  public static HashMap<String, Command> eventMap;

  // The robot's subsystems and commands are defined here...
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    driveController = new PS4Controller(0);
    eventMap  = new HashMap<>();
    autoChooser = new SendableChooser<>();

    drivetrain = new DrivetrainSubsystem();

    defaultDrive = new Drive(
      drivetrain,
      ()-> driveController.getL1Button(),
      ()-> modifyAxis(driveController.getRawAxis(Constants.PS4Driver.Y_AXIS), Constants.PS4Driver.DEADBAND_NORMAL),
      ()-> modifyAxis(driveController.getRawAxis(Constants.PS4Driver.X_AXIS), Constants.PS4Driver.DEADBAND_NORMAL),
      ()-> modifyAxis(driveController.getRawAxis(Constants.PS4Driver.Z_AXIS), Constants.PS4Driver.DEADBAND_NORMAL));
    
    drivetrain.setDefaultCommand(defaultDrive);
    SmartDashboard.putBoolean("Use limelight", false);
    SmartDashboard.putBoolean("trust limelight", false);

    autoInit();


  }

  public void autoInit()
  {
    autos = Autos.getInstance();
    eventMap.put("marker1", new PrintCommand("passed marker 1"));
    eventMap.put("marker2", new PrintCommand("passed marker 2"));
    eventMap.put("stop", new InstantCommand(drivetrain::stop, drivetrain));

    autos.autoInit(autoChooser, eventMap, drivetrain);
    SmartDashboard.putData(autoChooser);
  }



  private void configureBindings() {
    // new Trigger(driveController::getPSButton).onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));
   
  }


  private double modifyAxis(double value, double deadband)
  {

    value = MathUtil.applyDeadband(value, deadband);

    value = Math.copySign(value * value, value);
    return value;
  }


   public void robotInit()
   {
    drivetrain.zeroGyroscope();
   }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
