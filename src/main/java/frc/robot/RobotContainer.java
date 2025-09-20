// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final Swerve drivebase = new Swerve(driverXbox);
 

  public RobotContainer() {
    configureBindings();
  } 

  private void configureBindings() {
    // Default comand is to drive Field Oriented with Angular Velocity
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(drivebase.driveAngularVelocity));

    //driverXbox.a().onTrue(drivebase.sysIdDriveMotorCommand());
  }
    

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
