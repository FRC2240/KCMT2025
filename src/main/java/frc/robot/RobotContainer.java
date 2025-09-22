// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.utils.ManipulatorState;

public class RobotContainer {
  final CommandXboxController stick0 = new CommandXboxController(0);
  final CommandXboxController stick1 = new CommandXboxController(1);

  private final Swerve drivebase = new Swerve(stick0);
  private final Grabber grabber = new Grabber();
  private final Wrist wrist = new Wrist();
  private final Elevator elevator = new Elevator();

  private ManipulatorState currentState = Constants.ManipulatorStates.IDLE;
 

  public RobotContainer() {
    configureBindings();
  } 

  private void configureBindings() {
    // Default comand is to drive Field Oriented with Angular Velocity
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(drivebase.driveAngularVelocity));

    //driverXbox.a().onTrue(drivebase.sysIdDriveMotorCommand());

  }

  private Command setStateCommand(ManipulatorState target) {
    ManipulatorState lastState = currentState;
    currentState = target;

    Command elevatorCommand = elevator.setPositionCommand(target.elevatorPos);
    Command wristCommand = wrist.setAngleCommand(target.wristPos);

    // Special cases where elevator moves first
    for (ManipulatorState state : Constants.ManipulatorStates.ELEVATOR_FIRST_STATES) {
      if (target.equals(state)) return elevatorCommand.andThen(wristCommand);
    }

    // Special cases where wrist moves first
    for (ManipulatorState state : Constants.ManipulatorStates.WRIST_FIRST_STATES) {
      if (target.equals(state)) return wristCommand.andThen(elevatorCommand);
    }
    if (lastState.equals(Constants.ManipulatorStates.BARGE)) return wristCommand.andThen(elevatorCommand);

    // Base case where elevator and wrist move together
    return wristCommand.alongWith(elevatorCommand);
  }
    

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
