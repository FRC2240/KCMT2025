// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Vision.CAMERA_0_NAME;
import static frc.robot.Constants.Vision.CAMERA_0_POS;
import static frc.robot.Constants.Vision.CAMERA_1_NAME;
import static frc.robot.Constants.Vision.CAMERA_1_POS;

import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.*;
import frc.robot.utils.ManipulatorState;
import frc.robot.vision.RealLimelightVisionIO;
import frc.robot.vision.SimPhotonVisionIO;
import frc.robot.vision.BaseVisionIO;
import frc.robot.vision.Vision;
import frc.robot.Constants.Alignment;
import frc.robot.Constants.ManipulatorStates; // So i dont have to prepend "constants." on every state

public class RobotContainer {
  final CommandXboxController stick0 = new CommandXboxController(0);
  final CommandXboxController stick1 = new CommandXboxController(1);

  private final Vision vision;
  private final Swerve drivebase = new Swerve(stick0);
  private final Funnel funnel = new Funnel();
  private final Grabber grabber = new Grabber();
  private final Wrist wrist = new Wrist();
  private final Elevator elevator = new Elevator();
  private final Candle candle = new Candle(grabber::has_gp);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private ManipulatorState currentState = Constants.ManipulatorStates.IDLE;

  public RobotContainer() {
    addNamedCommands();

    switch (Constants.CURRENT_MODE) {
        case REAL:
            vision =
                new Vision(
                    drivebase::addVisionMeasurement,
                    new RealLimelightVisionIO(CAMERA_0_NAME, drivebase::getPitch),
                    new RealLimelightVisionIO(CAMERA_1_NAME, drivebase::getPitch));
            break;
        case SIM :
            vision =
                new Vision(
                    drivebase::addVisionMeasurement,
                    new SimPhotonVisionIO(CAMERA_0_NAME, drivebase::getPose, CAMERA_0_POS),
                    new SimPhotonVisionIO(CAMERA_1_NAME, drivebase::getPose, CAMERA_1_POS));
            break;
        default:
            vision = new Vision(drivebase::addVisionMeasurement, new BaseVisionIO() {}, new BaseVisionIO() {});
            break;
    }
    configureBindings();
    configureAutos();
  }

  private void addNamedCommands() {
    NamedCommands.registerCommand("algae intake", Commands.print("start algae intake").andThen(grabber.intakeAlgaeCommand()));
    NamedCommands.registerCommand("algae extake", Commands.print("start algae extake").andThen(grabber.extakeAlgaeCommand()));
    NamedCommands.registerCommand("algae ground", Commands.print("start ground algae intake").andThen(setStateCommand(ManipulatorStates.GROUND_ALGAE)));
    NamedCommands.registerCommand("algae l2", Commands.print("start algae l2").andThen(setStateCommand(ManipulatorStates.ALGAE_L2)));
    NamedCommands.registerCommand("algae l3", Commands.print("start algae l3").andThen(setStateCommand(ManipulatorStates.ALGAE_L3)));
    NamedCommands.registerCommand("barge", Commands.print("start barge").andThen(setStateCommand(ManipulatorStates.BARGE)));
    NamedCommands.registerCommand("processor", Commands.print("start processor").andThen(setStateCommand(ManipulatorStates.PROCESSOR)));
    NamedCommands.registerCommand("score", Commands.print("start score").andThen(scoreCommand()));
    NamedCommands.registerCommand("intake", Commands.print("start intake").andThen(setStateCommand(ManipulatorStates.INTAKE).alongWith(grabber.intakeCoralCommand())));
    NamedCommands.registerCommand("extake", Commands.print("start extake").andThen(grabber.spinCommand(Constants.Grabber.EXTAKE_CORAL_L1_CURRENT)));
    NamedCommands.registerCommand("l4", Commands.print("start l4").andThen(setStateCommand(ManipulatorStates.L4)));
    NamedCommands.registerCommand("l3", Commands.print("start l3").andThen(setStateCommand(ManipulatorStates.L3)));
    NamedCommands.registerCommand("l2", Commands.print("start l2").andThen(setStateCommand(ManipulatorStates.L2)));
    NamedCommands.registerCommand("l1", Commands.print("start l1").andThen(setStateCommand(ManipulatorStates.L1)));
    NamedCommands.registerCommand("idle", Commands.print("start idle").andThen(setStateCommand(ManipulatorStates.IDLE).alongWith(grabber.idleCommand())));
    //Add auto align here
  }

  private void configureBindings() {
    // Default comand is to drive Field Oriented with Angular Velocity
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(drivebase.driveAngularVelocity));

    funnel.setDefaultCommand(funnel.spinCommand());

    // driverXbox.a().onTrue(drivebase.sysIdDriveMotorCommand());

    // Intake coral
    stick0.leftBumper().and(stick0.leftTrigger().negate())
        .onTrue(setStateCommand(ManipulatorStates.INTAKE).alongWith(grabber.intakeCoralCommand()));

    // Intake Algae from ground
    stick0.leftBumper().and(stick0.leftTrigger())
        .onTrue(setStateCommand(ManipulatorStates.GROUND_ALGAE).alongWith(grabber.intakeAlgaeCommand()));

    // Goes to algae l2
    stick0.a().and(stick0.leftTrigger())
        .onTrue(setStateCommand(ManipulatorStates.ALGAE_L2).alongWith(grabber.idleCommand()));

    // Goes to algae l3
    stick0.x().and(stick0.leftTrigger())
        .onTrue(setStateCommand(ManipulatorStates.ALGAE_L3).alongWith(grabber.idleCommand()));

    // Goes to barge
    stick0.leftTrigger().and(stick0.y())
        .onTrue(setStateCommand(ManipulatorStates.BARGE));

    // Goes to processor
    stick0.b().and(stick0.leftTrigger())
        .onTrue(setStateCommand(ManipulatorStates.PROCESSOR));

    // Scores
    stick0.rightBumper().and(stick0.leftTrigger().negate())
        .onTrue(scoreCommand());

    // Extakes algae
    stick0.rightBumper().and(stick0.leftBumper())
        .onTrue(grabber.extakeAlgaeCommand());

    // Goes to l4
    stick0.y().and(stick0.leftTrigger().negate())
        .onTrue(setStateCommand(ManipulatorStates.L4));

    // Goes to l3
    stick0.x().and(stick0.leftTrigger().negate())
        .onTrue(setStateCommand(ManipulatorStates.L3));

    // Goes to l2
    stick0.a().and(stick0.leftTrigger().negate())
        .onTrue(setStateCommand(ManipulatorStates.L2));

    // Goes to l1
    stick0.b()
        .onTrue(setStateCommand(ManipulatorStates.L1));

    // Idle
    stick0.rightTrigger()
        .onTrue(setStateCommand(ManipulatorStates.IDLE).alongWith(grabber.idleCommand()));

    // Rezero wrist
    stick1.start()
        .whileTrue(wrist.rezeroCommand());

    // Offest wrist up
    stick1.y()
        .onTrue(wrist.offsetCommand(Constants.Wrist.OFFSET_AMOUNT));

    // Offset wrist down
    stick1.b()
        .onTrue(wrist.offsetCommand(Constants.Wrist.OFFSET_AMOUNT.times(-1)));

    // Offset elevator up
    stick1.x()
        .onTrue(elevator.offsetCommand(Constants.Elevator.OFFSET_AMOUNT));

    // Offset elevator down
    stick1.a()
        .onTrue(elevator.offsetCommand(Constants.Elevator.OFFSET_AMOUNT.times(-1)));

    // Align to left side
    stick0.povLeft()
        .onTrue(drivebase.alignCommand(Alignment.LEFT));


    // Align to right side
    stick0.povRight()
        .onTrue(drivebase.alignCommand(Alignment.RIGHT));

    // Ground algae intake
    stick1.rightTrigger()
        .onTrue(setStateCommand(ManipulatorStates.GROUND_ALGAE));

    // Funnel toggle
    stick1.leftTrigger()
        .toggleOnTrue(funnel.stopCommand());
  }

  private void configureAutos(){
    //autoChooser.setDefaultOption("Do Nothing", null);
    // add autos here using format shown bellow
    autoChooser.addOption("Barge", new PathPlannerAuto("Barge"));
    autoChooser.addOption("Left 4gp", new PathPlannerAuto("left 4gp"));
    autoChooser.addOption("Right 4gp", new PathPlannerAuto("right 4gp"));
    

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  
  private Command setStateCommand(ManipulatorState target) {
    ManipulatorState lastState = currentState;
    currentState = target;

    Command elevatorCommand = elevator.setPositionCommand(target.elevatorPos);
    Command wristCommand = wrist.setAngleCommand(target.wristPos);

    // Special cases where elevator moves first
    for (ManipulatorState state : ManipulatorStates.ELEVATOR_FIRST_STATES) {
      if (target.equals(state)) {
        System.out.println("Elevator First Command");
        return elevatorCommand.andThen(wristCommand);
      }
    }

    // Special cases where wrist moves first
    for (ManipulatorState state : ManipulatorStates.WRIST_FIRST_STATES) {
      if (target.equals(state)) {
        System.out.println("Wrist First Command");
        return wristCommand.andThen(elevatorCommand);
      } 
    }
    if (lastState.equals(ManipulatorStates.BARGE))
      return wristCommand.andThen(elevatorCommand);

    // Base case where elevator and wrist move together
    return wristCommand.alongWith(elevatorCommand);
  }

  private Command scoreCommand() {
    if (currentState.equals(ManipulatorStates.L1)) {
      return grabber.spinCommand(Constants.Grabber.EXTAKE_CORAL_L1_CURRENT);
    }

    return wrist.setAngleCommand(ManipulatorStates.POST_SCORE_WRIST_ANGLE).alongWith(grabber.coastCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
