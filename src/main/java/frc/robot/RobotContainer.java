// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Vision.CAMERA_0_POS;
import static frc.robot.Constants.Vision.CAMERA_1_POS;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.Constants.ManipulatorStates;

public class RobotContainer {
  final CommandXboxController stick0 = new CommandXboxController(0);
  final CommandXboxController stick1 = new CommandXboxController(1);

  private final Swerve drivebase = new Swerve(stick0);
  private final Vision vision; //Warning is wrong
  private final Grabber grabber = new Grabber();
  private final Wrist wrist = new Wrist();
  private final Elevator elevator = new Elevator();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private ManipulatorState currentState = Constants.ManipulatorStates.IDLE;

  public RobotContainer() {

    switch (Constants.CURRENT_MODE) {
        case REAL:
            vision =
                new Vision( // Assuming this is what we are getting bc we get vision (right?)
                    drivebase::addVisionMeasurement,
                    new RealLimelightVisionIO("limelight-left", drivebase::getPitch),
                    new RealLimelightVisionIO("limelight-right", drivebase::getPitch));
            break;
        case SIM :
            vision =
                new Vision(
                    drivebase::addVisionMeasurement,
                    new SimPhotonVisionIO("camera_0", drivebase::getPose, CAMERA_0_POS),
                    new SimPhotonVisionIO("camera_1", drivebase::getPose, CAMERA_1_POS));
            break;
        default:
            vision = new Vision(drivebase::addVisionMeasurement, new BaseVisionIO() {}, new BaseVisionIO() {});
            break;
    }
    configureAutos();
  }

  private void configureAutos(){
    autoChooser.addOption("Barge", new PathPlannerAuto("Barge"));
    autoChooser.addOption("Left 4gp", new PathPlannerAuto("left 4gp"));
    autoChooser.addOption("Right 4gp", new PathPlannerAuto("right 4gp"));
    

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }





  
  
  private Command setStateCommand(ManipulatorState target) {
    ManipulatorState lastState = currentState;
    currentState = target;

    // Special cases where elevator moves first
      if (target.equals(ManipulatorStates.IDLE)) {
        return elevator.setPositionCommand(target.elevatorPos).andThen(wrist.setAngleCommand(target.wristPos)); // go to e/and/w positions
      }

    // Special cases where wrist moves first
    for (ManipulatorState state : ManipulatorStates.WRIST_FIRST_STATES) {
      if (target.equals(state)) { // L2 or Ground Algae
        return wrist.setAngleCommand(target.wristPos).andThen(elevator.setPositionCommand(target.elevatorPos));
      } 
    }
    
    if (lastState.equals(ManipulatorStates.BARGE))
      return wrist.setAngleCommand(target.wristPos).andThen(elevator.setPositionCommand(target.elevatorPos));

    // Base case where elevator and wrist move together
    return wrist.setAngleCommand(target.wristPos).alongWith(elevator.setPositionCommand(target.elevatorPos));
  }







  private Command scoreCommand() {
    if (currentState.equals(ManipulatorStates.L1)) {
      return grabber.spinCommand(Constants.Grabber.EXTAKE_CORAL_L1_CURRENT);
    }

    return wrist.setAngleCommand(ManipulatorStates.POST_SCORE_WRIST_ANGLE).alongWith(grabber.coastCommand());
  }



  
  

  public Command getAutonomousCommand() {
    boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;

    return Commands.parallel(drivebase.driveToPose(isRed? FlippingUtil.flipFieldPose(Constants.Alignment.REEF_4_LEFT):Constants.Alignment.REEF_4_LEFT), setStateCommand(Constants.ManipulatorStates.L4)).andThen(scoreCommand()).andThen(setStateCommand(Constants.ManipulatorStates.IDLE));
  }
}
