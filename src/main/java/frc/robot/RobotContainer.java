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
import frc.robot.vision.BaseVisionIO;
import frc.robot.vision.Vision;

public class RobotContainer {
  final CommandXboxController stick0 = new CommandXboxController(0);
  final CommandXboxController stick1 = new CommandXboxController(1);

  private final Swerve drivebase = new Swerve(stick0);
  private final Vision vision; //Warning is wrong

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    switch (Constants.CURRENT_MODE) {
        case REAL:
            vision =
                new Vision( // Assuming this is what we are getting bc we get vision (right?)
                    drivebase::addVisionMeasurement,
                    new RealLimelightVisionIO("limelight-left", drivebase::getPitch), //Why are we using this instead of Constants.Vision.CAM_POS
                    new RealLimelightVisionIO("limelight-right", drivebase::getPitch)); //Why are we using this instead of Constants.Vision.CAM_POS
            break;
        default: // Included for no errors
            vision = new Vision(drivebase::addVisionMeasurement, new BaseVisionIO() {}, new BaseVisionIO() {});
            break;
    }
    configureAutos();
  }

  private void configureAutos(){ // This may be why they aren't running.. Check Names please!
    autoChooser.addOption("Barge", new PathPlannerAuto("Barge"));
    autoChooser.addOption("Left 4gp", new PathPlannerAuto("left 4gp"));
    autoChooser.addOption("Right 4gp", new PathPlannerAuto("right 4gp"));
    

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }







  public Command getAutonomousCommand() {
    boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;
    if (isRed) {
      // Do Flipped drive and set e/and/w
      return Commands.parallel(drivebase.driveToPose(FlippingUtil.flipFieldPose(Constants.Alignment.REEF_4_LEFT)), setStateCommand(Constants.ManipulatorStates.L4)).andThen(scoreCommand()).andThen(setStateCommand(Constants.ManipulatorStates.IDLE));
    }
    // Do Normal drive and set e/and/w
    return Commands.parallel(drivebase.driveToPose(Constants.Alignment.REEF_4_LEFT), setStateCommand(Constants.ManipulatorStates.L4)).andThen(scoreCommand()).andThen(setStateCommand(Constants.ManipulatorStates.IDLE));
  }





  
  
  private Command setStateCommand(ManipulatorState target) {
    return Commands.print("Valid code");
  }

  private Command scoreCommand() {
    return Commands.print("Valid code");
  }
}
