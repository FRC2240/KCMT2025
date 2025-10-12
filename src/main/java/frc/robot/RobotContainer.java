// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Vision.CAMERA_0_POS;
import static frc.robot.Constants.Vision.CAMERA_1_POS;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  private final Swerve drivebase = new Swerve(stick0);
  private final Vision vision; //Warning is wrong

  public RobotContainer() {

    switch (Constants.CURRENT_MODE) {
        case REAL:
            vision =
                new Vision( // Assuming this is what we are getting bc we get vision (right?)
                    drivebase::addVisionMeasurement,
                    new RealLimelightVisionIO("limelight-left", drivebase::getPitch), //Why are we using this instead of Constants.Vision.CAM_POS, why is it different
                    new RealLimelightVisionIO("limelight-right", drivebase::getPitch)); 
            break;
        default: // Included for no errors
            vision = new Vision(drivebase::addVisionMeasurement, new BaseVisionIO() {}, new BaseVisionIO() {});
            break;
    }
  }







  public Command getAutonomousCommand() {
    boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;
    if (isRed) {
      // Do Flipped drive and set e/and/w
      return Commands.none();
    }
    // Do Normal drive and set e/and/w
    return Commands.none();
  }





  
  
  private Command setStateCommand(ManipulatorState target) {
    return Commands.print("Valid code");
  }

  private Command scoreCommand() {
    return Commands.print("Valid code");
  }
}
