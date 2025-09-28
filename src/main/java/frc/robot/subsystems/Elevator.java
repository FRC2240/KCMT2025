package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
    private final TalonFX rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);
    
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotionMagicTorqueCurrentFOC req = new MotionMagicTorqueCurrentFOC(0);

    public Elevator() {
        // Init PIDs and follower here
        MotionMagicConfigs conf = talonFXConfigs.MotionMagic;

        conf.MotionMagicAcceleration = 250;
        conf.MotionMagicCruiseVelocity = 35;
        talonFXConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        talonFXConfigs.Slot0.kP = 40;
        talonFXConfigs.Slot0.kD = 10;
        talonFXConfigs.Slot0.kS = 4.5;

        leftMotor.getConfigurator().apply(talonFXConfigs);
        rightMotor.getConfigurator().apply(talonFXConfigs);

        rightMotor.setControl(new Follower(Constants.Elevator.LEFT_MOTOR_ID, false));

    
    }

    public Command setPositionCommand(Angle position) {
        return Commands.run(() -> {
            leftMotor.setControl(req.withPosition(position));
        }, this).withName("Set Elavator Position").until(() -> {
            return getPosition().isNear(position, Constants.Elevator.POSITION_THRESHOLD);
        });
    }

    public Command offsetCommand(Angle offset) {
        return Commands.runOnce(() -> {
            leftMotor.setControl(req.withPosition((getPosition().plus(offset))));
        }, this).withName("Elavator Offset");
    }

    public Angle getPosition() {
        return leftMotor.getPosition().getValue();
    }
}
