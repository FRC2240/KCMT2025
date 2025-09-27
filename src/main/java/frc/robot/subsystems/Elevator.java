package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
    private final TalonFX rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);

    public Elevator() {
        // Init PIDs and follower here
    }

    public Command setPositionCommand(Angle position) {
        return Commands.none();
    }

    public Command offsetCommand(Angle angle) {
        return Commands.none();
    }

    public Angle getPosition() {
        return leftMotor.getPosition().getValue();
    }
}
