package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class Wrist {
    private final TalonFX motor = new TalonFX(Constants.Wrist.MOTOR_ID);

    public Command setAngleCommand(Angle angle) {
        return Commands.none();
    }

    public Command rezeroCommand() {
        return Commands.run(() -> {
            motor.setControl(new VoltageOut(Volt.of(-3)));
        }).until(() -> {
            return motor.getVelocity().getValue().compareTo(RotationsPerSecond.of(1)) <= 0;
        }).andThen(() -> {
            motor.setPosition(0);
            motor.setControl(new VoltageOut(0));
        }).withName("Rezero Wrist");
    }

    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    public Command offsetCommand(Angle angle) {
        return setAngleCommand(getAngle().plus(angle));
    }
}
