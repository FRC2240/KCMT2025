package frc.robot.subsystems;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
    TalonFX motor = new TalonFX(Constants.Funnel.MOTOR_ID);

    TorqueCurrentFOC req = new TorqueCurrentFOC(0);

    public Command spinCommand() {
        return Commands.run(() -> {
            motor.setControl(req.withOutput(Constants.Funnel.DEFAULT_CURRENT));
        }, this);
    }

    public Command stopCommand() {
        return Commands.run(() -> {
            motor.setControl(req.withOutput(Amps.of(0)));
        }, this);
    }
}
