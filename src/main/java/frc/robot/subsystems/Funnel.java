package frc.robot.subsystems;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
    TalonFX motor = new TalonFX(Constants.Funnel.MOTOR_ID);

    public Command spinCommand() {
        return Commands.run(() -> {
            ControlRequest req = new TorqueCurrentFOC(Constants.Funnel.DEFAULT_CURRENT);
            motor.setControl(req);
        });
    }

    public Command stopCommand() {
        return Commands.run(() -> {
            ControlRequest req = new TorqueCurrentFOC(Amps.of(0));
            motor.setControl(req);
        });
    }
}
