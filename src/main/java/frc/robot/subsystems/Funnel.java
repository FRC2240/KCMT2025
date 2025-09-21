package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
    TalonFX motor = new TalonFX(Constants.Funnel.MOTOR_ID);

    public Funnel() {
        // Sets the current to the default value
        ControlRequest req = new TorqueCurrentFOC(Constants.Funnel.DEFAULT_CURRENT);
        motor.setControl(req);
    }

    /***
     * Spin the funnel at a certain amerage
     */
    public Command spinFunnelCommand(Current current) {
        return Commands.run(() -> {
            ControlRequest req = new TorqueCurrentFOC(current);
            motor.setControl(req);
        });
    }
}
