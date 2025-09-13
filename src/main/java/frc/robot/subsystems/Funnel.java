package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
    TalonFX motor = new TalonFX(Constants.Funnel.MOTOR_ID);

    /***
     * Spin the funnel at a certain amerage
     */
    Command spinFunnelCommand(Current current) {
        return Commands.runOnce(() -> {
            TorqueCurrentFOC req = new TorqueCurrentFOC(current);
            motor.setControl(req);
        });
    }

    

}
