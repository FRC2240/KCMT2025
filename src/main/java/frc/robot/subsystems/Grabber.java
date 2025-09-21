package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class Grabber {
    private TimeOfFlight sensor = new TimeOfFlight(Constants.Grabber.SENSOR_ID);
    private TalonFX motor = new TalonFX(Constants.Grabber.MOTOR_ID);

    public boolean has_gp() {
        Distance distance = Millimeters.of(sensor.getRange());
        return distance.compareTo(Constants.Grabber.INTAKE_THRESHOLD) <= 0;
    }

    private Command spinCommand(Current current) {
        return Commands.run(() -> {
            ControlRequest req = new TorqueCurrentFOC(current);
            motor.setControl(req);
        });
    }

    public Command intakeCoralCommand() {
        return spinCommand(Constants.Grabber.INTAKE_CORAL_CURRENT)
                .until(() -> {
                    return has_gp();
                }).withName("Intake Coral");
    }

    public Command intakeAlgaeCommand() {
        return spinCommand(Constants.Grabber.INTAKE_ALGAE_CURRENT)
                .withName("Intake Algae");
    }

    public Command extakeAlgaeCommand() {
        return spinCommand(Constants.Grabber.EXTAKE_ALGAE_CURRENT);
    }

    public Command coastCommand() {
        return Commands.runOnce(() -> {
            ControlRequest req = new CoastOut();
            motor.setControl(req);
        });
    }
}
