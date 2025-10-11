package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
    private TimeOfFlight sensor = new TimeOfFlight(Constants.Grabber.SENSOR_ID);
    private TalonFX motor = new TalonFX(Constants.Grabber.MOTOR_ID);

    TorqueCurrentFOC req = new TorqueCurrentFOC(0);
    VelocityDutyCycle brake = new VelocityDutyCycle(0);
    CoastOut coastReq = new CoastOut();

    public Grabber() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        conf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        conf.Slot0.kP = 10;
        conf.Slot0.kS = 6;

        motor.getConfigurator().apply(conf);
    }

    public boolean has_gp() {
        Distance distance = Millimeters.of(sensor.getRange());
        return distance.compareTo(Constants.Grabber.INTAKE_THRESHOLD) <= 0;
    }

    public Command spinCommand(Current current) {
        return Commands.run(() -> {
            motor.setControl(req.withOutput(current));
        }, this);
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

    public Command idleCommand() {
        return spinCommand(Amp.of(0));
    }

    public Command coastCommand() {
        return Commands.runOnce(() -> {
            motor.setControl(coastReq);
        }, this);
    }
}
