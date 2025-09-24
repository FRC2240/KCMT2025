package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Elevator {
    public Command setPositionCommand(Angle position) {
        return Commands.none();
    }

    public Command offsetCommand(Angle angle) {
        return Commands.none();
    }
}
