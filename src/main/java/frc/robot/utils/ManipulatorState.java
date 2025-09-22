package frc.robot.utils;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.units.measure.Angle;

public class ManipulatorState {
  private final Angle tolerance = Rotation.of(0.005); 
  public final Angle elevatorPos;
  public final Angle wristPos;

  public ManipulatorState(Angle elevatorPos, Angle wristPos) {
    this.elevatorPos = elevatorPos;
    this.wristPos = wristPos;
  }
  
  // Override function for more consise code. params are in turns.
  public ManipulatorState(double elevatorPos, double wristPos) {
    this.elevatorPos = Rotation.of(elevatorPos);
    this.wristPos = Rotation.of(wristPos);
  }

  // Creates a copy of the mainipulator state
  public ManipulatorState copy() {
    return new ManipulatorState(this.elevatorPos, this.wristPos);
  }

  // Compares against another manipulator state with the tolerance defined above.
  public boolean equals(ManipulatorState other) {
    return elevatorPos.isNear(other.elevatorPos, tolerance) && wristPos.isNear(other.wristPos, tolerance);
  }
}