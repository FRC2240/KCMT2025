package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;
import frc.robot.Constants;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase {

    private final CANdle candle = new CANdle(Constants.Candle.CANDLE_ID);
    private final Supplier<Boolean> hasGP;
    private final LarsonAnimation larsonAnim = new LarsonAnimation(0, 0, 0, 0, 0.5, Constants.Candle.LED_COUNT,
            BounceMode.Center, 3);
    private final RainbowAnimation rainbowAnim = new RainbowAnimation(0.5, 0.5, Constants.Candle.LED_COUNT);
    private int cyclesInState = 0;

    private enum State {
        AUTONOMOUS,
        ENABLED,
        DISABLED,
        DISABLED_BLINK,
        ERROR,
        BROWN,
        HASGP
    }

    State state = State.DISABLED;

    public Candle(Supplier<Boolean> hasGP) {
        this.hasGP = hasGP;

        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 0.5;
        candle.configAllSettings(config);
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0);
        candle.configLOSBehavior(true, 1000);
    }


    @Override
    public void periodic() {

        // Default color based off alliance
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            return;

        if (alliance.get().equals(Alliance.Red)) {
            larsonAnim.setR(255);
            larsonAnim.setB(0);
            larsonAnim.setG(0);
        } else {
            larsonAnim.setR(0);
            larsonAnim.setB(255);
            larsonAnim.setG(0);
        }

        State previousState = state;
        if (DriverStation.isAutonomous()) {
            state = State.AUTONOMOUS;
        } else if (DriverStation.isDisabled()) {
            double secondsInState = cyclesInState / 50.0;

            if (secondsInState < 3 && state != State.DISABLED) {
                state = State.DISABLED_BLINK;
            } else if (DriverStation.isEStopped()) {
                state = State.ERROR;
            } else {
                state = State.DISABLED;
            }

        } else if (hasGP.get()) {
            state = State.HASGP;
        } else if (RobotController.getBatteryVoltage() < 0.9) {
            state = State.BROWN;
        } else if (DriverStation.isEnabled()) {
            state = State.ENABLED;
        } else {
            state = State.ERROR;
        }

        if (!state.equals(previousState)) {
            cyclesInState = 0;
            candle.clearAnimation(0);
        }
        cyclesInState++;

        switch (state) {
            case AUTONOMOUS:
                candle.animate(larsonAnim);
                break;
            case HASGP:
                candle.setLEDs(0, 255, 0);
                break;
            case DISABLED:
                if (alliance.get().equals(Alliance.Red)) {
                    candle.setLEDs(255, 0, 0);
                } else {
                    candle.setLEDs(0, 0, 255);
                }
                break;
            case BROWN:
                candle.setLEDs(0, 0, 0);
                break;
            case DISABLED_BLINK:
                if (cyclesInState % 10 > 5) {
                    candle.setLEDs(255, 255, 0);
                } else {
                    candle.setLEDs(0, 0, 0);
                }
                break;
            case ENABLED:
                candle.animate(rainbowAnim);
                break;
            case ERROR:
                if (cyclesInState % 10 > 5) {
                    candle.setLEDs(255, 0, 0);
                } else {
                    candle.setLEDs(255, 255, 0);
                }
                break;
            default:
                candle.setLEDs(255, 255, 255);
        }
    }
}