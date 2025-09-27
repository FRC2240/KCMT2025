package frc.robot.subsystems;

import java.time.chrono.HijrahDate;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.led.LarsonAnimation;


import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

        






public class Candle extends SubsystemBase{


    private int wants_r = 0;
    private int wants_g = 0;
    private int wants_b = 0;
    private boolean hasGP;  
    private int cyclesInState = 0;
  

    class Colour{
        public int r;
        public int g;
        public int b;
        public Colour(int r, int g, int b){
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    private Colour teamcolour = new Colour(255, 255, 255);
    public CANdle candle;
    
    public Candle(Boolean hasGP)
    { 
        this.hasGP = hasGP;
        this.candle = new CANdle(Constants.Candle.CANDLE_ID);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness
        candle.configAllSettings(config); 
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0);
        candle.configLOSBehavior(true, 1000);

    }

    
    public enum RobotState {
        AUTO,
        DISABLED,
        DISABLED_BLINK,
        ENABLED,
        BROWN,
        ERROR,
        HASGP,
        WANTGP
    }
    
    private RobotState state = RobotState.DISABLED;
    private RobotState prev_state = RobotState.DISABLED; //Pervious state

    public Command periodiCommand() {
        if (DriverStation.getAlliance().isPresent()) {
            Alliance alliance = DriverStation.getAlliance().get();
            if (alliance == Alliance.Red) 
            {
                teamcolour.r = 255;
                teamcolour.g = 0;
                teamcolour.b = 0;
            } 
            else if (alliance == Alliance.Blue) 
            {
                teamcolour.r = 0;
                teamcolour.g = 0;
                teamcolour.b = 255;
            }
        }
    
        LarsonAnimation.setR(teamcolour.r); //no clue how to fix
        LarsonAnimation.setB(teamcolour.b);
        LarsonAnimation.setG(teamcolour.g);


        /*
        Step 1: Determine desired state
        Step 1a: Determine if a state change occurs
        Step 2: Switch on state
        */
        if (DriverStation.isAutonomous())
        {
            state = RobotState.AUTO;
        }
        else if (DriverStation.isDisabled())
        {
            if ((cyclesInState / 50.0) < 0.5) //cycles/50 = seconds
            {
                state = RobotState.DISABLED_BLINK;
            }
            else if (DriverStation.isEStopped())
            {
                state = RobotState.ERROR;
            }
            else
            {
            state = RobotState.DISABLED; // Not always blinking. Assume normal disabled then check for time in state
            }
        }
        else if (hasGP)
        {
            wants_r = 0;
            wants_g = 0;
            wants_b = 0;
            state = RobotState.HASGP;
        }
        else if (wants_r + wants_g + wants_b > 0 && !hasGP) // Rationale is that getting a GP resets all these to 0
        {
            state = RobotState.WANTGP;
        }
        else if (RobotController.getBatteryVoltage() < 9.0) // Turn LEDs off to conserve voltage under brown
        {
            state = RobotState.BROWN;
        }
        else if (DriverStation.isEnabled()) // Must be last because WantGP and HasGP conflict
        {
            state = RobotState.ENABLED;
        }
        else
        {
            state = RobotState.ERROR;
        }

  
        if (prev_state != state && prev_state != RobotState.DISABLED_BLINK)
        {
            cyclesInState = 0;
            candle.clearAnimation(0);
        }
    
        if (cyclesInState == 0)
        {
            candle.clearAnimation(0);
        }
        cyclesInState++;
        prev_state = state;

        switch (state) {
            case AUTO:

                SmartDashboard.putString("candlesim", "#8ACE00");
                Candle.animate(m_larson_auto); //no clue
                break;

            case WANTGP:
            if (cyclesInState % 2 == 0) // Blink every other frame, consider rasing modulo operand to reduce seizure risk
            {
                candle.setLEDs(wants_r, wants_g, wants_b);
                SmartDashboard.putString("candlesim", new Colour(wants_r, wants_b, wants_g).toString());
            }
            else 
            {
                SmartDashboard.putString("candlesim", "#000000");
                candle.setLEDs(0, 0, 0);
            }
                break;

            case HASGP:

                SmartDashboard.putString("candlesim", "#00FF00");
                candle.setLEDs(0, 255, 0);

                break;

            case DISABLED:
                
                SmartDashboard.putString("candlesim", new Colour(teamcolour.r, teamcolour.g, teamcolour.b).toString());
                candle.setLEDs(teamcolour.r, teamcolour.g, teamcolour.b);

                break;
                
            case DISABLED_BLINK:

            if (cyclesInState % 10 > 5)
            {
                SmartDashboard.putString("candlesim", "#FFFF00");
                candle.setLEDs(255, 255, 0);
            }
            else
            {
                SmartDashboard.putString("candlesim", "#000000");
                candle.setLEDs(0, 0, 0);
            }

                break;

            case ENABLED:
                SmartDashboard.putString("candlesim", "#FF00FF");
                candle.animate(rainbow); //No idea what rainbow is

                break;

            case ERROR:

            if (cyclesInState % 10 > 5)
            {
                SmartDashboard.putString("candlesim", "#FF0000");
                candle.setLEDs(255, 0, 0);
            }
            else
            {
                SmartDashboard.putString("candlesim", "#FFFF00");
                candle.setLEDs(255, 255, 0);
            }

                break;

            case BROWN:
                SmartDashboard.putString("candlesim", "#000000");
                candle.setLEDs(0, 0, 0);

                break;

            default:
                SmartDashboard.putString("candlesim", "#FFFFFF");
                candle.setLEDs(255, 255, 255);

                break;
        }
    };

    public Command WantGp(int r, int g, int b) {
        return Commands.run(() -> {
            wants_r = r;
            wants_g = g;
            wants_b = b;

        });
    }

    public boolean IsRed()
    {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    
    // If alliance is present and is Blue, return false
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
        return false;
    }
    
    return true;
    }
}

