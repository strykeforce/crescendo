package frc.robot.subsystems.intake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.standards.OpenLoopSubsystem;
import org.strykeforce.telemetry.TelemetryService;


public class IntakeSubsystem extends SubsystemBase implements OpenLoopSubsystem {
    private final IntakeIO io;
    // private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged(); // imports are not working properly waiting until vendordeps configured
    // private Logger logger LoggerFactory.getLogger(IntakeSubsystem.class); 
    // private org.littletonrobotics.junction.Logger advLogger =
    //     org.littletonrobotics.junction.Logger.getInstance();

    private IntakeState curState = IntakeState.NONE;

    private boolean beamBroken = false; 
    private double beamBreakStableCounts = 0; 


    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public IntakeState getState() {
        return curState;
    }

    public boolean getObjectStatus() {
        return (curState == IntakeState.HAS_PIECE);
    } 

    public void toIntaking() {
        //logger.info("To Intaking");
        curState = IntakeState.INTAKING;
        setPercent();;
    }

    // public void intakeOpenLoop(double percentOutput) {
    //     // intakeFalcon.set(ControlMode.PercentOutput, percentOutput)
    // }

    @Override
    public void setPercent() {
        io.setPct(IntakeConstants.kIntakePercentOutput);
    }

    // if the switch is closed, a stable count is incremented. if not, stable count is reset to zero. 
    public boolean isBeamBroken() {
        if (io.isFwdLimitSwitchClosed) beamBreakStableCounts ++;
        else beamBreakStableCounts = 0;

        beamBroken = (beamBreakStableCounts > IntakeConstants.kBeamBreakStableCounts);
        return (beamBreakStableCounts > IntakeConstants.kBeamBreakStableCounts);
    }

    // intake state system
    @Override
    public void periodic() {

        switch(curState) {
            case HAS_PIECE:
            // has a gamepiece, disables intake
                break;
            case INTAKING:
                if (isBeamBroken()) {
                    io.setPct(0);;
                    curState = IntakeState.HAS_PIECE;
                }
                break;
            default:
                break;
        }
    }

    public Set<Measure> getMeasures() {
        return Set.of(
            new Measure("State", () -> getState().ordinal()),
            new Measure("Beam Broken", () -> beamBroken ? 1.0 : 0.0));
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
      super.registerWith(telemetryService);
      io.registerWith(telemetryService);
    }

    public enum IntakeState {
        HAS_PIECE,
        INTAKING,
        NONE
    }
}
