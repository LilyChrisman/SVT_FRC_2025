package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.ScoringGoal;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorSubsystem extends SubsystemBase{
    /** Motors for lift */
    private final TalonFX liftMotor1 = new TalonFX(31, "rio");
    private final TalonFX liftMotor2 = new TalonFX(32, "rio");
    
    private final DigitalInput limitSwitch = new DigitalInput(0);
    
    //Set Points for lift for scoring and picking up coral
    protected final double LIFT_SCORE_L1 = -20;
    protected final double LIFT_SCORE_L2 = -1;
    protected final double LIFT_SCORE_L3 = -1;
    protected final double LIFT_SCORE_L4 = -83.0;
    protected final double LIFT_BOTTOM = 0;
    protected final double LIFT_PREPARE_INTAKE_CORAL = -13;
    protected final double LIFT_INTAKE_CORAL = -2;

    //Checks if we have run the lift down to zero position
    private boolean hasZeroed = false;

    private boolean isManual = false;

    public void toggleManual() {
        this.isManual = !this.isManual;
    }

    public void zeroPosition() {
        this.liftMotor1.setPosition(LIFT_BOTTOM);
    }

    // 1 for up, -1 for down
    public void runMotorManual(double direction) {
        if(this.getCurrentCommand() != null){
            this.getCurrentCommand().cancel();
        }
        this.liftMotor1.set(direction * 0.3);
    }

    public ElevatorSubsystem(){
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = .4; // A position error of 3 rotations results in 1.2 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps(Max Speed)
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)(How fast we want to get to max speed)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)(Idk)

        //Applies the configuration to the two motors then sets the second lift motor to follow
        //the voltage of the first lift motor
        liftMotor1.getConfigurator().apply(slot0Configs);
        liftMotor2.getConfigurator().apply(slot0Configs);
        final Follower m_Follower = new Follower(31, false);
        liftMotor2.setControl(m_Follower);
        liftMotor1.setNeutralMode(NeutralModeValue.Brake);
        liftMotor2.setNeutralMode(NeutralModeValue.Brake);
    }

    public void periodic(){
        //Let's us see the position of the lift on the dashboard
        SmartDashboard.putNumber("Elevator Position", liftMotor1.getPosition().getValueAsDouble());

        SmartDashboard.putBoolean("Elevator Manual Mode", this.isManual);

        /*
        if(limitSwitch.get() == true){
            hasZeroed = true;
            liftMotor1.setPosition(0);
        }
        if(hasZeroed == false){
            if(this.getCurrentCommand() != null){
                this.getCurrentCommand().cancel();
            }
            final VoltageOut m_request = new VoltageOut(-.45);
            liftMotor1.setControl(m_request);
        }
         */

        if (this.isManual) {
            this.runMotorManual(RobotContainer.utilityController.getLeftY());
        }
        
    }

    //Command to run the lift to a position. Uses Magic Motion profiling
    public Command goToScoringGoal(ScoringGoal goal){
        this.isManual = false;
        if(this.getCurrentCommand() != null) {
            this.getCurrentCommand().cancel();
        }

        double position = switch (goal) {
            case PrepareIntake -> LIFT_PREPARE_INTAKE_CORAL;
            case Intake -> LIFT_INTAKE_CORAL;
            case L1 -> LIFT_SCORE_L1;
            case L2 -> LIFT_SCORE_L2;
            case L3 -> LIFT_SCORE_L3;
            case L4 -> LIFT_SCORE_L4;
            default -> 0;
        };

        final MotionMagicVoltage m_request = new MotionMagicVoltage(position)
            .withSlot(0);
        return run(() -> {
           liftMotor1.setControl(m_request);
        });
    }

    boolean aroundPos(double pos) {
        double cpos = this.liftMotor1.getPosition().getValueAsDouble();
        return cpos >= pos-0.5 && cpos <= pos+0.5;
    }

    public Command runIntake(GrabberSubsystem grabber) {
        if(this.isManual) return Commands.run(() -> {}); // do nothing in manual for safety
        
        return Commands.sequence(
            this.goToScoringGoal(ScoringGoal.Intake).withTimeout(0.5),
            grabber.activeIntake().withTimeout(0.1),
            this.goToScoringGoal(ScoringGoal.PrepareIntake).withTimeout(0.25)
        );
    }
}
