package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.ScoringGoal;
import frc.robot.subsystems.GrabberSubsystem;

public class ArmSubsystem extends SubsystemBase{
    /** Motor for arm on the lift that has the grabber on the end */
    private final TalonFX armMotor = new TalonFX(42, "rio");

    //set positions for the arm to score
    protected double posForL4 = 44;
    protected double posForL3 = 44;
    protected double posForL2 = 32;
    protected double posForL1 = 18;
    protected double posForCoralIntake = -1;
    
    // used to determine which goal pos to reposition when leaving manual mode
    protected ScoringGoal lastScoringGoal = ScoringGoal.None;
    protected double posBeforeManual = 0.0;

    private double lastScorePostion = 0;

    // why does this exist?????
    // TODO change all instances of this to isManual and hope it works
    protected boolean inManual = false;

    // bad hacky i hate it
    protected double lastManualPos = 0.0;
    protected boolean wasManual = false;

    private boolean isManual = false;

    public Command sheath() {
        return runOnce(() -> {
            double position = armMotor.getPosition().getValueAsDouble() - 14;
            final MotionMagicVoltage m_request = new MotionMagicVoltage(position)
            .withSlot(0);
            armMotor.setControl(m_request);
        });
    }

    public void cancelCommands() {
        if(this.getCurrentCommand() != null) {
            this.getCurrentCommand().cancel();
        }
    }
    public Command convinceStuartHeIsInTheRightSpot() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(this.armMotor.getPosition().getValueAsDouble())
            .withSlot(0);
        return run(() -> {
            armMotor.setControl(m_request);
        });
    }

    public void toggleManual() {
        this.isManual = !this.isManual;
    }

    protected void recordPositionBeforeManual() {
        this.posBeforeManual = this.armMotor.getPosition().getValueAsDouble();
    }

    // ran after exiting manual mode
    protected void setMotorPreferences() {
        double currentArmPos = this.lastManualPos;
        switch (this.lastScoringGoal) {
            case Intake -> this.posForCoralIntake = currentArmPos;
            case L1 -> this.posForL1 = currentArmPos;
            case L2 -> this.posForL2 = currentArmPos;
            case L3 -> this.posForL3 = currentArmPos;
            case L4 -> this.posForL4 = currentArmPos;
            default -> System.err.println("This should never print (ArmSubSystem.setMotorPreferences)");
        };
    }

    public void runMotorManual(double direction) {
        // this only runs the first cycle in a fresh manual run
        if(!this.inManual) {
            this.recordPositionBeforeManual();
        }

        this.cancelCommands();
        this.armMotor.set(direction * 0.2);

        this.inManual = true;
        // bad hacky i hate it
        this.lastManualPos = this.armMotor.getPosition().getValueAsDouble();
        this.wasManual = true;
    }


    public ArmSubsystem(){
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.0; // Something
        slot0Configs.kV = 0.1; // Velocity
        slot0Configs.kA = 0.0; // Acceleration

        slot0Configs.kP = 0.1; // A position error of 3 rotations results in .3 V output
        slot0Configs.kI = 0.0; // Integrated error
        slot0Configs.kD = 0.0; // Derivative
        slot0Configs.kG = 0.0;

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.01; // seemingly does nothing
        motionMagicConfigs.MotionMagicAcceleration = 1; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        //Applies slot 1 config to the arm motor
        armMotor.getConfigurator().apply(slot0Configs);
        armMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void periodic(){
        //Puts the arms position on the dashboard
        SmartDashboard.putNumber("Arm Position", armMotor.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("Arm Goal 1", this.posForL1);
        SmartDashboard.putNumber("Arm Goal 2", this.posForL2);
        SmartDashboard.putNumber("Arm Goal 3", this.posForL3);
        SmartDashboard.putNumber("Arm Goal 4", this.posForL4);
        SmartDashboard.putNumber("Arm Goal Intake", this.posForCoralIntake);

        SmartDashboard.putBoolean("Arm Manual Mode", this.isManual);

        if (this.isManual) {
            // direction inverted so it's good
            this.runMotorManual(-RobotContainer.utilityController.getRightY()*1.2);
        }

        // bad hacky i hate it
        if(this.wasManual) {
            this.setMotorPreferences();
            this.wasManual = false;
        }
    }

    //Command to move the arm to a position, currently using Motion magic
    //If magic motion is not working, set MotionMagicVoltage to PositionVoltage. Will only use PID and optional feedforward
    public Command goToScoringGoal(ScoringGoal goal){
        this.isManual = false;

        double position = switch (goal) {
            case Intake, PrepareIntake -> this.posForCoralIntake;
            case L1 -> this.posForL1;
            case L2 -> this.posForL2;
            case L3 -> this.posForL3;
            case L4 -> this.posForL4;
            default -> 0;
        };

        final MotionMagicVoltage m_request = new MotionMagicVoltage(position)
            .withSlot(0);
        this.lastScoringGoal = goal;
        System.out.println(lastScorePostion);
        return run(() -> {
            armMotor.setControl(m_request);
        });
    }

    
}
