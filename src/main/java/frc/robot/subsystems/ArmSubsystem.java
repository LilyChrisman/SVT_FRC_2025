package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    /** Motor for arm on the lift that has the grabber on the end */
    private final TalonFX armMotor = new TalonFX(42, "rio");

    //set positions for the arm to score
    public final double ARM_EXTAKE_L4 = 40;
    public final double ARM_INTAKE_CORAL = 0;
    public final double ARM_EXTAKE_L3 = 48;
    public final double ARM_EXTAKE_L2 = 20;
    public final double ARM_EXTAKE_L1 = 18;


    public ArmSubsystem(){
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.1; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output

        slot0Configs.kP = 0.1; // A position error of 3 rotations results in 1.2 V output
        slot0Configs.kI = 0.0; // no output for integrated error
        slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
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
    }

    //Command to move the arm to a position, currently using Motion magic
    //If magic motion is not working, set MotionMagicVoltage to PositionVoltage. Will only use PID and optional feedforward
    public Command armGoToPosCommand(double position){
        final MotionMagicVoltage m_request = new MotionMagicVoltage(position).withSlot(0);
        return run(() -> {
            armMotor.setControl(m_request);
        });
    }

    
}
