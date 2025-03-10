package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX rotatorMotor = new TalonFX(51, "rio");
    private final TalonFX wheelMotor = new TalonFX(52, "rio");

    private final double IDLE_POS = 0;
    private final double INTAKE_POS = 0;
    private final double TRANSFER_POS = 0;

    public IntakeSubsystem(){
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

        rotatorMotor.getConfigurator().apply(slot0Configs);
        rotatorMotor.setNeutralMode(NeutralModeValue.Brake);
        wheelMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void runMotorManual(double direction) {
        if(this.getCurrentCommand() != null){
            this.getCurrentCommand().cancel();
        }
        this.rotatorMotor.set(direction * 0.3);
    }

    public Command goToPos(double pos){
        final MotionMagicVoltage m_request = new MotionMagicVoltage(pos)
            .withSlot(0);
         return run(() -> {
            rotatorMotor.setControl(m_request);
         });
    }

    public Command runIntake(double speed){
        final VoltageOut m_request = new VoltageOut(speed);
        return run(() -> {
            wheelMotor.setControl(m_request);
        });
    }
}
