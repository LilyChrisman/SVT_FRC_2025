package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtakeSubsystem extends SubsystemBase{
    private final TalonFX liftMotor1 = new TalonFX(31, "rio");
    private final TalonFX liftMotor2 = new TalonFX(32, "rio");
    private final TalonFX grabberMotor = new TalonFX(33, "rio");



    public ExtakeSubsystem(){
        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        liftMotor1.getConfigurator().apply(slot0Configs);
    }

    public void periodic(){
        
    }

    public Command liftGoToPosCommand(double position){
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
         return run(() -> {
            liftMotor1.setControl(m_request.withPosition(position));
            liftMotor2.setControl(liftMotor1.getAppliedControl());
         });
    }

    public Command grab(){
        final VoltageOut m_request = new VoltageOut(0.5);
        return run(() -> grabberMotor.setControl(m_request));
    }

    public Command release(){
        final VoltageOut m_request = new VoltageOut(-0.5);
        return run(() -> grabberMotor.setControl(m_request));
    }

}
