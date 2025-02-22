package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase{
    /** Motor for coral intake */
    private final TalonFX grabberMotor = new TalonFX(33, "rio");
    private final AnalogInput grabberSensor = new AnalogInput(0);

    public GrabberSubsystem(){
        var slot2Configs = new Slot2Configs();
        slot2Configs.kP = 0.5;
        slot2Configs.kI = 0; // no output for integrated error
        slot2Configs.kD = 0;

        grabberMotor.getConfigurator().apply(slot2Configs);
    }

    public Command grab(){
        final DutyCycleOut m_request = new DutyCycleOut(0.5);
        return run(() -> {
            while(grabberSensor.getValue() < 1){
                grabberMotor.setControl(m_request);
            }
        });
    }

    public Command grab2(){
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(2);
        return run(() -> grabberMotor.setControl(m_request.withPosition(.5)));
    }

    public Command release(){
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(2);
        return run(() -> grabberMotor.setControl(m_request.withPosition(-.5)));
    }

}
