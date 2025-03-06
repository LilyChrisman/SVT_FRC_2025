package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase{
    /** Motor for coral intake */
    private final TalonFX grabberMotor = new TalonFX(41, "rio");
    private final AnalogInput grabberSensor = new AnalogInput(0);

    public GrabberSubsystem(){
        //basic PID Config
        var slot2Configs = new Slot2Configs();
        slot2Configs.kP = 2.4;
        slot2Configs.kI = 0; // no output for integrated error
        slot2Configs.kD = 0;

        //Applies config to grabber motor
        grabberMotor.getConfigurator().apply(slot2Configs);
    }

    //Meant to run continously until it senses it has a coral
    //Don't think it works
    public Command grab(){
        final DutyCycleOut m_request = new DutyCycleOut(0.5);
        return run(() -> {
            while(grabberSensor.getValue() < 1){
                grabberMotor.setControl(m_request);
            }
        });
    }

    //Runs grabber inward very slowly. Should be used to just hold the coral
    public Command passiveIntake(){
        final VoltageOut m_request = new VoltageOut(-0.5);
        return runOnce(() -> {
            grabberMotor.setControl(m_request);
        });
    }

    //grab command we are actively using. just turns inward very fast
    public Command activeIntake(){
        final VoltageOut m_request = new VoltageOut(-2);
        return run(() -> {
            grabberMotor.setControl(m_request);
        });
    }

    //just reverses the grabber motor to allow coral to fall out
    public Command release(){
        final VoltageOut m_request = new VoltageOut(.6);
        return run(() -> {
            grabberMotor.setControl(m_request);
        });
    }

}
