package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    /** Motor for arm on the lift that has the grabber on the end */
    private final TalonFX armMotor = new TalonFX(34, "rio");

    public ArmSubsystem(){
         var slot1Configs = new Slot1Configs();
        slot1Configs.kP = 0.0001;
        slot1Configs.kI = 0; // no output for integrated error
        slot1Configs.kD = 0;

        armMotor.getConfigurator().apply(slot1Configs);
    }

    public Command armGoToPosCommand(double position){
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(1);
        return run(() -> {
            armMotor.setControl(m_request.withPosition(position));
        });
    }
}
