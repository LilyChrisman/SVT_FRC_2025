package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtakeSubsystem extends SubsystemBase{
    /** Motors for lift */
    private final TalonFX liftMotor1 = new TalonFX(31, "rio");
    private final TalonFX liftMotor2 = new TalonFX(32, "rio");
    
    
    

    public final double LIFT_SCORE_L1 = 0;
    public final double LIFT_SCORE_L2 = 0;
    public final double LIFT_SCORE_L3 = 0;
    public final double LIFT_SCORE_L4 = 0;
    public final double LIFT_BOTTOM = 0;
    public final double LIFT_PICKUP_CORAL = 0;
    
    public final double ARM_EXTAKE_HIGH = 0;
    public final double ARM_INTAKE_CORAL = 0;
    public final double ARM_EXTAKE_LOW = 0;

    public ExtakeSubsystem(){
        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

        liftMotor1.getConfigurator().apply(slot0Configs);
        liftMotor2.getConfigurator().apply(slot0Configs);
    }

    public void periodic(){
        
    }

    public Command liftGoToPosCommand(double position){
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
         return run(() -> {
            System.out.println("Test Print");
            liftMotor1.setControl(m_request.withPosition(position));
            liftMotor2.setControl(m_request.withPosition(position));
         });
    }
}
