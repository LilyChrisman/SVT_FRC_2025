package frc.robot.commands.lift;

import frc.robot.subsystems.ExtakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class LiftDriveToPosition extends Command{
    private final ExtakeSubsystem extake;
    private double position;

    public LiftDriveToPosition(double position){
        extake = new ExtakeSubsystem();
        this.position = position;
    }

    //Things we want to happen when we initialize the command
    public void initialize(){

    }

    //What actually happens when we call the command
    public void execute(){
        extake.liftGoToPos(position);
    }

     // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }


}

