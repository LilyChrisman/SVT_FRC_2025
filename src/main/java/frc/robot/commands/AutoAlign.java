package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoAlign extends Command{
    private PIDController xController, yController, rotController;
    private HolonomicDriveController driveController;
    private boolean isRightSCore;
    private Timer dontSeeTagTimer, stopTimer;
    private double tagID = -1;
    private SwerveSubsystem drivebase;
    private boolean hasRotated = false;

    public final double DONT_SEE_TAG_WAIT_TIME = 3;
    public final double POSE_VALIDATION_TIME = 0.3;

    public final double X_REEF_ALIGNMENT_P = 1.25;
    public final double Y_REEF_ALIGNMENT_P = 1;
    public final double REEF_ALIGNMENT_D = 0.000;
    public final double ROT_REEF_ALIGNMENT_P = 0.11;

    public final double ROT_SETPOINT_REEF_ALIGNMENT = 0;
    public final double ROT_TOLERANCE_REEF_ALIGNMENT = 5;

    public final double X_SETPOINT_REEF_ALIGNMENT = -.6;
    public final double X_TOLERANCE_REEF_ALIGNMENT = .08;

    public final double Y_SETPOINT_REEF_ALIGNMENT = 0.1;
    public final double Y_TOLERANCE_REEF_ALIGNMENT = 0.05;

    public AutoAlign(boolean isRightSCore, SwerveSubsystem drivebase){
        xController = new PIDController(X_REEF_ALIGNMENT_P, 0, REEF_ALIGNMENT_D);
        yController = new PIDController(Y_REEF_ALIGNMENT_P, 0, REEF_ALIGNMENT_D);
        rotController = new PIDController(ROT_REEF_ALIGNMENT_P, 0, 0);
        driveController = new HolonomicDriveController(xController, yController, 
            new ProfiledPIDController(0.001, 0, 0, new TrapezoidProfile.Constraints(4.8, 5.0)));
        this.isRightSCore = isRightSCore;
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    public void initialize(){
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

       // rotController.setSetpoint(ROT_SETPOINT_REEF_ALIGNMENT);
        //rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);

       // xController.setSetpoint(X_SETPOINT_REEF_ALIGNMENT);
      //  xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

        yController.setSetpoint(isRightSCore ? Y_SETPOINT_REEF_ALIGNMENT*2 : -Y_SETPOINT_REEF_ALIGNMENT*2.6);
        yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);
       // yController.setSetpoint(isRightSCore ? Y_SETPOINT_REEF_ALIGNMENT : -Y_SETPOINT_REEF_ALIGNMENT);
       // yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);

        driveController.getThetaController().setGoal(ROT_SETPOINT_REEF_ALIGNMENT);
        driveController.getThetaController().setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);

        driveController.getXController().setSetpoint(X_SETPOINT_REEF_ALIGNMENT);
        driveController.getXController().setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

        driveController.getYController().setSetpoint(isRightSCore ? Y_SETPOINT_REEF_ALIGNMENT*1.8 : -Y_SETPOINT_REEF_ALIGNMENT*2.5);
        driveController.getYController().setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);

        if(LimelightHelpers.getTV("")){
            tagID = LimelightHelpers.getFiducialID("");
        }
    }

    public void execute(){
        if(LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID){
            this.dontSeeTagTimer.reset();



            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");

            double xSpeed = driveController.getXController().calculate(positions[2]);
            double ySpeed = -driveController.getYController().calculate(positions[0]);        
            double rotValue = -driveController.getThetaController().calculate(positions[4]);
            System.out.print("xSpeed");
            System.out.print(xSpeed);
            if(xSpeed <= .35){ xSpeed = .35;}

            if (!hasRotated) {
                //drivebase.drive(new Translation2d(0.0, 0.0), rotValue, false);
                hasRotated = true;
                return;
            }

            if (isFinished()) {
                //drivebase.drive(new Translation2d(0.0, 0.0), rotValue, false);
                end();
                return;
            }
            

           /*   if(positions[4] > ROT_TOLERANCE_REEF_ALIGNMENT || positions[4] < -ROT_TOLERANCE_REEF_ALIGNMENT){
                xSpeed = 0.0;
                ySpeed = 0.0;
            } else {
                rotValue = 0.0;
            } */

            SmartDashboard.putNumber("xSpeed: ", xSpeed);
            SmartDashboard.putNumber("ySpeed: ", ySpeed);
            SmartDashboard.putNumber("rotSpeed: ", rotValue);
            SmartDashboard.putNumber("xTarget", X_SETPOINT_REEF_ALIGNMENT);
            SmartDashboard.putNumber("yTarget", Y_SETPOINT_REEF_ALIGNMENT);
            SmartDashboard.putNumber(
                "RotTarget", ROT_SETPOINT_REEF_ALIGNMENT);

             drivebase.drive(
                new Translation2d(xSpeed, ySpeed),
                0.0,
                false
            ); 

          // drivebase.driveCommand2(() -> xSpeed, () -> ySpeed, () -> rotValue);

            if(!driveController.getThetaController().atSetpoint() ||
            !driveController.getYController().atSetpoint() ||
            !driveController.getXController().atSetpoint()){
                stopTimer.reset();
            } else {
                drivebase.drive(new Translation2d(), 0, false);
            }
        }
    }

    public void end(){
        drivebase.drive(new Translation2d(), 0, false);
    }

    public boolean isFinished(){
        return this.dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME) || 
            stopTimer.hasElapsed(POSE_VALIDATION_TIME);
    }
}