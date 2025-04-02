// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import com.pathplanner.lib.commands.PathPlannerAuto;
import java.io.File;
import java.util.function.Function;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initializing our two controllers
  public static final CommandPS4Controller driverController = new CommandPS4Controller(0);
  public static final CommandXboxController utilityController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );

  final double DRIVE_CONTROLLER_SLOW_DOWN = 0.7;

  final double DRIVE_CONTROLLER_MOD = 0.6;
  boolean driveIsDefaultSpeed = true;

  private final Field2d field;

  void setDriveIsSlow(boolean isSlow) {
    this.driveIsDefaultSpeed = !isSlow;
    this.drivebase.setMaxSpeed(
      this.driveIsDefaultSpeed ? Constants.MAX_SPEED : Constants.SLOWED_SPEED
    );
  }

  SendableChooser<Command> auto_Chooser = new SendableChooser<>();

  //Initializes our three subsystems
  final ElevatorSubsystem elevator = new ElevatorSubsystem();
  final GrabberSubsystem grabber = new GrabberSubsystem();
  final ArmSubsystem arm = new ArmSubsystem();
  final IntakeSubsystem intake = new IntakeSubsystem();
  public final LimelightSubsystem limelight = new LimelightSubsystem(this);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> driverController.getLeftY(),
    () -> driverController.getLeftX()
  ).withControllerRotationAxis(driverController::getRightX)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(1)
  .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    .withControllerHeadingAxis(
      () -> driverController.getRightX(),
      () -> driverController.getRightY()
    ).headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
    .robotRelative(true)
    .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> -driverController.getLeftY(),
    () -> -driverController.getLeftX()
  ).withControllerRotationAxis(() -> driverController.getRawAxis(2))
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
    .withControllerHeadingAxis(
      () -> Math.sin(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
      () -> Math.cos(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2)
    ).headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //Named commands for autonomous. In path planner, match the name of the command to the one set in here and it will
    //run the command associated with it
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("grabCoral", grabber.passiveIntake());
    NamedCommands.registerCommand("releaseCoral", grabber.release().withTimeout(0.5));
    NamedCommands.registerCommand("LiftScoreHigh", elevator.goToScoringGoal(ScoringGoal.L4));
    NamedCommands.registerCommand("LiftPickupCoral", elevator.goToScoringGoal(ScoringGoal.Intake));
    NamedCommands.registerCommand("PrepareIntake", elevator.goToScoringGoal(ScoringGoal.PrepareIntake).withTimeout(1));
    NamedCommands.registerCommand("ArmScoreHigh", arm.goToScoringGoal(ScoringGoal.L4));
    NamedCommands.registerCommand("ArmPickupCoral", arm.goToScoringGoal(ScoringGoal.Intake));
    NamedCommands.registerCommand("ArmScoreLow", arm.goToScoringGoal(ScoringGoal.L1));
    NamedCommands.registerCommand("RunGroundIntake", Commands.run(() -> intake.runIntake(-3), intake));
    NamedCommands.registerCommand("Sheath", arm.sheath());

    

    NamedCommands.registerCommand("PrepareL4", Commands.sequence(
      Commands.print("RUNNING!!!!!!!!!!!!!!!!!!!!!!!!"),
      // make sure the arm swings out a little, as to not hit the shoe box
      arm.goToScoringGoal(ScoringGoal.L4).withTimeout(0.3),
      Commands.deadline(
        elevator.goToScoringGoal(ScoringGoal.L4).withTimeout(1),
        arm.goToScoringGoal(ScoringGoal.L4).withTimeout(0.3)
      )
    ));

     auto_Chooser = AutoBuilder.buildAutoChooser();
     SmartDashboard.putData("Auto Chooser", auto_Chooser);
    //Zeroes the gyro when the robot container is initialized
    drivebase.zeroGyro();
    //When the grabber is neither grabbing or releasing, it runs inward very slowly to hold any coral
    grabber.setDefaultCommand(grabber.passiveIntake());
    intake.setDefaultCommand(intake.goToPos(IntakePosition.Up));

    field = new Field2d();
    drivebase.field = field;
    SmartDashboard.putData("Field: ", field);

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      driverController.options().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }

    if(DriverStation.isTest()) {
      // scoring
      // composing a deadline of commands into a funtion
      Function<ScoringGoal, Command> scoringCommand = (goal) -> {
        return Commands.sequence(
          // make sure the arm swings out a little, as to not hit the shoe box
          //arm.goToScoringGoal(goal).withTimeout(0.3),
          Commands.deadline(
            elevator.goToScoringGoal(goal)//,
            //arm.goToScoringGoal(goal).withTimeout(0.3)
          )
        );
      };
      utilityController.y().onTrue(scoringCommand.apply(ScoringGoal.L4));
      utilityController.x().onTrue(scoringCommand.apply(ScoringGoal.L3));
      utilityController.b().onTrue(scoringCommand.apply(ScoringGoal.L2));
    } else {
      // All controller inputs for main teleop mode
      // driver
      driverController.options()
        .onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverController.L3()
        .onTrue(Commands.runOnce(() -> {
          this.setDriveIsSlow(true);
        }));
      
      // ground intake control
      driverController.L2().whileTrue(
        Commands.run(() -> {
          intake.runIntake(1);
        }, intake)
      );
      driverController.R2().onTrue(
        Commands.run(() -> {
          intake.runIntake(-3);
        }, intake)
      );
      driverController.R2().onFalse(
        Commands.run(() -> {
          intake.runIntake(0);
        }, intake)
      );
      driverController.circle().onTrue(
        Commands.run(() -> {
          this.setDriveIsSlow(false);
          intake.runIntake(0);
        }, intake)
      );
      driverController.R1().onTrue(
        intake.goToPos(IntakePosition.Down)
      );
      driverController.L1().onTrue(
        intake.goToPos(IntakePosition.Up)
      );
      // reset state, and stop commands
      driverController.triangle().onTrue(intake.killSwitch());

      driverController.povRight().whileTrue(new AutoAlign(true, drivebase));
      driverController.povLeft().whileTrue(new AutoAlign(false, drivebase));   

      // operator
      // extake / intake
      utilityController.leftBumper().whileTrue(grabber.release());
      utilityController.rightBumper().onTrue(elevator.runIntake(grabber));
      utilityController.rightTrigger().whileTrue(grabber.activeIntake(-4));
      
      // elevator/arm preset positions
      utilityController.a().onTrue(Commands.deadline(
        elevator.goToScoringGoal(ScoringGoal.PrepareIntake),
        arm.goToScoringGoal(ScoringGoal.PrepareIntake)
      ));
      // scoring
      // composing a deadline of commands into a funtion
      Function<ScoringGoal, Command> scoringCommand = (goal) -> {
        return Commands.sequence(
          Commands.print("Input is recieved"),
          // make sure the arm swings out a little, as to not hit the shoe box
          arm.goToScoringGoal(goal).withTimeout(0.3),
          Commands.deadline(
            elevator.goToScoringGoal(goal),
            arm.goToScoringGoal(goal).withTimeout(0.3)
          )
        );
      };
      utilityController.y().onTrue(scoringCommand.apply(ScoringGoal.L4));
      utilityController.x().onTrue(scoringCommand.apply(ScoringGoal.L3));
      utilityController.b().onTrue(scoringCommand.apply(ScoringGoal.L2));

      // manual override toggle for controlling the elevator
      utilityController.povLeft()
        .onTrue(Commands.runOnce(elevator::toggleManual));
      // manual override toggle for controlling the arm
      utilityController.povRight()
        .onTrue(Commands.runOnce(arm::toggleManual));
      // go down
      utilityController.povDown().onTrue(elevator.goToScoringGoal(ScoringGoal.Intake));

      // set elevator zero position manually
      // I don't think this does anything
      utilityController.start()
        .onTrue(Commands.runOnce(elevator::zeroPosition));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  long startingTime = 0;
  long runningTime = 2000;
  public Command getAutonomousCommand() {
    // drive
     //return Commands.run(
       //() -> drivebase.drive(new ChassisSpeeds(-1, 0, 0)),
       //drivebase
     //).withTimeout(1);

     return auto_Chooser.getSelected();

    //return new PathPlannerAuto("testi");

    //SmartDashboard.putData(auto_chooser);
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}