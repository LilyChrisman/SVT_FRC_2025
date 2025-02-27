// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtakeSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Initializing our two controllers
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController utilityController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );

  //Initializes our three subsystems
  private final ExtakeSubsystem extake = new ExtakeSubsystem();
  private final GrabberSubsystem grabber = new GrabberSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();

 // private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> driverXbox.getLeftY() * -1,
    () -> driverXbox.getLeftX() * -1
  ).withControllerRotationAxis(driverXbox::getRightX)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
    .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
    .robotRelative(true)
    .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> -driverXbox.getLeftY(),
    () -> -driverXbox.getLeftX()
  ).withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
    .withControllerHeadingAxis(
      () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
      () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2)
    ).headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //Named commands for autonomous. In path planner, match the name of the command to the one set in here and it will
    //run the command associated with it
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("grabCoral", grabber.grab2());
    NamedCommands.registerCommand("releaseCoral", grabber.release());
    NamedCommands.registerCommand("LiftScoreHigh", extake.liftGoToPosCommand(extake.LIFT_SCORE_L4));
    NamedCommands.registerCommand("LiftPickupCoral", extake.liftGoToPosCommand(extake.LIFT_PICKUP_CORAL));
    NamedCommands.registerCommand("ArmScoreHigh", arm.armGoToPosCommand(arm.ARM_EXTAKE_L4));
    NamedCommands.registerCommand("ArmPickupCoral", arm.armGoToPosCommand(arm.ARM_INTAKE_CORAL));
    NamedCommands.registerCommand("ArmScoreLow", arm.armGoToPosCommand(arm.ARM_EXTAKE_L1));

    //autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto Chooser", autoChooser);
    //Zeroes the gyro when the robot container is initialized
    drivebase.zeroGyro();
    //When the grabber is neither grabbing or releasing, it runs inward very slowly to hold any coral
    grabber.setDefaultCommand(grabber.brake());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      //All controller inputs for main teleop mode
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      utilityController.a().onTrue(Commands.deadline(
        arm.armGoToPosCommand(arm.ARM_INTAKE_CORAL),
        extake.liftGoToPosCommand(extake.LIFT_PICKUP_CORAL)
      ));
      utilityController.leftBumper().whileTrue(Commands.deadline(
        grabber.release()
        //arm.armGoToPosCommand(extake.ARM_INTAKE_CORAL)
      ));
      utilityController.y().onTrue(Commands.deadline(
        extake.liftGoToPosCommand(extake.LIFT_SCORE_L4),
        arm.armGoToPosCommand(arm.ARM_EXTAKE_L4)
      ));
      utilityController.x().onTrue(Commands.deadline(
        extake.liftGoToPosCommand(extake.LIFT_SCORE_L3),
        arm.armGoToPosCommand(arm.ARM_EXTAKE_L3)
      ));
      utilityController.b().onTrue(Commands.deadline(
        extake.liftGoToPosCommand(extake.LIFT_SCORE_L1),
        arm.armGoToPosCommand(arm.ARM_EXTAKE_L1)
      ));
      utilityController.rightBumper().whileTrue(grabber.grab2());
      utilityController.start()
        .whileTrue(Commands.run(() -> {
          extake.runMotor(utilityController.getLeftY());
        }));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return autoChooser.getSelected();
    return Commands.none();
  
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
