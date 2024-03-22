// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ClimberCommand;
import frc.robot.Commands.GroundIntakeCommand;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Constants.Controls;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Intake.IntakePosition;
import frc.robot.oldcommands.*;
//import frc.robot.oldsubsystems.ArmSubsystem;
//import frc.robot.oldsubsystems.IntakeSubsystem;
//import frc.robot.oldsubsystems.LED;
import frc.robot.oldsubsystems.RobotPosition;
import frc.robot.oldsubsystems.TimerSubsystem;
//import frc.robot.oldsubsystems.ArmSubsystem.Position;

import static frc.robot.Constants.*;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
//  private final LED s_LED = new LED();
//  private final IntakeSubsystem s_Intake = new IntakeSubsystem(s_LED);
//  private final ArmSubsystem s_Arm = new ArmSubsystem(s_LED, s_Intake);
  private final Swerve s_Swerve = new Swerve();
// private final Climber s_Climber = new Climber();
  private final Intake s_Intake = new Intake();
  private final Shooter s_Shooter = new Shooter();
//  private final ClimberCommand climberCommand;
//  private final RobotPosition s_RobotPosition = new RobotPosition(s_Swerve);
private final Auto auto = new Auto();//s_Swerve, s_RobotPosition);
SendableChooser<Command> simpleAutoChooser;
private final @SuppressWarnings("unused") TimerSubsystem timerSubsystem = new TimerSubsystem();

//  private Position driverTargetPosition = Position.CHASSIS;

//  private final SendableChooser<ArmSubsystem.Position> positionChooser = new SendableChooser<>();

  // Drive Controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> Controls.driver.getRawAxis(translationAxis),
            () -> Controls.driver.getRawAxis(strafeAxis),
            () -> -Controls.driver.getRawAxis(rotationAxis),
            () -> false, // always field relative
            s_Swerve::getSpeedLimitXY,
            s_Swerve::getSpeedLimitRot
        ));
  //  climberCommand = new ClimberCommand(s_Climber);
  //  s_Climber.setDefaultCommand(climberCommand);
//    s_Intake.setDefaultCommand(Commands.startEnd(s_Intake::idle, () -> {}, s_Intake));
    configureBindings();

NamedCommands.registerCommand("Shoot", new ShootCommand(s_Intake, s_Shooter, Constants.shooterSpeakerSpeed)); //Shoot
NamedCommands.registerCommand("Shoot Amp", new ShootCommand(s_Intake, s_Shooter, Constants.shooterAmpSpeed)); //Shoot amp
NamedCommands.registerCommand("Intake", new GroundIntakeCommand(s_Intake)); //Intake
NamedCommands.registerCommand("Load", Commands.runOnce(() -> s_Intake.moveTo(IntakePosition.feed))); // Get in Shooter Position
NamedCommands.registerCommand("Intake Pos", Commands.runOnce(() -> s_Intake.moveTo(IntakePosition.intake))); // Get in Intake Position*/
NamedCommands.registerCommand("Wait", new WaitCommand(0.01)); //Wait, Josie

   new JoystickButton(Controls.driver, XboxController.Button.kBack.value)
    .onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));

    // Arm position chooser
//    positionChooser.setDefaultOption("--- Arm Direct Debug Positions ---", null);
//    for (Position pos : Position.values()) {
//      positionChooser.addOption(pos.toString(), pos);
//    }
//    SmartDashboard.putData("Position chooser", positionChooser);

simpleAutoChooser = new SendableChooser<Command>();

// Move a little, shoot, then move more and intake, then move back, then shoot again, then stop
Command twoRings =
      moveDistance(.240, 0, 0.240)
      .andThen(
        stopSwerve())
      .andThen(Commands.waitSeconds(0.5))
      .andThen(
        new ShootCommand(s_Intake, s_Shooter, Constants.shooterSpeakerSpeed))
      .andThen(
        new GroundIntakeCommand(s_Intake)
        .alongWith(moveDistance(1.5, 0, 3)
          .andThen(Commands.waitSeconds(0.01))
          .andThen(stopSwerve())))
      .andThen(
        Commands.runOnce(() -> s_Intake.moveTo(IntakePosition.feed))
        .alongWith(moveDistance(-1.5, 0, 3)))
      .andThen(
        stopSwerve(),
        new ShootCommand(s_Intake, s_Shooter, Constants.shooterSpeakerSpeed)
      );

    // Move a little, then shoot, then move more, then stop
    Command oneRing =
      moveDistance(.240, 0, 0.240)
      .andThen(
        stopSwerve())
      .andThen(Commands.waitSeconds(0.5))
      .andThen(
        new ShootCommand(s_Intake, s_Shooter, Constants.shooterSpeakerSpeed))
      .andThen(moveDistance(1.5, 0, 3)
          .andThen(Commands.waitSeconds(0.01))
          .andThen(stopSwerve()));

    simpleAutoChooser.addOption("One note", oneRing);
    simpleAutoChooser.addOption("Two notes", twoRings);
    // SmartDashboard.putData("Auto", simpleAutoChooser);
    // Easy way to test AutoBalance
    //SmartDashboard.putData("Auto-Balance", new AutoBalanceCommand(s_Swerve, s_LED));
  }

  private Command moveDistance(double x, double y, double t)
  {
    return Commands.waitSeconds(t).alongWith(
      Commands.runOnce(
        () -> s_Swerve.drive(new Translation2d(x / t, y / t), 0, true, true), s_Swerve));
  }
  private Command stopSwerve()
  {
    return Commands.runOnce(
      () -> s_Swerve.drive(new Translation2d(0, 0), 0, false, false), s_Swerve);
  }
  public Command getAutonomousCommand() {
//    return new PathPlannerAuto("New Auto");//auto.getAutoCommand();
//   return auto.getAutoCommand();

    // Shoot, then intake while driving forward for 2 seconds,
    // then move to feed while driving backward for 2 seconds,
    // then stop and shoot
    return auto.getAutoCommand(s_Swerve);
    // return simpleAutoChooser.getSelected();
  }

  /*
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController Xbox}/{@link
   * edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link
   * edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /* DRIVER BINDS */

//    new JoystickButton(Controls.driver, XboxController.Button.kA.value)
//    .onTrue(Commands.runOnce(() -> climberCommand.toggleMode()));
    // Driver arm controls
 /*  new JoystickButton(Controls.driver, XboxController.Button.kA.value)
      .onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_LOW));
    new JoystickButton(Controls.driver, XboxController.Button.kB.value)
      .onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_MID));
    new JoystickButton(Controls.driver, XboxController.Button.kY.value)
      .onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_HIGH));
*/
    Trigger driverLeftBumper = new JoystickButton(Controls.driver, XboxController.Button.kLeftBumper.value);
    Trigger driverRightBumper = new JoystickButton(Controls.driver, XboxController.Button.kRightBumper.value);
    Trigger driverLeftTrigger = new Trigger(() -> Controls.driver.getLeftTriggerAxis() > Controls.triggerAxisThreshold);
    Trigger driverRightTrigger = new Trigger(() -> Controls.driver.getRightTriggerAxis() > Controls.triggerAxisThreshold);
  
    driverLeftTrigger.whileTrue(new GroundIntakeCommand(s_Intake).andThen(Commands.runOnce(()->s_Intake.moveTo(IntakePosition.feed), s_Intake)));
    driverRightTrigger.whileTrue(new ShootCommand(s_Intake, s_Shooter, Constants.shooterSpeakerSpeed).andThen(Commands.runOnce(()->s_Intake.moveTo(IntakePosition.intake), s_Intake)));
    driverRightBumper.whileTrue(new ShootCommand(s_Intake, s_Shooter, Constants.shooterAmpSpeed));

    /*
    new JoystickButton(Controls.driver, XboxController.Button.kY.value)
      .whileTrue(Commands.runOnce(() -> s_Climber.move(Constants.climberSpeed)))
      .onFalse(Commands.runOnce(() -> s_Climber.stop()));
    new JoystickButton(Controls.driver, XboxController.Button.kA.value)
      .whileTrue(Commands.runOnce(() -> s_Climber.move(-Constants.climberSpeed)))
      .onFalse(Commands.runOnce(() -> s_Climber.stop()));
    */
    new POVButton(Controls.driver, 0).onTrue(Commands.runOnce(() -> s_Intake.moveTo(IntakePosition.feed)));
    new POVButton(Controls.driver, 180).onTrue(Commands.runOnce(() -> s_Intake.moveTo(IntakePosition.intake)));
    // Move to selected position
/*    Trigger armTrigger = 
      driverRightBumper.whileTrue(
        Commands.runOnce(s_Swerve::enableSpeedLimit)
          .andThen(Commands.runOnce(() -> s_Arm.setTargetPosition(s_Arm.isDirectMode() ? positionChooser.getSelected() : driverTargetPosition)))
          .andThen(new ArmCommand(s_Arm)));
    armTrigger
      .onFalse(
        Commands.waitSeconds(0.5)
          .andThen(Commands.runOnce(s_Swerve::disableSpeedLimit))
        .alongWith(new ArmCommand(s_Arm, Position.CHASSIS))
        .unless(s_Arm::isDirectMode));

    Trigger intakeSubstationTrigger = 
      driverRightTrigger.whileTrue(new IntakeForeverCommand(s_Intake)
        .alongWith(Commands.runOnce(s_Swerve::enableSpeedLimit))
        .alongWith(new ArmCommand(s_Arm, Position.INTAKE_SUBSTATION))
        .andThen(Commands.runOnce(s_Intake::idle, s_Intake)));
 
    intakeSubstationTrigger
      .onFalse(
        Commands.waitSeconds(0.3)
          .andThen(Commands.runOnce(s_Swerve::disableSpeedLimit))
        .alongWith(Commands.runOnce(s_Intake::idle, s_Intake))
        .alongWith(new ArmCommand(s_Arm, Position.CHASSIS)
          .unless(s_Arm::isDirectMode)));
        
    Trigger intakeGroundTrigger = 
      driverLeftTrigger.whileTrue(new IntakeCommand(s_Intake)
        .alongWith(new ArmCommand(s_Arm, Position.INTAKE_GROUND))
        .andThen(new ArmCommand(s_Arm, Position.CHASSIS)));
    
    intakeGroundTrigger
      .onFalse(new ArmCommand(s_Arm, Position.CHASSIS).unless(s_Arm::isDirectMode));

    // Place currently held game piece
    driverLeftBumper.whileTrue(Commands.startEnd(s_Intake::place, () -> {}, s_Intake));

    // Toggle between cones and cubes
    new JoystickButton(Controls.driver, XboxController.Button.kX.value).onTrue(Commands.runOnce(() -> s_Intake.toggleCube()));
*/
    SmartDashboard.putData("Zero Gyro", new InstantCommand(() -> s_Swerve.zeroHeading()));
  }
// TODO Replace this ugly hack
  // For some reason, after auto, the teleop controls are inverted
  public void hack() {
    s_Swerve.hack();
  }
  public void disabled() {
 //   s_Intake.disabled();
  }
}