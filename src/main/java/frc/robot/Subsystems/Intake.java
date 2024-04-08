// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  public static enum IntakePosition
  {
    intake,
    feed,
//    fireAmp,
//    fireSpeaker,
    none
  }

  private TalonFX aimMotor, shootMotor;
  private CANcoder encoder;
  private IntakePosition targetPosition;
  private DigitalInput sensor; // Detects presence of ring
  private DigitalInput motorSideSensor;
  private DigitalInput intakeLimit; // Detects hitting floor
  private DigitalInput feedLimit; // Detects hitting back of robot
  private PositionVoltage anglePosition = new PositionVoltage(0);

  /** Creates a new Intake. */
  public Intake()
  {
    aimMotor = new TalonFX(Constants.intakeAimMotorPort);
    shootMotor = new TalonFX(Constants.intakePullMotorPort);
    encoder = new CANcoder(Constants.intakeEncoderPort);
    sensor = new DigitalInput(Constants.intakeSensorPort);
    motorSideSensor = new DigitalInput(Constants.intakeMotorSideSensorPort);
    intakeLimit = new DigitalInput(Constants.intakeLimitPort);
    feedLimit = new DigitalInput(Constants.feedLimitPort);
    //aimMotor.getConfigurator().apply(Robot.ctreConfigs.intakeAimFXConfig);
    //encoder.getConfigurator().apply(Robot.ctreConfigs.intakeAimCANcoderConfig);// Don't know how to put in CTRE config yet)
    //aimMotor.getConfigurator().setPosition(0.0);
    aimMotor.setNeutralMode(NeutralModeValue.Brake);

    MagnetSensorConfigs c = new MagnetSensorConfigs();
    c.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    encoder.getConfigurator().apply(c);
    resetPosition();
  }

  public void resetPosition()
  {
    double absolutePosition = encoder.getAbsolutePosition().getValue();
    aimMotor.setPosition(absolutePosition);
  }
  public void setIntakeMotor(double speed)
  {
    shootMotor.set(speed);
  }
  private static double getAngleFor(IntakePosition position)
  {
switch (position)
    {
      case intake: // Move to intake position
        return Constants.intakeAngles.intake;
      case feed: // Move to shooter-feeding position
        return Constants.intakeAngles.feed;
  /*    case fireAmp: // Move to firing position for amp
        return Constants.intakeAngles.fireAmp;
      case fireSpeaker: // Move to firing position for speaker
        return Constants.intakeAngles.fireSpeaker;*/
      default:
        return 10; // So we'll never reach it
    }
  }
  public void moveTo(IntakePosition position)
  {
    if (getPosition() == position) return; // Don't do if we're already there
    targetPosition = position;
    if (targetPosition == IntakePosition.feed)// encoder.getAbsolutePosition().getValue() < getAngleFor(position))
      aimMotor.set(Constants.IntakeAim.speed);
    else if (targetPosition == IntakePosition.intake)// encoder.getAbsolutePosition().getValue() > getAngleFor(position))
      aimMotor.set(-Constants.IntakeAim.speed);
    else aimMotor.set(0);
    // aimMotor.setControl(anglePosition.withPosition(getAngleFor(position)));
  }
  public IntakePosition getPosition() // Which position the intake is currently in
  {
    // if (intakeLimit.get()) return IntakePosition.intake;
    // if (feedLimit.get()) return IntakePosition.feed;
    
    for (IntakePosition position: IntakePosition.values())
    if (distanceAngles(encoder.getAbsolutePosition().getValue(), getAngleFor(position)) < Constants.IntakeAim.threshold)
      return position;
    return IntakePosition.none;
  }
  public double getAimSpeed()
  {
    return aimMotor.get();
  }
  public boolean containsNote()
  {
    return !(sensor.get() || motorSideSensor.get()); //Two sensors, containsnote when either are active//
  }
  public static double distanceAngles(double a, double b)
  {
    return Math.min(Math.abs(a - b), 1 - Math.abs(a - b));
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Intake angle: ", encoder.getAbsolutePosition().getValue());
    SmartDashboard.putString("Intake position: ", getPosition().name());
    SmartDashboard.putBoolean("Note loaded: ", containsNote());
    SmartDashboard.putBoolean("Rear limit switch: ", feedLimit.get());
    SmartDashboard.putBoolean("Forward limit switch: ", intakeLimit.get());
    if ((getPosition() == IntakePosition.intake || intakeLimit.get())
      && targetPosition == IntakePosition.intake)
      aimMotor.stopMotor();
    if ((getPosition() == IntakePosition.feedee || feedLimit.get())
      && targetPosition == IntakePosition.feed)
      aimMotor.stopMotor();
    // This method will be called once per scheduler run
  }
}
