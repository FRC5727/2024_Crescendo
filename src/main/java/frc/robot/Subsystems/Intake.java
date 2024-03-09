// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

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
    fireAmp,
    fireSpeaker,
    none
  }

  private static final TalonFXConfiguration intakePID = null;

  private TalonFX aimMotor, shootMotor;
  private CANcoder encoder;
  private IntakePosition targetPosition;
  private DigitalInput sensor; // Detects presence of ring

  private PositionVoltage anglePosition = new PositionVoltage(0);

  /** Creates a new Intake. */
  public Intake()
  {
    aimMotor = new TalonFX(Constants.intakeAimMotorPort);
    shootMotor = new TalonFX(Constants.intakePullMotorPort);
    encoder = new CANcoder(Constants.intakeEncoderPort);
    sensor = new DigitalInput(Constants.intakeSensorPort);

    aimMotor.getConfigurator().apply(intakePID);
    aimMotor.getConfigurator().setPosition(0.0);
    resetPosition();

    aimMotor.getConfigurator().apply(Robot.ctreConfigs.intakeAimFXConfig);
    encoder.getConfigurator().apply(Robot.ctreConfigs.intakeAimCANcoderConfig);// Don't know how to put in CTRE config yet)
    aimMotor.getConfigurator().setPosition(0.0);
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
      case fireAmp: // Move to firing position for amp
        return Constants.intakeAngles.fireAmp;
      case fireSpeaker: // Move to firing position for speaker
        return Constants.intakeAngles.fireSpeaker;
      default:
        return 0;
    }
  }
  public void moveTo(IntakePosition position)
  {
    if (getPosition() == position) return; // Don't do if we're already there
    targetPosition = position;
    aimMotor.setControl(anglePosition.withPosition(new Rotation2d(getAngleFor(position)).getRotations()));
  }
  public IntakePosition getPosition() // Which position the intake is currently in
  {
    for (IntakePosition position: IntakePosition.values())
    if (Math.abs(encoder.getAbsolutePosition().getValue() - getAngleFor(position)) < Constants.intakeAimThreshold)
      return position;
    return IntakePosition.none;
  }
  public double getAimSpeed()
  {
    return aimMotor.get();
  }
  public boolean containsNote()
  {
    return !sensor.get(); // Inverted I guess
  }
  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Intake angle: ", encoder.getAbsolutePosition().getValue());
    SmartDashboard.putString("Nearest intake position: ", getPosition().name());
    SmartDashboard.putBoolean("Note loaded: ", containsNote());
    if (getPosition() == targetPosition)
      aimMotor.stopMotor();
    // This method will be called once per scheduler run
  }
}
