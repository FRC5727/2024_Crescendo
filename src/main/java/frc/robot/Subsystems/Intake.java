// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  public static enum IntakePosition
  {
    intake,
    feed,
    fireAmp,
    fireSpeaker,
  }

  private TalonFX aimMotor, shootMotor;
  private CANcoder encoder;
  private IntakePosition targetPosition;

  /** Creates a new Intake. */
  public Intake()
  {
    aimMotor = new TalonFX(Constants.intakeAimMotorPort);
    shootMotor = new TalonFX(Constants.intakePullMotorPort);
    encoder = new CANcoder(Constants.intakeEncoderPort);
  }

  public void shoot(double speed)
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
  public void moveTo(IntakePosition position, double speed)
  {
    targetPosition = position;
    if (encoder.getAbsolutePosition().getValue() < getAngleFor(position))
      aimMotor.set(speed);
    else if (encoder.getAbsolutePosition().getValue() > getAngleFor(position))
      aimMotor.set(-speed);
    else
      aimMotor.set(0);
  }
  public IntakePosition getPosition() // Which position the intake is currently in
  {
    for (IntakePosition position: IntakePosition.values())
    if (Math.abs(encoder.getAbsolutePosition().getValue() - getAngleFor(position)) < Constants.intakeAimThreshold)
      return position;
    return null;
  }
  public double getAimSpeed()
  {
    return aimMotor.get();
  }
  @Override
  public void periodic()
  {
    if (getPosition() == targetPosition)
      aimMotor.stopMotor();
    // This method will be called once per scheduler run
  }
}
