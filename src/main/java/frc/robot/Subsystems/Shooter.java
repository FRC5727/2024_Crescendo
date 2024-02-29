// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX leftMotor, rightMotor;
  /** Creates a new Shooter. */
  public Shooter()
  {
    leftMotor = new TalonFX(Constants.leftShooterMotorPort);
    rightMotor = new TalonFX(Constants.rightShooterMotorPort);
  }

  public void setSpeed(double speed)
  {
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }

  public void stop()
  {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
