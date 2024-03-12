// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private TalonFX leftClimberMotor;
  private TalonFX rightClimberMotor;

  private double maxRotations;
  private double currentRotationsL, currentRotationsR;

  private Pigeon2 gyro;

  /** Creates a new ClimberSubsystem. */
  public Climber() {
    leftClimberMotor = new TalonFX(Constants.leftClimberMotorPort);
    rightClimberMotor = new TalonFX(Constants.rightClimberMotorPort);
    CANcoder coder;

    gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANivoreName);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.setYaw(0);

    maxRotations = 0;
    currentRotationsL = 0;
    currentRotationsR = 0;

    leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getTilt() {
    return gyro.getYaw().getValueAsDouble();
  }

  public void move(double speed){
    leftClimberMotor.setControl(new DutyCycleOut(speed * 1.0));
    rightClimberMotor.setControl(new DutyCycleOut(speed * -1.0));
    leftClimberMotor.getStatorCurrent();
    rightClimberMotor.getStatorCurrent();
  }

  public void moveLeft(double speed)
  {
    leftClimberMotor.setControl(new DutyCycleOut(speed * 1.0));
  }
  public void moveRight(double speed){
    rightClimberMotor.setControl(new DutyCycleOut(speed * -1.0));
  }
  public void resetLeft(){
    leftClimberMotor.setControl(new DutyCycleOut(-0.15));
  }  

  public void resetRight(){
    rightClimberMotor.setControl(new DutyCycleOut(0.15));
  }  

  public void stopLeft(){
    leftClimberMotor.setControl(new DutyCycleOut( 0.0));
  }

  public void stopRight(){
    rightClimberMotor.setControl(new DutyCycleOut( 0.0));
  }

  public void stop(){
    stopLeft();
    stopRight();
  }

  @Override
  public void periodic() {
  }
    
}
