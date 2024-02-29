// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public static enum IntakePosition
  {
    intake,
    feed,
    fireAmp,
    fireSpeaker,
  }

  /** Creates a new Intake. */
  public Intake() {}

  public void moveTo(IntakePosition position)
  {
    switch (position)
    {
      case intake: // Move to intake position
      break;
      case feed: // Move to shooter-feeding position
      break;
      case fireAmp: // Move to firing position for amp
      break;
      case fireSpeaker: // Move to firing position for speaker
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
