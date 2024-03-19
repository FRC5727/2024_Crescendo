// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.IntakePosition;

public class IntakeAimCommand extends Command
{
  Intake intake;
  IntakePosition targetPosition;
  double speed;
  /** Creates a new IntakeAimCommand. */
  public IntakeAimCommand(Intake s_Intake, IntakePosition position, double speed)
  {
    this.intake = intake;
    this.targetPosition = targetPosition;
    this.speed = speed;

    addRequirements(s_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    if (intake.getPosition() == targetPosition)
      cancel();
    else if (Intake.getAngleFor(intake.getPosition()) < Intake.getAngleFor(targetPosition))
      intake.setAimSpeed(speed);
    else if (Intake.getAngleFor(intake.getPosition()) > Intake.getAngleFor(targetPosition))
      intake.setAimSpeed(-speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    intake.setAimSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getPosition() == targetPosition;
  }
}
