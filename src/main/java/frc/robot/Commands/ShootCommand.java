// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Feed the shooter from the intake

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Intake.IntakePosition;

public class ShootCommand extends Command
{
  private Intake intake;
  private Shooter shooter;
  private int step; // Which step of the process we're on
  // step 0 - start aiming
  // step 1 - wait for motor to reach position
  // step 2 - fire
  // step 3 - done
  private double startTime;
  private double targetSpeed;
  
  /** Creates a new ShooterFeedCommand. */
  public ShootCommand(Intake intake, Shooter shooter, double targetSpeed)
  {
    this.intake = intake;
    this.shooter = shooter;
    this.targetSpeed = targetSpeed;
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    step = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    switch (step)
    {
      case 0: // Moving intake
        if (intake.getPosition() != IntakePosition.feed)
          intake.moveTo(IntakePosition.feed);
        shooter.setSpeed(-targetSpeed);
        step = 1;
        break;
      case 1: // Waiting to spin up
        if (
          (Math.abs(shooter.getSpeed() - (-targetSpeed)) < Constants.shooterSpeedThreshold) &&
          (intake.getPosition() == IntakePosition.feed))
        {
          intake.setIntakeMotor(-Constants.intakeShootSpeed);
          step = 2;
          startTime = System.currentTimeMillis();
        }
        break;
      case 2: // Waiting to finish
      {
        if (System.currentTimeMillis() >= startTime + Constants.fireDurationMS) // Done firing
        {
          intake.setIntakeMotor(0);
          shooter.setSpeed(0);
          step = 3;
        }
        break;
      }
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    intake.setIntakeMotor(0);
    shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return (step == 3);
  }
}
