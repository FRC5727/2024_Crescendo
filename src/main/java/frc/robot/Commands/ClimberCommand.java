// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Climber;
public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  static final double kBalanceAngleThresholdDegrees = 2;
  static final double speed = 0.1;
  private Climber climber;
  private int mode; // 0 -> start, 1 -> extend, 2 -> retract & level

  public ClimberCommand(Climber climber)
  {
    this.climber = climber;
    this.mode = 0;
    addRequirements(climber);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Get the current height of the left arm
  public double leftArmHeight()
  //josie goofing below
  
  {
    return 0;
  }
  // Get the current height of the left arm
  public double rightArmHeight()
  {
    return 0;
  }
  public void setMode(int mode)
  {
    this.mode = mode;
  }
  public void toggleMode()
  {
    mode += 1;
    if (mode == 3)
      mode = 0;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    switch (mode)
    {
    case 0: // Retract maximally, for startup
      if (Math.min(leftArmHeight(), rightArmHeight()) > 0)
      // Not far enough retracted, retract both equally
      {
        climber.move(-speed);
      } 
      break;
    case 1: // Extend maximally
    if (Math.min(leftArmHeight(), rightArmHeight()) < Constants.maxClimberHeight)
      // Not far enough retracted, retract both equally
      {
        climber.move(-speed);
      } 
      break;
    case 2: // Extend and level
        double rollAngleDegrees = climber.getTilt();

      if (Math.min(leftArmHeight(), rightArmHeight()) > Constants.minimumClimberHeight)
      // Not far enough retracted, retract both equally
      {

      }
      else if (rollAngleDegrees > kBalanceAngleThresholdDegrees)
      // Tilted to the left, retract the left arm more
      {
        
      }
      else if (rollAngleDegrees < -kBalanceAngleThresholdDegrees)
      // Tilted to the right, retract the right arm more
      {

      }
      else // Done
        climber.move(0);
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}