// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Climber;
public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  static final double kBalanceAngleThresholdDegrees = 2;
  Climber climber;
  public ClimberCommand(Climber climber)
  {
    this.climber = climber;
  }

public void operatorControl(Translation2d translation,) {
  //maybe add more like translation2d to translation, eg for gyroscope. their gyro is ahrs or smth
    while (isOperatorControl() && isEnabled()) {

        double xAxisRate            = translation.getX(); 
        double yAxisRate            = translation.getY();
        double rollAngleDegrees    = climber.getTilt();
        //was stick and ahrs
        boolean autoBalanceXMode;
        boolean autoBalanceYmode;

        if double rollAngleDegrees = 0; 
        if ( !autoBalanceXMode && 
             (Math.abs(rollAngleDegrees) >= 
              Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = true;
        }
        else if ( autoBalanceXMode && 
                  (Math.abs(rollAngleDegrees) <= 
                   Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = false;
        }
        if ( !autoBalanceYMode && 
             (Math.abs(rollAngleDegrees) >= 
              Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = true;
        }
        else if ( autoBalanceYMode && 
                  (Math.abs(rollAngleDegrees) <= 
                   Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = false;
        }
        
        // Control drive system automatically, 
        // driving in reverse direction of pitch/roll angle,
        // with a magnitude based upon the angle
        
        if ( autoBalanceXMode ) {
            double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
            xAxisRate = Math.sin(pitchAngleRadians) * -1;
        }
      
        }
    // Use addRequirements() here to declare subsystem dependencies.
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
}