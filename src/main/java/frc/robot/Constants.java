package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.math.BetterSwerveKinematics;
import frc.lib.math.SecondOrderSwerveKinematics;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final String CANivoreName = "CANivore";

    public static final int LED_CANDLE = 0; // Port for LED light

    public static final class Controls {
        public static final double stickDeadband = 0.10; // TODO We have used 0.05 previously
        public static final double triggerAxisThreshold = 0.10; // Threshold to consider a trigger pulled
        public static final XboxController driver = new XboxController(0);
    }

    /* Climber Motor PID Values */
        public static final double climberKP = 1;
        public static final double climberKI = 0;
        public static final double climberKD = 0;

    public static final double minimumClimberHeight = 0; // Minimum during retraction mode
    public static final double maxClimberHeight = 25.4; // CM
    public static final double rotationsPerCM = 100 / maxClimberHeight;
    public static final class Swerve {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(19.625); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(23.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        // Multipliers when speed limit is in effect;
        public static final double speedLimitXY = 0.45;
        public static final double speedLimitRot = 0.15;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.04; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
    
      
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(driveKP, driveKI, driveKD, driveKF), // Translation constants 
      new PIDConstants(angleKP, angleKI, angleKD), // Rotation constants 
      //new PIDConstants(aimKP, aimKI, aimKD), //Aim Constants
      maxSpeed, 
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0).getNorm(),
      //flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(121.904297);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(133.066406);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-158.818359);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-89.824219);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }



/* Back Right Module - Module 3 */
public static final class Mod3 {
  public static final int driveMotorID = 4;
  public static final int angleMotorID = 5;
  public static final int canCoderID = 2;
  public static final Rotation2d angleOffset = Rotation2d.fromDegrees(237.041016);
  public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
      canCoderID, angleOffset);
  }

  public static final class Vision {
    public static final String limelightName = "limelight";
    public static final double maxXYError = 1.0;

    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 10},
      {1.5, 0.01, 0.01, 10},
      {3, 0.145, 1.20, 30},
      {4.5, 0.75, 5.0, 90},
      {6, 1.0, 8.0, 180}
    };

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 5},
      {1.5, 0.02, 0.02, 5},
      {3, 0.04, 0.04, 15},
      {4.5, 0.1, 0.1, 30},
      {6, 0.3, 0.3, 60}
    };
  }

  public static int leftClimberMotorPort = 8;
  public static int rightClimberMotorPort = 9;

  public static int leftShooterMotorPort = 10;
  public static int rightShooterMotorPort = 11;

  public static int intakeAimMotorPort = 12;
  public static int intakePullMotorPort = 13;
  public static int intakeEncoderPort = 4;
  public static int intakeSensorPort = 0;
  public static int intakeLimitPort = 9;
  public static int feedLimitPort = 4;

  public static double shooterSpeakerSpeed = 1.0;
  public static double shooterAmpSpeed = 0.15;
  public static double shooterSpeedThreshold = 0.02;
  public static double intakeShootSpeed = 1.0;
  public static double intakeIntakeSpeed = 0.4;
  public static double spinupDurationMS = 2000.0; // Milliseconds
  public static double fireDurationMS = 500.0; // Milliseconds

  public static class IntakeAim
  {
    public static SensorDirectionValue encoderInvert = SensorDirectionValue.CounterClockwise_Positive;
    public static InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
    public static NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static double threshold = 0.01; // In rotations
    public static double gearRatio = 220.;

    public static double currentLimit = 35;
    /*Aim Motor Josie PID */
    public static final double aimKP = 0.04; // Josie Added instead of non-static PID
    public static final double aimKI = 0.00;
    public static final double aimKD = 0.00;  

    // Not using above PID For now

    public static final double speed = 0.50;
  }

  public static class intakeAngles
  {
    public static double intake = -0.02;
    public static double feed = 0.489932;
 //   public static double fireAmp = 0.0;
 //   public static double fireSpeaker = 0.0;
  }

  
  public static final int canCoderID = 4;
  /*public PIDConstants intakeConstants = new PIDConstants( // PID
    1.0, // P
    0.0, // I
    0.0  // D
  );*/

  public static final double climberSpeed = 0.80;
}