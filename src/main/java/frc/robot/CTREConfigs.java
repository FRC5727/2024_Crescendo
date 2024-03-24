package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public TalonFXConfiguration intakeAngleFXConfig = new TalonFXConfiguration(); //Possibly Angle config 
    public TalonFXConfiguration intakeAimFXConfig = new TalonFXConfiguration(); //jOSIE
    public CANcoderConfiguration intakeAimCANcoderConfig = new CANcoderConfiguration(); //MIAH

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;

        swerveAngleFXConfig.Voltage.PeakForwardVoltage = 12.0;
        swerveAngleFXConfig.Voltage.PeakReverseVoltage = -12.0;


        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;

        swerveDriveFXConfig.Voltage.PeakForwardVoltage = 12.0;
        swerveDriveFXConfig.Voltage.PeakReverseVoltage = -12.0;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Aim PID Config */
        intakeAimCANcoderConfig.MagnetSensor.SensorDirection = Constants.IntakeAim.encoderInvert;

        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.IntakeAim.motorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.IntakeAim.neutralMode;

        /* Gear Ratio and Wrapping Config */
        intakeAimFXConfig.Feedback.SensorToMechanismRatio = Constants.IntakeAim.gearRatio;
        intakeAimFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        intakeAimFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        intakeAimFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.IntakeAim.currentLimit;
        intakeAimFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.angleCurrentThreshold;
        intakeAimFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
       intakeAimFXConfig.Slot0.kP = Constants.IntakeAim.aimKP;
       intakeAimFXConfig.Slot0.kI = Constants.IntakeAim.aimKI;
       intakeAimFXConfig.Slot0.kD = Constants.IntakeAim.aimKD;

        /* Open and Closed Loop Ramping */
        intakeAimFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        intakeAimFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        intakeAimFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        intakeAimFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
    }
}