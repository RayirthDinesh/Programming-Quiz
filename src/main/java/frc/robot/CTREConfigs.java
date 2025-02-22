
package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.CANCODER_INVERT;

        
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.ANGLE_MOTOR_INVERT;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.ANGLE_NEUTRAL_MODE;

       
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.ANGLE_GEAR_RATIO;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
      
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.ANGLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.SwerveConstants.ANGLE_CURRENT_THRESHOLD;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.SwerveConstants.ANGLE_CURRENT_THRESHOLD_TIME;

       
        swerveAngleFXConfig.Slot0.kP = Constants.SwerveConstants.ANGLE_KP;
        swerveAngleFXConfig.Slot0.kI = Constants.SwerveConstants.ANGLE_KI;
        swerveAngleFXConfig.Slot0.kD = Constants.SwerveConstants.ANGLE_KD;

      
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.DRIVE_MOTOR_INVERT;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.DRIVE_NEUTRAL_MODE;

        
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.DRIVE_GEAR_RATIO;

        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.DRIVE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.SwerveConstants.DRIVE_CURRENT_THRESHOLD;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.SwerveConstants.DRIVE_CURRENT_THRESHOLD_TIME;

        swerveDriveFXConfig.Slot0.kP = Constants.SwerveConstants.DRIVE_KP;
        swerveDriveFXConfig.Slot0.kI = Constants.SwerveConstants.DRIVE_KI;
        swerveDriveFXConfig.Slot0.kD = Constants.SwerveConstants.DRIVE_KD;

       
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.SwerveConstants.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.SwerveConstants.CLOSED_LOOP_RAMP;
    }
}