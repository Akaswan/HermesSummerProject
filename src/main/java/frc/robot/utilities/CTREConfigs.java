package frc.robot.utilities;

import static frc.robot.utilities.Constants.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            ANGLE_ENABLE_CURRENT_LIMIT, 
            ANGLE_CONTINUOUS_CURRENT_LIMIT, 
            ANGLE_PEAK_CURRENT_LIMIT, 
            ANGLE_PEAK_CURRENT_DURATION);

        swerveAngleFXConfig.slot0.kP = ANGLE_KP;
        swerveAngleFXConfig.slot0.kI = ANGLE_KI;
        swerveAngleFXConfig.slot0.kD = ANGLE_KD;
        swerveAngleFXConfig.slot0.kF = ANGLE_KF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            DRIVE_ENABLE_CURRENT_LIMIT, 
            DRIVE_CONTINUOUS_CURRENT_LIMIT, 
            DRIVE_PEAK_CURRENT_LIMIT, 
            DRIVE_PEAK_CURRENT_DURATION);

        swerveDriveFXConfig.slot0.kP = DRIVE_KP;
        swerveDriveFXConfig.slot0.kI = DRIVE_KI;
        swerveDriveFXConfig.slot0.kD = DRIVE_KD;
        swerveDriveFXConfig.slot0.kF = DRIVE_KF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp = CLOSED_LOOP_RAMP;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = CAN_CODER_INVERT;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}