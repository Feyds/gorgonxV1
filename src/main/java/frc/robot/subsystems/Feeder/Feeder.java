package frc.robot.subsystems.Feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Feeder {
    private final SparkMax centerMotor = new SparkMax(FeederConstants.CENTER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(FeederConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(FeederConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    public Feeder() {
        SparkMaxConfig centerConfig = new SparkMaxConfig();
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();

        centerConfig.idleMode(IdleMode.kBrake);
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);
        rightConfig.inverted(true); 

        centerMotor.configure(centerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void feedToShooter() {
        centerMotor.set(FeederConstants.FEED_SPEED);
        leftMotor.set(FeederConstants.FEED_SPEED);
        rightMotor.set(FeederConstants.FEED_SPEED);
    }

    public void indexSlowly() {
        centerMotor.set(FeederConstants.INDEX_SPEED);
        leftMotor.set(FeederConstants.INDEX_SPEED);
        rightMotor.set(FeederConstants.INDEX_SPEED);
    }

    public void spitOut() {
        centerMotor.set(FeederConstants.REVERSE_SPEED);
        leftMotor.set(FeederConstants.REVERSE_SPEED);
        rightMotor.set(FeederConstants.REVERSE_SPEED);
    }

    public void stop() {
        centerMotor.set(0);
        leftMotor.set(0);
        rightMotor.set(0);
    }
}