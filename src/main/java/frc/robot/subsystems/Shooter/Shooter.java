package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Shooter {
    private final TalonFX mainMotor = new TalonFX(ShooterConstants.MAIN_KRAKEN_ID);
    private final TalonFX followerMotor = new TalonFX(ShooterConstants.FOLLOWER_KRAKEN_ID);

    private final SparkMax kickerMotor = new SparkMax(ShooterConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    public Shooter() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        shooterConfig.Slot0.kP = ShooterConstants.kP;
        shooterConfig.Slot0.kI = ShooterConstants.kI;
        shooterConfig.Slot0.kD = ShooterConstants.kD;
        shooterConfig.Slot0.kV = ShooterConstants.kV;

        mainMotor.getConfigurator().apply(shooterConfig);

        followerMotor.setControl(new Follower(ShooterConstants.MAIN_KRAKEN_ID, MotorAlignmentValue.Opposed));

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.idleMode(IdleMode.kCoast); 
        
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rpmMap.put(1.5, 3000.0);
        rpmMap.put(3.0, 4200.0);
        rpmMap.put(5.0, 5500.0);
    }

    public void setRPM(double distanceMeters) {
        double targetRPM = rpmMap.get(distanceMeters);

        double targetRPS = targetRPM / 60.0;
        
        mainMotor.setControl(velocityRequest.withVelocity(targetRPS));

        kickerMotor.set(ShooterConstants.KICKER_POWER);
    }

    public void stop() {
        mainMotor.setControl(velocityRequest.withVelocity(0));
        kickerMotor.set(0);
    }

    public boolean isAtTargetRPM(double distanceMeters) {
        double targetRPS = rpmMap.get(distanceMeters) / 60.0;
        double currentRPS = mainMotor.getVelocity().getValueAsDouble();

        return Math.abs(targetRPS - currentRPS) < ShooterConstants.RPS_TOLERANCE; 
    }
}