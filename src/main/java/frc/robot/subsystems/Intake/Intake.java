package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Intake {
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
    private final SparkMax rollerMotor = new SparkMax(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0).withSlot(0);

    public Intake() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;

        pivotConfig.Slot0.kP = IntakeConstants.kP;
        pivotConfig.Slot0.kI = IntakeConstants.kI;
        pivotConfig.Slot0.kD = IntakeConstants.kD;
        pivotConfig.Slot0.kG = IntakeConstants.kG;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; 

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.CRUISE_VELOCITY;
        pivotConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.ACCELERATION;
        pivotConfig.MotionMagic.MotionMagicJerk = IntakeConstants.JERK;

        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(IntakeConstants.RETRACT_POSITION);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast); 
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void deploy() {
        pivotMotor.setControl(pivotRequest.withPosition(IntakeConstants.DEPLOY_POSITION));
    }

    public void retract() {
        pivotMotor.setControl(pivotRequest.withPosition(IntakeConstants.RETRACT_POSITION));
    }

    public void setRollerPower(double power) {
        rollerMotor.set(power);
    }

    public void stop() {
        rollerMotor.set(0);
    }
}