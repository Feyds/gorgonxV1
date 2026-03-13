package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    public final String moduleName;
    private final TalonFX driveMotor;
    private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;
    private final CANcoder absoluteEncoder;
    private final double angleOffset;

    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    private double simDrivePosition = 0.0;
    private double simAngleRad = 0.0;
    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    public SwerveModule(String name, int driveId, int angleId, int cancoderId, double offset) {
        this.moduleName = name;
        this.angleOffset = offset;

        driveMotor = new TalonFX(driveId);
        angleMotor = new SparkMax(angleId, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(cancoderId);

        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        configureDevices();
        resetToAbsolute();
    }

    private void configureDevices() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.Slot0.kP = SwerveConstants.DRIVE_P;
        driveConfig.Slot0.kI = SwerveConstants.DRIVE_I;
        driveConfig.Slot0.kD = SwerveConstants.DRIVE_D;

        driveConfig.Slot0.kV = SwerveConstants.DRIVE_V; 
        driveConfig.Slot0.kS = SwerveConstants.DRIVE_S;

        driveConfig.CurrentLimits.SupplyCurrentLimit = 60;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = 80;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        driveMotor.getConfigurator().apply(driveConfig);

        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig.encoder.positionConversionFactor(SwerveConstants.ANGLE_POS_FACTOR);
        angleConfig.inverted(true);
        angleConfig.smartCurrentLimit(30);
        angleConfig.closedLoop.pid(SwerveConstants.ANGLE_P, SwerveConstants.ANGLE_I, SwerveConstants.ANGLE_D);
        angleConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        angleConfig.closedLoop.positionWrappingEnabled(true);
        angleConfig.closedLoop.positionWrappingInputRange(0, 2 * Math.PI);
        angleConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void resetToAbsolute() {
        double absolutePosition = getAbsoluteEncoderRad();
        angleEncoder.setPosition(absolutePosition);
    }

    private double getAbsoluteEncoderRad() {
        if (RobotBase.isSimulation()) {
            return simAngleRad;
        }
        double rotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        double angleRad = Units.rotationsToRadians(rotations);
        return angleRad - angleOffset;
    }

    public double getRawAbsoluteEncoderRad() {
        double rotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        return Units.rotationsToRadians(rotations);
    }

    public SwerveModuleState getState() {
        if (RobotBase.isSimulation()) {
            return new SwerveModuleState(lastDesiredState.speedMetersPerSecond, new Rotation2d(simAngleRad));
        }

        double velocityRPS = driveMotor.getVelocity().getValueAsDouble();
        double velocityMps = velocityRPS * SwerveConstants.DRIVE_POS_FACTOR;

        return new SwerveModuleState(velocityMps, new Rotation2d(angleEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        if (RobotBase.isSimulation()) {
            return new SwerveModulePosition(simDrivePosition, new Rotation2d(simAngleRad));
        }

        double positionRotations = driveMotor.getPosition().getValueAsDouble();
        double positionMeters = positionRotations * SwerveConstants.DRIVE_POS_FACTOR;

        return new SwerveModulePosition(positionMeters, new Rotation2d(angleEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = new Rotation2d(angleEncoder.getPosition());

        if (RobotBase.isSimulation()) {
            currentRotation = new Rotation2d(simAngleRad);
        }

        desiredState.optimize(currentRotation);
        this.lastDesiredState = desiredState;

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }

        double velocityRPS = desiredState.speedMetersPerSecond / SwerveConstants.DRIVE_POS_FACTOR;

        driveMotor.setControl(driveVelocity.withVelocity(velocityRPS));
        anglePID.setSetpoint(desiredState.angle.getRadians(), com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }

    public void stop() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    public void simulationPeriodic(double dt) {
        simDrivePosition += lastDesiredState.speedMetersPerSecond * dt;
        simAngleRad = lastDesiredState.angle.getRadians();
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber(moduleName + " RAW Encoder (Rad)", getRawAbsoluteEncoderRad());
        SmartDashboard.putNumber(moduleName + " Integrated Angle", angleEncoder.getPosition());
        SmartDashboard.putNumber(moduleName + " Velocity", getState().speedMetersPerSecond);
    }
}