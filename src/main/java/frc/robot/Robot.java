// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.filter.SlewRateLimiter;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.AutoRunner;
import frc.robot.controller.Driver;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private AutoChooser autoChooser;
  AutoRunner autoRunner;

  private Drivetrain drivetrain;
  private Driver driver;
  private Shooter shooter;
  private Feeder feeder;
  private Intake intake;
  private Vision vision;

  private final Field2d field = new Field2d();
  private final PIDController aimPID = new PIDController(0.04, 0.0, 0.001);

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  double lastKnownTx = 0.0;
  private double lastKnownDistance = 3.0;
  private final Timer targetLostTimer = new Timer();

  @Override
  public void robotInit() {
    drivetrain = new Drivetrain();
    driver = new Driver(0);
    shooter = new Shooter();
    feeder = new Feeder();
    intake = new Intake();
    vision = new Vision();

    autoRunner = new AutoRunner(drivetrain);
    autoChooser = new AutoChooser(drivetrain, shooter, feeder, intake, vision);
    aimPID.setTolerance(1.0);

    SmartDashboard.putData("Saha (Field2d)", field);

    targetLostTimer.start();
  }

  @Override
  public void robotPeriodic() {
    drivetrain.periodic(); 

    SmartDashboard.putBoolean("Hedef Goruldu", vision.hasTarget());
    SmartDashboard.putNumber("Mesafe (Metre)", vision.getDistanceToTargetMeters());
    SmartDashboard.putBoolean("Shooter Hazir", shooter.isAtTargetRPM(vision.getDistanceToTargetMeters()));

    String gameData = DriverStation.getGameSpecificMessage();
    Optional<Alliance> alliance = DriverStation.getAlliance();

    boolean hub = false; 

    if (alliance.isPresent() && gameData != null && !gameData.isEmpty()) {
        char activeHub = gameData.charAt(0);

        if (alliance.get() == Alliance.Red && activeHub == 'R') {
            hub = true;
        } else if (alliance.get() == Alliance.Blue && activeHub == 'B') {
            hub = true;
        }
    }
    SmartDashboard.putBoolean("Hub Durumumuz", hub);
  }

  @Override
  public void disabledInit() {
    drivetrain.stopModules();
    shooter.stop();
    feeder.stop();
    intake.stop();
    intake.retract();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelectedTask();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().cancelAll();

    intake.deploy();
  }

  @Override
  public void teleopPeriodic() {
    boolean hasTarget = vision.hasTarget();
    double currentTx = vision.getTx();
    double currentDistance = vision.getDistanceToTargetMeters();

    if (hasTarget) {
        lastKnownTx = currentTx;
        lastKnownDistance = currentDistance;
        targetLostTimer.restart();
    }

    double xSpeed = driver.getForwardSpeed();
    double ySpeed = driver.getStrafeSpeed();
    double rot = driver.getRotationSpeed();

    xSpeed = Math.abs(xSpeed) > 0.1 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > 0.1 ? ySpeed : 0.0;
    rot = Math.abs(rot) > 0.1 ? rot : 0.0;

    double maxSpeed = SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    double maxRot = SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    
    if (driver.isSlowMode()) {
        maxSpeed /= 2;
        maxRot /= 2;
    }

    if (driver.isAutoAimAndShoot()) {
        vision.setPipeline(0);

        if (hasTarget || !targetLostTimer.hasElapsed(1.0)) {

            if (hasTarget) {
                rot = aimPID.calculate(currentTx, 0);
            } else {
                rot = 0.0;
            }

            shooter.setRPM(lastKnownDistance);

            if (shooter.isAtTargetRPM(lastKnownDistance) && (hasTarget ? aimPID.atSetpoint() : true)) {
                feeder.feedToShooter();
                driver.setRumble(1.0);
            } else {
                feeder.stop();
                driver.setRumble(0.2);
            }
        } else {
            shooter.stop();
            feeder.stop();
            driver.setRumble(0);
        }
    } else if (driver.isIntakeAndIndex()) {
        intake.setRollerPower(1.0);
        feeder.indexSlowly();
        shooter.stop();
    } else if (driver.isSpitOut()) {
        intake.setRollerPower(-1.0);
        feeder.spitOut();
        shooter.stop();
    } else {
        intake.setRollerPower(0.0);
        feeder.stop();
        shooter.stop();
        driver.setRumble(0.0);
    }

    double finalX = xLimiter.calculate(xSpeed) * maxSpeed;
    double finalY = yLimiter.calculate(ySpeed) * maxSpeed;
    double finalRot = rotLimiter.calculate(rot) * maxRot;

    if (driver.isResetGyroButtonPressed()) {
        drivetrain.resetGyro();
    }

    drivetrain.drive(finalX, finalY, finalRot, true);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}