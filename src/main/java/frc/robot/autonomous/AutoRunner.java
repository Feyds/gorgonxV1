package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain.Drivetrain;

public class AutoRunner {
    private Command currentTask;
    public AutoRunner(Drivetrain drivetrain) {

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return; 
        }

        AutoBuilder.configure(
            drivetrain::getPose,          
            drivetrain::resetOdometry,    
            drivetrain::getChassisSpeeds, 
            (speeds, feedforwards) -> drivetrain.drive(
                speeds.vxMetersPerSecond, 
                speeds.vyMetersPerSecond, 
                speeds.omegaRadiansPerSecond, 
                false 
            ),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), 
                new PIDConstants(5.0, 0.0, 0.0)  
            ),
            config, 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            drivetrain
        );
    }

    public void runTask(Command selectedTask) {
        currentTask = selectedTask;
        if (currentTask != null) {
            currentTask.schedule();
        }
    }

    public void stopTask() {
        if (currentTask != null) {
            currentTask.cancel();
        }
    }
}