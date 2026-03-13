package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Vision.Vision;

public class MainOtonom {
    
    public static Command getTask(Drivetrain drivetrain, Shooter shooter, Feeder feeder, Intake intake, Vision vision) {

        NamedCommands.registerCommand("IntakeCalistir", 
            Commands.runOnce(() -> {
                intake.deploy();
                intake.setRollerPower(1.0);
                feeder.indexSlowly();
            })
        );

        NamedCommands.registerCommand("OtomatikAtesEt", 
            Commands.run(() -> {
                vision.setPipeline(0);
                double distance = vision.getDistanceToTargetMeters();
                
                shooter.setRPM(distance);

                if (shooter.isAtTargetRPM(distance) && vision.hasTarget()) {
                    feeder.feedToShooter();
                } else {
                    feeder.stop();
                }
            }).withTimeout(2.5)
        );

        NamedCommands.registerCommand("SistemleriKapat", 
            Commands.runOnce(() -> {
                intake.setRollerPower(0.0);
                feeder.stop();
                shooter.stop();
            })
        );

        try {
            return AutoBuilder.buildAuto("MainOtonom");
            
        } catch (Exception e) {
            System.out.println("OTONOM HATASI: 'MainOtonom.auto' bulunamadi!");
            e.printStackTrace();
            return Commands.none();
        }
    }
}