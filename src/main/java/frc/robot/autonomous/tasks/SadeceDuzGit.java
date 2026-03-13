package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Intake.Intake;

public class SadeceDuzGit {
    
    public static Command getTask(Drivetrain drivetrain, Intake intake) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("SadeceDuzGitRotasi");

            return new SequentialCommandGroup(
                Commands.runOnce(() -> intake.deploy()),
                AutoBuilder.followPath(path)
            );
            
        } catch (Exception e) {
            System.out.println("OTONOM HATASI: 'SadeceDuzGitRotasi' dosyasi bulunamadi!");
            e.printStackTrace();
            return Commands.none(); 
        }
    }
}