package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.tasks.MainOtonom;
import frc.robot.autonomous.tasks.SadeceDuzGit;

import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class AutoChooser {
    private final SendableChooser<Command> chooser = new SendableChooser<>();

    public AutoChooser(Drivetrain drivetrain, Shooter shooter, Feeder feeder, Intake intake, Vision vision) {
        chooser.setDefaultOption("Sadece Düz Git", SadeceDuzGit.getTask(drivetrain, intake));

        chooser.addOption("Ana Otonom (Trench ve Outpost)", MainOtonom.getTask(drivetrain, shooter, feeder, intake, vision));
        chooser.addOption("Düz git", SadeceDuzGit.getTask(drivetrain, intake));

        SmartDashboard.putData("Otonom Secimi", chooser);
    }

    public Command getSelectedTask() {
        return chooser.getSelected();
    }
}