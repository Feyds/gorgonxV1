package frc.robot.controller;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Driver {
    private final PS5Controller controller;

    public Driver(int port) {
        this.controller = new PS5Controller(port);
    }
    
    public double getForwardSpeed() {
        return -controller.getLeftY(); 
    }

    public double getStrafeSpeed() {
        return -controller.getLeftX();
    }

    public double getRotationSpeed() {
        return -controller.getRightX();
    }

    public boolean isResetGyroButtonPressed() {
        return controller.getCrossButton();
    }

    public boolean isSlowMode() {
        return controller.getL1Button();
    }

    public boolean isAutoAimAndShoot() {
        return controller.getR2Axis() > 0.5;
    }

    public boolean isIntakeAndIndex() {
        return controller.getL2Axis() > 0.5;
    }

    public boolean isSpitOut() {
        return controller.getR1Button();
    }

    public void setRumble(double intensity) {
        controller.setRumble(RumbleType.kBothRumble, intensity);
    }
}