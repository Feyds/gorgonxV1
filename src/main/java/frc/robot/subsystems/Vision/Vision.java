package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class Vision {
    private final NetworkTable limelightTable;

    public Vision() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getTx() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    public int getTargetID() {
        return (int) limelightTable.getEntry("tid").getDouble(-1.0);
    }

    public void setPipeline(int pipelineIndex) {
        limelightTable.getEntry("pipeline").setNumber(pipelineIndex);
    }

    public int getPipeline() {
        return (int) limelightTable.getEntry("pipeline").getDouble(0.0);
    }
    
    public double getDistanceToTargetMeters() {
        if (!hasTarget()) {
            return 0.0;
        }

        double targetOffsetAngle_Vertical = getTy();
        double angleToGoalDegrees = VisionConstants.MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double heightDifference = VisionConstants.TARGET_HEIGHT_METERS - VisionConstants.LENS_HEIGHT_METERS;
        
        return heightDifference / Math.tan(angleToGoalRadians);
    }

    public double[] getBotPoseWpiBlue() {
        return limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    public double getLatencySeconds() {
        double tl = limelightTable.getEntry("tl").getDouble(0.0);
        double cl = limelightTable.getEntry("cl").getDouble(0.0);
        return (tl + cl) / 1000.0;
    }

    public boolean isValidHubTarget() {
        if (!hasTarget()) {
            return false;
        }

        int currentTargetID = getTargetID();
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                return (currentTargetID == 5 || currentTargetID == 8 || currentTargetID == 9 || currentTargetID == 10 || currentTargetID == 11 || currentTargetID == 2);
            } else {
                return (currentTargetID == 21 || currentTargetID == 24 || currentTargetID == 25 || currentTargetID == 26 || currentTargetID == 27 || currentTargetID == 18);
            }
        }
        
        return true; // TODO: ATÖLYE TESTİ İÇİN TRUE YAPILDI BUNU MAÇ ÖNCESİ DEĞİŞTİRMELİSİN!!!!!!!!!!!!!!!!
    }
}