package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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

        double distance = heightDifference / Math.tan(angleToGoalRadians);
        
        return distance;
    }
}