package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

// turns where the robot is (from pedro) into
// how far the hive is and where the turret should point
public class AimingHelper {

    private boolean isRed;

    // which cell of our hive is facing up right now
    private boolean farCellUp;

    public AimingHelper(boolean isRed) {
        setAlliance(isRed);
    }

    // pick red or blue. also resets the hive to how it starts the match
    public void setAlliance(boolean isRed) {
        this.isRed = isRed;
        if (isRed) {
            farCellUp = Constants.RED_FAR_CELL_STARTS_UP;
        } else {
            farCellUp = Constants.BLUE_FAR_CELL_STARTS_UP;
        }
    }

    // driver presses a button when our hive tips. the other cell is up now
    public void hiveTipped() {
        farCellUp = !farCellUp;
    }

    public boolean isRed() {
        return isRed;
    }

    public boolean isFarCellUp() {
        return farCellUp;
    }

    // x of the cell we're shooting at
    public double getTargetX() {
        if (isRed) {
            return farCellUp ? Constants.RED_FAR_CELL_X : Constants.RED_AUDIENCE_CELL_X;
        }
        return farCellUp ? Constants.BLUE_FAR_CELL_X : Constants.BLUE_AUDIENCE_CELL_X;
    }

    // y of the cell we're shooting at
    public double getTargetY() {
        if (isRed) {
            return farCellUp ? Constants.RED_FAR_CELL_Y : Constants.RED_AUDIENCE_CELL_Y;
        }
        return farCellUp ? Constants.BLUE_FAR_CELL_Y : Constants.BLUE_AUDIENCE_CELL_Y;
    }

    // where the turret is on the field
    // pedro tracks the middle of the robot, the turret might not be there
    public double getTurretX(Pose pose) {
        double heading = pose.heading();
        return pose.x()
                + Constants.TURRET_FORWARD_IN * Math.cos(heading)
                - Constants.TURRET_LEFT_IN * Math.sin(heading);
    }

    public double getTurretY(Pose pose) {
        double heading = pose.heading();
        return pose.y()
                + Constants.TURRET_FORWARD_IN * Math.sin(heading)
                + Constants.TURRET_LEFT_IN * Math.cos(heading);
    }

    // flat distance from the turret to the cell in inches
    // this is what goes into the lookup table
    public double getDistance(Pose pose) {
        double dx = getTargetX() - getTurretX(pose);
        double dy = getTargetY() - getTurretY(pose);
        return Math.hypot(dx, dy);
    }

    // which way the turret has to point, in degrees
    // 0 = straight ahead, + = left, - = right
    public double getTurretAngle(Pose pose) {
        double dx = getTargetX() - getTurretX(pose);
        double dy = getTargetY() - getTurretY(pose);

        // angle to the cell on the field, minus which way the robot faces
        double angle = Math.toDegrees(Math.atan2(dy, dx) - pose.heading());

        // keep it between -180 and 180
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
