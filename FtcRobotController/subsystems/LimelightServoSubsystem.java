package org.firstinspires.ftc.teamcode.subsystems; // Declares the package folder where this subsystem class resides

import com.qualcomm.robotcore.hardware.HardwareMap; // Imports FTC HardwareMap class used to look up and initialize hardware
import com.qualcomm.robotcore.hardware.Servo; // Imports FTC Servo class to control servo motors for tilting

public class LimelightServoSubsystem { // Defines the public subsystem class managing the Limelight camera tilt servo

    private final Servo servo; // Reference variable for the physical servo motor controlling camera pitch
    private static final double POSITION_UP = 0.75; // Servo position value (0.0 to 1.0) pointing the camera upward toward the hive AprilTags
    private static final double POSITION_DOWN = 0.25; // Servo position value pointing the camera downward toward the floor for color/nectar tracking
    private boolean isUp = false; // Internal tracking flag indicating whether the camera is currently facing upward

    public LimelightServoSubsystem(HardwareMap hardwareMap) { // Constructor executed when initializing the subsystem
        servo = hardwareMap.get(Servo.class, "limelight_servo"); // Maps the physical servo hardware device named "limelight_servo"
    } // Closes constructor

    public void servoUp() { // Public command method to tilt the camera upward
        moveToPosition(POSITION_UP); // Commands the servo to move to the upper target position
    } // Closes servoUp method

    public void servoDown() { // Public command method to tilt the camera downward
        moveToPosition(POSITION_DOWN); // Commands the servo to move to the lower target position
    } // Closes servoDown method

    public void toggle() { // Public method to flip the camera position between up and down states
        if (isUp) { // Checks if the camera is currently tilted up
            servoDown(); // If it is up, tilt it down
        } else { // Otherwise, if it is down
            servoUp(); // Tilt it up
        } // Closes conditional toggle block
    } // Closes toggle method

    public void moveToPosition(double position) { // Core helper method to safely command and clamp servo positions
        double targetPos = Math.max(0.0, Math.min(1.0, position)); // Clamps the requested position between valid hardware limits 0.0 and 1.0
        if (servo != null) { // Null safety check to prevent runtime crashes if hardware isn't mapped
            servo.setPosition(targetPos); // Sends the movement command to the physical servo motor
        } // Closes null check block
        isUp = Math.abs(targetPos - POSITION_UP) < Math.abs(targetPos - POSITION_DOWN); // Updates the boolean state based on proximity to the up position
    } // Closes moveToPosition method

    public double getServoPosition() { // Public getter method to check the current position of the servo
        return servo != null ? servo.getPosition() : 0.0; // Returns current position if initialized, otherwise returns 0.0 safely
    } // Closes getServoPosition method

    public boolean isServoUp() { // Public getter method to check if the camera is currently in the upward orientation
        return isUp; // Returns the boolean status of the isUp flag
    } // Closes isServoUp method

} // Closes LimelightServoSubsystem class
