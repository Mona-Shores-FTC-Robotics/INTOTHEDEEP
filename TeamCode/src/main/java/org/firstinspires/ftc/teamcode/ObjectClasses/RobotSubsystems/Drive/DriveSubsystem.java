package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveClasses.MecanumDriveMona;

public class DriveSubsystem extends SubsystemBase {

    @Config
    public static class DriveParameters {
        /** Set these drive parameters for faster TeleOp driving**/
        public static double DRIVE_SPEED_FACTOR=.7;
        public static double STRAFE_SPEED_FACTOR=.7;
        public static double TURN_SPEED_FACTOR=.8;
        public static double DEAD_ZONE = .2;
    }

    // Constants for motor ticks, wheel diameter, and gear ratio
    private static final double TICKS_PER_REVOLUTION = 28;  // Example: REV HD Hex motor
    private static final double WHEEL_DIAMETER_INCHES = 4.0;  // Example wheel diameter in inches
    private static final double GEAR_RATIO = 1.0;  // Adjust if you have a gear ratio

    // Calculating wheel circumference
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;

    public MecanumDriveMona mecanumDrive;

    public boolean fieldOrientedControl;

    public double yawOffset;  // offset depending on alliance color

    public double drive;
    public double strafe;
    public double turn;

    public double leftYAdjusted;
    public double leftXAdjusted;
    public double rightXAdjusted;

    public DriveSubsystem(HardwareMap hardwareMap) {
        //Make the mecanumDrive at pose 0,0,0 because we don't know where we are yet
        mecanumDrive = new MecanumDriveMona(hardwareMap, new Pose2d(0, 0, 0));
    }

    public void init()
    {
        Robot.getInstance().registerSubsystem(Robot.SubsystemType.DRIVE);
        fieldOrientedControl=false; // Default to non-field-oriented
        mecanumDrive.init();

        // Calculate yaw offset based on alliance color
        // This offset assumes robot forward direction faces away from driver
        //      -For red start, the audience is on your left
        //      -For blue start, the audience is on your right
        //  See this: https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html

        if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
            yawOffset = Math.toRadians(90);  // 90 degrees for blue side
        } else {
            yawOffset = Math.toRadians(-90);  // -90 degrees for red side
        }
    }

    public MecanumDriveMona getMecanumDrive()
    {
       return mecanumDrive;
    }

    /**
     * Adjust the drive, strafe, and turn values for robot-centric or field-centric control
     */
    public void setDriveStrafeTurnValues(double leftY, double leftX, double rightX) {
        boolean gamepadActive = driverGamepadIsActive(leftY, leftX, rightX);
        if (gamepadActive) {
            if (fieldOrientedControl) {
                fieldOrientedControl(leftY, leftX);
                rightXAdjusted = rightX * DriveParameters.TURN_SPEED_FACTOR;
            } else {
                // Apply speed factors
                leftYAdjusted = leftY * DriveParameters.DRIVE_SPEED_FACTOR;
                leftXAdjusted = leftX * DriveParameters.STRAFE_SPEED_FACTOR;
                rightXAdjusted = rightX * DriveParameters.TURN_SPEED_FACTOR;
            }
        } else {
            // No input, set adjusted values to 0
            leftYAdjusted = 0;
            leftXAdjusted = 0;
            rightXAdjusted = 0;
        }

        // Set drive, strafe, and turn values
        drive = leftYAdjusted;
        strafe = leftXAdjusted;
        turn = rightXAdjusted;

        // Pass these values to the Mecanum drive system
        mecanumDrive.mecanumDriveSpeedControl(drive, strafe, turn);
    }

    /**
     * Implements field-centric control. Adjusts the robot's movement based on the heading.
     */
    public void fieldOrientedControl(double y, double x) {
        // Get the robot's current heading in radians (directly from the gyro)
        double botHeading = Math.toRadians(getYawDegrees());

        // Adjust the robot's heading by the stored offset
        double adjustedHeading = botHeading - yawOffset;

        // Rotate the movement direction relative to the robot's adjusted heading
        leftXAdjusted = x * Math.cos(-adjustedHeading) - y * Math.sin(-adjustedHeading);
        leftYAdjusted = x * Math.sin(-adjustedHeading) + y * Math.cos(-adjustedHeading);

        // Optionally counteract imperfect strafing
        leftYAdjusted = Math.min(leftYAdjusted * 1.1, 1);
    }

    // Display basic telemetry for Driver Station
    @SuppressLint("DefaultLocale")
    public void displayBasicTelemetry(Telemetry telemetry) {
        // Display the robot's current pose and headings on a single line
        double imuYawDegrees = getYawDegrees();
        double roadrunnerPoseHeadingDegrees = Math.toDegrees(mecanumDrive.pose.heading.log());

        telemetry.addLine(String.format("Pose: (X: %.2f, Y: %.2f, IMU Heading: %.2f°, RR Heading: %.2f°)",
                mecanumDrive.pose.position.x, mecanumDrive.pose.position.y, imuYawDegrees, roadrunnerPoseHeadingDegrees));

        // Display whether field-centric driving is enabled
        telemetry.addData("Field-Centric Driving: ", fieldOrientedControl ? "Enabled" : "Disabled");

        // Display the current speed of the robot
        double averageSpeed = calculateAverageSpeed();
        telemetry.addData("Average Speed: ", "%.2f in/s", averageSpeed);
    }

    /**
     * Calculate the average speed of the robot based on motor velocities.
     */
    private double calculateAverageSpeed() {
        // Read the current motor velocities in ticks per second
        double lfTicksPerSecond = mecanumDrive.leftFront.getVelocity();
        double lbTicksPerSecond = mecanumDrive.leftBack.getVelocity();
        double rfTicksPerSecond = mecanumDrive.rightFront.getVelocity();
        double rbTicksPerSecond = mecanumDrive.rightBack.getVelocity();

        // Average the motor velocities in ticks per second
        double averageTicksPerSecond = (lfTicksPerSecond + lbTicksPerSecond + rfTicksPerSecond + rbTicksPerSecond) / 4.0;

        // Convert ticks per second to inches per second
        double averageSpeedInchesPerSecond = (averageTicksPerSecond / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE * GEAR_RATIO;

        return averageSpeedInchesPerSecond;
    }

    // Display verbose telemetry for Driver Station
    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addLine("---- Drive Subsystem (Verbose) ----");
        displayYawTelemetry(telemetry);  // Detailed yaw telemetry
        telemetry.addData("Pose X", "%.2f", mecanumDrive.pose.position.x);
        telemetry.addData("Pose Y", "%.2f", mecanumDrive.pose.position.y);
        telemetry.addData("Pose Heading", "%.2f", Math.toDegrees(mecanumDrive.pose.heading.log()));
        telemetry.addData("Motor Power FL", "%.2f", mecanumDrive.leftFront.getPower());
        telemetry.addData("Motor Power FR", "%.2f", mecanumDrive.rightFront.getPower());
        telemetry.addData("Motor Power BL", "%.2f", mecanumDrive.leftBack.getPower());
        telemetry.addData("Motor Power BR", "%.2f", mecanumDrive.rightBack.getPower());
    }

    // Display IMU absolute yaw, FTC field yaw, and RoadRunner pose heading
    public void displayYawTelemetry(Telemetry telemetry) {
        // 1. IMU absolute yaw (in degrees)
        double imuYawDegrees = getYawDegrees();  // IMU's yaw from MecanumDrive

        // 2. RoadRunner pose heading (in degrees, without correction)
        double roadrunnerPoseHeadingDegrees = Math.toDegrees(mecanumDrive.pose.heading.log());

        // 3. FTC field convention yaw (based on the stored offset and IMU)
        double ftcYawDegrees = imuYawDegrees + Math.toDegrees(yawOffset);  // Use the stored yawOffset
        ftcYawDegrees = normalizeAngle(ftcYawDegrees);  // Normalize to -180 to 180 for display

        // Display telemetry data
        telemetry.addData("IMU Absolute Yaw (Degrees)", imuYawDegrees);
        telemetry.addData("FTC Field Convention Yaw (Degrees)", ftcYawDegrees);
        telemetry.addData("RoadRunner Pose Heading (Degrees)", roadrunnerPoseHeadingDegrees);

        // Optionally display any difference between field convention yaw and RoadRunner heading
        double yawDifference = ftcYawDegrees - roadrunnerPoseHeadingDegrees;
        telemetry.addData("IMU vs RoadRunner Yaw Difference", yawDifference);
    }

    // Use the IMU in MecanumDrive to get the current yaw angle
    public double getYawDegrees() {
        return mecanumDrive.getYawDegrees(); // Access IMU yaw from MecanumDrive
    }

    // Helper method to normalize angles to the range of -180 to 180 degrees
    private double normalizeAngle(double angle) {
        // Use conditional statements instead of a while loop
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    //Helper method to decide if driver gamepad is active
    public Boolean driverGamepadIsActive(double leftY, double leftX, double rightX) {
        return (Math.abs(leftY) > DriveParameters.DEAD_ZONE ||
                Math.abs(leftX) > DriveParameters.DEAD_ZONE ||
                Math.abs(rightX) > DriveParameters.DEAD_ZONE);
    }

}


