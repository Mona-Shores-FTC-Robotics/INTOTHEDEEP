package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

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

    public MecanumDriveMona mecanumDrive;
    private double currentRelativeYawDegrees;
    private double offsetFromAbsoluteYawDegrees; // To manage relative yaw offset

    public boolean fieldOrientedControl;

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
    }

    public void periodic(){
        updateCurrentRelativeYaw();
        Robot.getInstance().getActiveOpMode().telemetry.addData("Relative Yaw (Degrees)", currentRelativeYawDegrees);
        DriverStationTelemetry();
    }

    // Use the IMU in MecanumDrive to get the current yaw angle
    public double getYawDegrees() {
        return mecanumDrive.getYawDegrees(); // Access IMU yaw from MecanumDrive
    }

    // Get the current relative yaw (adjusted by offset)
    public void updateCurrentRelativeYaw() {
        double absoluteYawDegrees = getYawDegrees(); // Use MecanumDrive's IMU
        currentRelativeYawDegrees = absoluteYawDegrees + offsetFromAbsoluteYawDegrees;

        // Normalize to -180 to 180 degrees
        if (currentRelativeYawDegrees > 180) {
            currentRelativeYawDegrees -= 360;
        } else if (currentRelativeYawDegrees <= -180) {
            currentRelativeYawDegrees += 360;
        }
    }

    // Telemetry for driver station
    public void DriverStationTelemetry() {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        telemetry.addLine("Yaw Angle (Degrees): " + currentRelativeYawDegrees);
        telemetry.addLine("Robot Pose Heading (Degrees): " + Math.toDegrees(mecanumDrive.pose.heading.log()));
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

    public Boolean driverGamepadIsActive(double leftY, double leftX, double rightX) {
        return (Math.abs(leftY) > DriveParameters.DEAD_ZONE ||
                Math.abs(leftX) > DriveParameters.DEAD_ZONE ||
                Math.abs(rightX) > DriveParameters.DEAD_ZONE);
    }

    /**
     * Implements field-centric control. Adjusts the robot's movement based on the heading.
     */
    public void fieldOrientedControl(double leftY, double leftX) {
        double y = leftY;
        double x = leftX;

        // Get the robot's current heading in radians (directly from the gyro)
        double botHeading = Math.toRadians(getYawDegrees());

        // Determine the heading offset based on alliance (assuming you're tracking this in MatchConfig)
        double headingOffset = 0;  // Default

        if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
            headingOffset = Math.toRadians(90);  // 90 degrees for blue side
        } else if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED) {
            headingOffset = Math.toRadians(-90);  // -90 degrees for red side
        }

        // Adjust the robot's heading by the alliance-specific offset
        double adjustedHeading = botHeading - headingOffset;

        // Rotate the movement direction relative to the robot's adjusted heading
        leftXAdjusted = x * Math.cos(-adjustedHeading) - y * Math.sin(-adjustedHeading);
        leftYAdjusted = x * Math.sin(-adjustedHeading) + y * Math.cos(-adjustedHeading);

        // Optionally counteract imperfect strafing
        leftYAdjusted = Math.min(leftYAdjusted * 1.1, 1);
    }
}


