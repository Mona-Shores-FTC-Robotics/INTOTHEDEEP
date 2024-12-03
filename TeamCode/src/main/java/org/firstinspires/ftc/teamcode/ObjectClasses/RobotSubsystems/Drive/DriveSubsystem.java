package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static java.lang.Math.abs;
import static java.lang.Math.cos;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.ConnectionInfoFormatter;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
public class DriveSubsystem extends SubsystemBase {

    public static TeleopParams.StickParams STICK_PARAMS = new TeleopParams.StickParams();
    public static TeleopParams.PIDParams PID_PARAMS = new TeleopParams.PIDParams();
    public static TeleopParams.RampParams RAMP_PARAMS = new TeleopParams.RampParams();

    protected static MecanumDrive mecanumDrive;

    public boolean fieldOrientedControl;

    public double drive;
    public double strafe;
    public double turn;

    private double current_drive_ramp = 0;
    private double current_strafe_ramp = 0;
    private double current_turn_ramp = 0;

    public double leftYAdjusted;
    public double leftXAdjusted;

    public double MAX_MOTOR_SPEED_RPS = 435.0 / 60.0;
    public double TICKS_PER_REV = 384.5;
    public double MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;

    public double leftFrontTargetSpeed;
    public double rightFrontTargetSpeed;
    public double leftBackTargetSpeed;
    public double rightBackTargetSpeed;
    private DriveModeConfig.DriveMode lastDriveMode = null;
    private YawPitchRollAngles angles;

    private double prevLeftFrontTargetSpeed = 0;
    private double prevRightFrontTargetSpeed = 0;
    private double prevLeftBackTargetSpeed = 0;
    private double prevRightBackTargetSpeed = 0;


    @Config
    public static class DriveModeConfig {
        public static DriveMode selectedDriveMode = DriveMode.SPEED_CONTROL;

        // Enum needs to be static or a top-level class for it to work smoothly with older JDK versions.
        public enum DriveMode {
            SPEED_CONTROL,
            POWER_WITHOUT_ENCODERS
        }
    }

    public DriveSubsystem(HardwareMap hardwareMap, Robot.RobotType robotType) {
        // Initialize appropriate drive system based on robot type
            switch (robotType) {
                case INTO_THE_DEEP_19429:
                    DriveParams.configureIntoTheDeep19429RRParams();
                    //if auto has run use the pose that was last set in auto
                    if (MatchConfig.hasAutoRun)
                    {
                        if (MatchConfig.endOfAutonomousPose!=null) mecanumDrive = new PinpointDrive(hardwareMap, MatchConfig.endOfAutonomousPose);
                        else
                        {
                            MatchConfig.hasAutoRun=false;
                            mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
                        }
                    }
                    //if auto has not run use a blank pose because we will set the pose based on the user setup stuff during init
                    else  mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

                    DriveParams.configureIntoTheDeep19429Directions(mecanumDrive);
                    break;

                case INTO_THE_DEEP_20245:
                    DriveParams.configureIntoTheDeep20245RRParams();
                    if (MatchConfig.hasAutoRun) {
                        if (MatchConfig.endOfAutonomousPose != null)
                            mecanumDrive = new PinpointDrive(hardwareMap, MatchConfig.endOfAutonomousPose);
                        else {
                            MatchConfig.hasAutoRun = false;
                            mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
                        }
                    } else  mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));                    DriveParams.configureIntoTheDeep20245Directions(mecanumDrive);
                    break;
            }
            mecanumDrive.lazyImu.get().resetYaw();
            configurePID();
    }

    public void init() {
        fieldOrientedControl = true; // Default to field-oriented control
    }

    public void periodic() {
        updatePIDFromDashboard();
        mecanumDrive.updatePoseEstimate();
        MatchConfig.telemetryPacket.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(MatchConfig.telemetryPacket.fieldOverlay(), mecanumDrive.pose);
    }

    private double lastP = 0, lastI = 0, lastD = 0, lastF = 0;

    private void updatePIDFromDashboard() {
        if (PID_PARAMS.P != lastP || PID_PARAMS.I != lastI ||
                PID_PARAMS.D != lastD || PID_PARAMS.F != lastF) {

            // Update the PID configuration when changes are detected
            configurePID();

            // Store the current values as the last known values
            lastP = PID_PARAMS.P;
            lastI = PID_PARAMS.I;
            lastD = PID_PARAMS.D;
            lastF = PID_PARAMS.F;
        }
    }

    public void CalculateYawOffset() {
        if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
            MatchConfig.offsetFromStartPoseDegrees = 90;  // 90 degrees for blue side
        } else {
            MatchConfig.offsetFromStartPoseDegrees = -90;  // -90 degrees for red side
        }
    }


    public MecanumDrive getMecanumDrive() {
        return mecanumDrive; // This could be a MecanumDrive/SparkFunDrive/PinpointDrive depending on initialization
    }

    /**
     * Adjust the drive, strafe, and turn values for robot-centric or field-centric control
     */
    public void setDriveStrafeTurnValues(double leftY, double leftX, double rightX) {
        boolean gamepadActive = driverGamepadIsActive(leftY, leftX, rightX);
        if (gamepadActive) {
            double speedFactor = isNitroModeEnabled() ? STICK_PARAMS.NITRO_MODE_FACTOR : 1.0;
            //slow mode trumps nitro mode
            speedFactor = isSlowModeEnabled() ? STICK_PARAMS.SLOW_MODE_FACTOR : speedFactor;

            if (fieldOrientedControl) {
                // Perform FOC rotation
                fieldOrientedControl(leftY, leftX);
            } else {
                // In robot-centric control, use compensated joystick inputs directly
                leftYAdjusted = leftY;
                leftXAdjusted = leftX;
            }
            // Calculate magnitude and angle of robot-centric movement vector
            double robotMovementMagnitude = Math.hypot(leftXAdjusted, leftYAdjusted);
            double robotMovementAngle = Math.atan2(leftYAdjusted, leftXAdjusted);

            // Get the blended scaling factor based on robot-centric movement angle
            double blendedScalingFactor = getBlendedScalingFactor(robotMovementAngle);

            // Apply the blended scaling factor to the magnitude
            double adjustedMagnitude = robotMovementMagnitude * blendedScalingFactor * speedFactor;

            // Reconstruct the adjusted robot-centric movement components
            double compensatedX = adjustedMagnitude * Math.cos(robotMovementAngle);
            double compensatedY = adjustedMagnitude * Math.sin(robotMovementAngle);

            // Assign adjusted movement commands
            drive = compensatedY;
            strafe = compensatedX;

            // Apply scaling to turn input
            turn = rightX * STICK_PARAMS.TURN_SPEED_FACTOR * speedFactor;
        } else {
            // No input, set adjusted values to 0
            drive = 0;
            strafe = 0;
            turn = 0;
        }
    }

    /**
     * Adjust the drive, strafe, and turn values for robot-centric or field-centric control
     */
    public void setDriveStrafeValues(double leftY, double leftX) {
        boolean gamepadActive = driverGamepadIsActive(leftY, leftX);
        if (gamepadActive) {
            double speedFactor = isNitroModeEnabled() ? STICK_PARAMS.NITRO_MODE_FACTOR : 1.0;
            //slow mode trumps nitro mode
            speedFactor = isSlowModeEnabled() ? STICK_PARAMS.SLOW_MODE_FACTOR : speedFactor;
            if (fieldOrientedControl) {
                // Perform FOC rotation
                fieldOrientedControl(leftY, leftX);
            } else {
                // In robot-centric control, use compensated joystick inputs directly
                leftYAdjusted = leftY;
                leftXAdjusted = leftX;
            }
            // Calculate magnitude and angle of robot-centric movement vector
            double robotMovementMagnitude = Math.hypot(leftXAdjusted, leftYAdjusted);
            double robotMovementAngle = Math.atan2(leftYAdjusted, leftXAdjusted);

            // Get the blended scaling factor based on robot-centric movement angle
            double blendedScalingFactor = getBlendedScalingFactor(robotMovementAngle);

            // Apply the blended scaling factor to the magnitude
            double adjustedMagnitude = robotMovementMagnitude * blendedScalingFactor * speedFactor;

            // Reconstruct the adjusted robot-centric movement components
            double compensatedX = adjustedMagnitude * Math.cos(robotMovementAngle);
            double compensatedY = adjustedMagnitude * Math.sin(robotMovementAngle);

            // Assign adjusted movement commands
            drive = compensatedY;
            strafe = compensatedX;
        } else {
            // No input, set adjusted values to 0
            leftYAdjusted = 0;
            leftXAdjusted = 0;
        }
    }


    /**
     * Calculates a compensation factor based on the desired movement angle.
     * Adjusts for the difference in speed between forward/backward and strafing movements in mecanum drive.
     *
     * @param angle The desired movement angle in radians (relative to the field)
     * @return The compensation factor to apply to the movement magnitude
     */
    private double getDirectionCompensationFactor(double angle) {
        // Normalize angle to range [0, 2π)
        angle = (angle + 2 * Math.PI) % (2 * Math.PI);

        // Define the compensation factors
        double forwardBackwardFactor = STICK_PARAMS.DRIVE_SPEED_FACTOR; // e.g., 1.0
        double strafeFactor = STICK_PARAMS.STRAFE_SPEED_FACTOR; // e.g., 1.2

        // Calculate the compensation factor based on the angle
        // Use the fact that forward/backward movement occurs at angles of 0°, 180° (0, π)
        // Strafing occurs at angles of 90°, 270° (π/2, 3π/2)
        double cosAngle = Math.abs(Math.cos(angle));
        double sinAngle = Math.abs(Math.sin(angle));

        // Weighted average of the forward/backward and strafe factors based on movement direction
        double compensationFactor = (forwardBackwardFactor * cosAngle) + (strafeFactor * sinAngle);

        // Normalize the compensation factor
        compensationFactor /= (cosAngle + sinAngle);

        return compensationFactor;
    }

    /**
     * Calculates a blended scaling factor based on the robot-centric movement angle.
     * Blends the drive and strafe scaling factors to compensate for mechanical differences.
     *
     * @param angle The movement angle in robot-centric coordinates (radians)
     * @return The blended scaling factor to apply to the movement magnitude
     */
    /**
     * Calculates a blended scaling factor based on the robot-centric movement angle.
     * Blends the drive and strafe scaling factors to compensate for mechanical differences.
     *
     * @param angle The movement angle in robot-centric coordinates (radians)
     * @return The blended scaling factor to apply to the movement magnitude
     */
    private double getBlendedScalingFactor(double angle) {
        // Normalize angle to range [0, 2π)
        angle = (angle + 2 * Math.PI) % (2 * Math.PI);

        // Retrieve scaling factors
        double driveScaling = STICK_PARAMS.DRIVE_SPEED_FACTOR;
        double strafeScaling = STICK_PARAMS.STRAFE_SPEED_FACTOR;

        // Compute cosine and sine of the angle
        double cosAngle = Math.cos(angle); // Represents strafe component
        double sinAngle = Math.sin(angle); // Represents drive component

        // Compute weighted scaling factors
        double driveComponent = Math.abs(sinAngle) * driveScaling;
        double strafeComponent = Math.abs(cosAngle) * strafeScaling;

        // Compute normalization factor
        double normalizationFactor = Math.abs(sinAngle) + Math.abs(cosAngle);

        // Avoid division by zero
        if (normalizationFactor == 0) {
            return 0;
        }

        // Compute the blended scaling factor
        double blendedScalingFactor = (driveComponent + strafeComponent) / normalizationFactor;

        return blendedScalingFactor;
    }

    /**
     * Implements field-centric control. Adjusts the robot's movement based on the heading.
     */
    public void fieldOrientedControl(double y, double x) {
        // Get the robot's current heading in radians (include any offsets)
        double botHeading = mecanumDrive.pose.heading.toDouble()
                + Math.toRadians(MatchConfig.offsetFromStartPoseDegrees);

        // Calculate the cosine and sine of the negative heading
        double cosHeading = Math.cos(-botHeading);
        double sinHeading = Math.sin(-botHeading);

        // Perform the rotation to get robot-centric commands
        leftXAdjusted = x * cosHeading - y * sinHeading;
        leftYAdjusted = x * sinHeading + y * cosHeading;
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
        return (Math.abs(leftY) > STICK_PARAMS.DEAD_ZONE ||
                Math.abs(leftX) > STICK_PARAMS.DEAD_ZONE ||
                Math.abs(rightX) > STICK_PARAMS.DEAD_ZONE);
    }

    public Boolean driverGamepadIsActive(double leftY, double leftX) {
        return (Math.abs(leftY) > STICK_PARAMS.DEAD_ZONE ||
                Math.abs(leftX) > STICK_PARAMS.DEAD_ZONE);
    }

    public void configurePID() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(PID_PARAMS.P, PID_PARAMS.I, PID_PARAMS.D, PID_PARAMS.F);

        // Set PID values for motors
        mecanumDrive.leftFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        mecanumDrive.rightFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        mecanumDrive.leftBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        mecanumDrive.rightBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    /**
     * Method to get current yaw (heading) in degrees from the IMU.
     */
    public double getInternalIMUYawDegrees() {
        return angles.getYaw(AngleUnit.DEGREES);  // Return yaw in degrees
    }

    /**
     * Method to get current yaw (heading) in degrees from the IMU.
     */
    public double getInternalIMUYawDegreesWithOffsetApplied() {
        return normalizeAngle(angles.getYaw(AngleUnit.DEGREES) - MatchConfig.offsetFromStartPoseDegrees);  // Return yaw in degrees
    }

    public void updateInternalIMU() {
        angles = mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles(); // Access IMU
    }

    public TrajectoryActionBuilder rotatedActionBuilder(Pose2d beginPose) {

        return new TrajectoryActionBuilder(
                (TimeTurn turn) -> mecanumDrive.new TurnAction(turn),
                (TimeTrajectory trajectory) -> mecanumDrive.new FollowTrajectoryAction(trajectory),
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(.25, .1, 1e-2
                        )
                ),
                beginPose, 0.0,
                Robot.getInstance().getDriveSubsystem().getMecanumDrive().defaultTurnConstraints,
                Robot.getInstance().getDriveSubsystem().getMecanumDrive().defaultVelConstraint,
                Robot.getInstance().getDriveSubsystem().getMecanumDrive().defaultAccelConstraint,
                pose -> new Pose2dDual<>(
                        pose.position.x.unaryMinus(), pose.position.y.unaryMinus(), pose.heading.plus(Math.toRadians(180))));
    }

    // ========== Helper/Utility Methods ==========
    private double Ramp(double target, double currentValue, double ramp_amount) {
        if (Math.abs(currentValue) + RAMP_PARAMS.RAMP_THRESHOLD < Math.abs(target)) {
            return Math.signum(target) * (Math.abs(currentValue) + ramp_amount);
        } else {
            return target;
        }
    }

    public static class TeleopParams {

        public static class StickParams {
            public double DEAD_ZONE = 0.1;
            public double DRIVE_SPEED_FACTOR = 0.7;
            public double STRAFE_SPEED_FACTOR = .92;
            public double TURN_SPEED_FACTOR = 1.0;
            public double SLOW_MODE_FACTOR = 0.5; // Reduce speed by 50% in slow mode
            public double NITRO_MODE_FACTOR = 1.8;
        }

        public static class RampParams {
            public double DRIVE_RAMP = 0.2;
            public double STRAFE_RAMP = 0.22;
            public double TURN_RAMP = 0.4;
            public double RAMP_THRESHOLD = 0.04;
        }

        public static class PIDParams {
            public double P = 3.0;
            public double D = 0;
            public double I = 0;
            public double F = 8.0;
        }
    }

    public void cycleDriveMode() {
        // Get the current drive mode
        DriveModeConfig.DriveMode currentMode = DriveModeConfig.selectedDriveMode;

        // Cycle to the next drive mode by incrementing the ordinal and wrapping around using modulus
        DriveModeConfig.selectedDriveMode = DriveModeConfig.DriveMode.values()[(currentMode.ordinal() + 1) % DriveModeConfig.DriveMode.values().length];
    }


    public void drive(double drive, double strafe, double turn) {
        DriveModeConfig.DriveMode currentMode = DriveModeConfig.selectedDriveMode;

        // Only change motor modes when the drive mode changes
        if (currentMode != lastDriveMode) {
            switch (currentMode) {
                case SPEED_CONTROL:
                    setMotorsRunUsingEncoder();
                    break;

                case POWER_WITHOUT_ENCODERS:
                    setMotorsRunWithoutEncoder();
                    break;
            }

            // Update the last drive mode to the current one
            lastDriveMode = currentMode;
        }

        // Now handle the actual drive logic based on the current mode
        switch (currentMode) {
            case SPEED_CONTROL:
                mecanumDriveSpeedControl(drive, strafe, turn);
                break;

            case POWER_WITHOUT_ENCODERS:
                mecanumDrivePowerControl(drive, strafe, turn);
                break;

            default:
                mecanumDrivePowerControl(drive, strafe, turn);
                break;
        }
    }

    private void setMotorsRunUsingEncoder() {
        mecanumDrive.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        mecanumDrive.leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive.leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive.rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive.rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void setMotorsRunWithoutEncoder() {
        mecanumDrive.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        mecanumDrive.leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        mecanumDrive.leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        mecanumDrive.rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        mecanumDrive.rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void mecanumDriveSpeedControl(double drive, double strafe, double turn) {
        if (drive == 0 && strafe == 0 && turn == 0) {
            //if power is not set to zero its jittery, doesn't work at all if we don't reset the motors back to run using encoders...
            leftFrontTargetSpeed = 0;
            rightFrontTargetSpeed = 0;
            leftBackTargetSpeed = 0;
            rightBackTargetSpeed = 0;

            // Only set power and velocity to zero if they are not already zero
            if (prevLeftFrontTargetSpeed != 0) {
                mecanumDrive.leftFront.setPower(0);
                mecanumDrive.leftFront.setVelocity(0);
                prevLeftFrontTargetSpeed = 0;
            }
            if (prevRightFrontTargetSpeed != 0) {
                mecanumDrive.rightFront.setPower(0);
                mecanumDrive.rightFront.setVelocity(0);
                prevRightFrontTargetSpeed = 0;
            }
            if (prevLeftBackTargetSpeed != 0) {
                mecanumDrive.leftBack.setPower(0);
                mecanumDrive.leftBack.setVelocity(0);
                prevLeftBackTargetSpeed = 0;
            }
            if (prevRightBackTargetSpeed != 0) {
                mecanumDrive.rightBack.setPower(0);
                mecanumDrive.rightBack.setVelocity(0);
                prevRightBackTargetSpeed = 0;
            }

            current_drive_ramp = 0;
            current_strafe_ramp = 0;
            current_turn_ramp = 0;

        } else {
            //If we see blue tags and we are red and we are driving toward them, then use the safetydrivespeedfactor to slow us down
            current_drive_ramp = Ramp(drive, current_drive_ramp, RAMP_PARAMS.DRIVE_RAMP);
            current_strafe_ramp = Ramp(strafe, current_strafe_ramp, RAMP_PARAMS.STRAFE_RAMP);
            current_turn_ramp = Ramp(turn, current_turn_ramp, RAMP_PARAMS.TURN_RAMP);

            double dPercent = abs(current_drive_ramp) / (abs(current_drive_ramp) + abs(current_strafe_ramp) + abs(current_turn_ramp));
            double sPercent = abs(current_strafe_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));
            double tPercent = abs(current_turn_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));

            leftFrontTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightFrontTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));
            leftBackTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightBackTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));

            // Update only if the target speed has changed
            if (leftFrontTargetSpeed != prevLeftFrontTargetSpeed) {
                mecanumDrive.leftFront.setVelocity(leftFrontTargetSpeed);
                prevLeftFrontTargetSpeed = leftFrontTargetSpeed;
            }
            if (rightFrontTargetSpeed != prevRightFrontTargetSpeed) {
                mecanumDrive.rightFront.setVelocity(rightFrontTargetSpeed);
                prevRightFrontTargetSpeed = rightFrontTargetSpeed;
            }
            if (leftBackTargetSpeed != prevLeftBackTargetSpeed) {
                mecanumDrive.leftBack.setVelocity(leftBackTargetSpeed);
                prevLeftBackTargetSpeed = leftBackTargetSpeed;
            }
            if (rightBackTargetSpeed != prevRightBackTargetSpeed) {
                mecanumDrive.rightBack.setVelocity(rightBackTargetSpeed);
                prevRightBackTargetSpeed = rightBackTargetSpeed;
            }
        }
    }

    public void mecanumDrivePowerControl(double drive, double strafe, double turn) {
        double dPercent = abs(drive) / (abs(drive) + abs(strafe) + abs(turn));
        double sPercent = abs(strafe) / (abs(drive) + abs(turn) + abs(strafe));
        double tPercent = abs(turn) / (abs(drive) + abs(turn) + abs(strafe));

        double leftFrontPower = ((drive * dPercent) + (strafe * sPercent) + (turn * tPercent));
        double rightFrontPower = ((drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent));
        double leftBackPower = ((drive * dPercent) + (-strafe * sPercent) + (turn * tPercent));
        double rightBackPower = ((drive * dPercent) + (strafe * sPercent) + (-turn * tPercent));

        mecanumDrive.leftFront.setPower(leftFrontPower);
        mecanumDrive.rightFront.setPower(rightFrontPower);
        mecanumDrive.leftBack.setPower(leftBackPower);
        mecanumDrive.rightBack.setPower(rightBackPower);
    }

    public void rrDriveControl(double left_stick_y, double left_stick_x, double right_stick_x) {
        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        left_stick_y,
                        -left_stick_x
                ),
                -right_stick_x
        ));
    }

    // Static inner class for organizing telemetry methods
    public static class TelemetryHelper {

        public static void displayVerboseTelemetry(Telemetry telemetry) {
            displayBasicTelemetry(telemetry);
            telemetry.addLine();
            displayDriveMotorDetails(telemetry);
            telemetry.addLine();
            displayPinpointDetails(telemetry);
            telemetry.addLine();
            displayYawTelemetry(telemetry);
        }

        @SuppressLint("DefaultLocale")
        public static void displayBasicTelemetry(Telemetry telemetry) {
            MecanumDrive mecanumDrive = Robot.getInstance().getDriveSubsystem().getMecanumDrive();
            telemetry.addLine()
                    .addData("Current Pose", String.format("(X: %.1f, Y: %.1f, Yaw: %.1f°)",
                            mecanumDrive.pose.position.x,
                            mecanumDrive.pose.position.y,
                            Math.toDegrees(mecanumDrive.pose.heading.log())));
            telemetry.addData("Drive Mode", DriveModeConfig.selectedDriveMode);
            telemetry.addData("Field-Centric Driving", Robot.getInstance().getDriveSubsystem().fieldOrientedControl ? "Enabled" : "Disabled");
        }

        @SuppressLint("DefaultLocale")
        public static void displayDriveMotorDetails(Telemetry telemetry) {
            DriveSubsystem driveSubsystem = Robot.getInstance().getDriveSubsystem();
            MecanumDrive mecanumDrive = driveSubsystem.getMecanumDrive();

            // Extract port numbers for each motor
            int lfPort = ConnectionInfoFormatter.extractPortNumber(mecanumDrive.leftFront.getConnectionInfo());
            int rfPort = ConnectionInfoFormatter.extractPortNumber(mecanumDrive.rightFront.getConnectionInfo());
            int lbPort = ConnectionInfoFormatter.extractPortNumber(mecanumDrive.leftBack.getConnectionInfo());
            int rbPort = ConnectionInfoFormatter.extractPortNumber(mecanumDrive.rightBack.getConnectionInfo());

            // Create header line
            String header = String.format("Ports        | LF(%d)      | RF(%d)      | LB(%d)      | RB(%d)", lfPort, rfPort, lbPort, rbPort);
            telemetry.addLine(header);

            // Create Power line with leading zeros and percentage symbol
            String powerLine = String.format("Power       | %03d%%     | %03d%%     | %03d%%      | %03d%%",
                    (int) (mecanumDrive.leftFront.getPower() * 100),
                    (int) (mecanumDrive.rightFront.getPower() * 100),
                    (int) (mecanumDrive.leftBack.getPower() * 100),
                    (int) (mecanumDrive.rightBack.getPower() * 100));
            telemetry.addLine(powerLine);

            // Create Velocity line (optional: keep decimal for precision)
            String velocityLine = String.format("Velocity    | %04d     | %04d      | %04d      | %04d",
                    (int) mecanumDrive.leftFront.getVelocity(),
                    (int) mecanumDrive.rightFront.getVelocity(),
                    (int) mecanumDrive.leftBack.getVelocity(),
                    (int) mecanumDrive.rightBack.getVelocity());
            telemetry.addLine(velocityLine);

            // Create Encoder Position line

            if (mecanumDrive.localizer instanceof MecanumDrive.DriveLocalizer) {
                MecanumDrive.DriveLocalizer localizer = (MecanumDrive.DriveLocalizer) mecanumDrive.localizer;

                String encoderPosLine = String.format("Encoder    | %04d    | %04d      | %04d      | %04d",
                    (int) localizer.leftFront.getPositionAndVelocity().position,
                    (int) localizer.rightFront.getPositionAndVelocity().position,
                    (int) localizer.leftBack.getPositionAndVelocity().position,
                    (int) localizer.rightBack.getPositionAndVelocity().position);
            telemetry.addLine(encoderPosLine);
            }
        }

        @SuppressLint("DefaultLocale")
        public static void displayPinpointDetails(Telemetry telemetry) {
            // Retrieve the current MecanumDrive instance from the robot's drive subsystem
            MecanumDrive mecanumDrive = Robot.getInstance().getDriveSubsystem().getMecanumDrive();

            // Check if the current drive system is an instance of PinpointDrive
            if (mecanumDrive instanceof PinpointDrive) {
                PinpointDrive pinpointDrive = (PinpointDrive) mecanumDrive;
                GoBildaPinpointDriverRR pinpoint = pinpointDrive.pinpoint;

                // Retrieve the required telemetry data
                GoBildaPinpointDriverRR.DeviceStatus deviceStatus = pinpoint.getDeviceStatus(); // Device status
                double loopTime = pinpoint.getLoopTime(); // Loop time in microseconds (us)
                String connectionInfo = pinpoint.getConnectionInfo(); // Raw connection info string

                // Reformat the connection information using the utility class
                String formattedConnectionInfo = ConnectionInfoFormatter.reformatConnectionInfo(connectionInfo);

                // Consolidate all telemetry data into two lines
                String line1 = String.format("Pinpoint Status: %s", deviceStatus.toString());
                String line2 = String.format("Loop Time (us): %.2f | %s", loopTime, formattedConnectionInfo);

                // Add the consolidated telemetry lines
                telemetry.addLine(line1);
                telemetry.addLine(line2);
            } else {
                // Inform the user if the drive system is not an instance of PinpointDrive
                telemetry.addLine("MecanumDrive is not an instance of PinpointDrive.");
            }
        }

        @SuppressLint("DefaultLocale")
        public static void displayYawTelemetry(Telemetry telemetry) {
            DriveSubsystem driveSubsystem = Robot.getInstance().getDriveSubsystem();

            // Retrieve IMU Yaw in degrees
            driveSubsystem.updateInternalIMU();
            double imuYawDegrees = driveSubsystem.getInternalIMUYawDegreesWithOffsetApplied();

            // Retrieve Pose Heading from Pinpoint (Roadrunner) in degrees
            double poseHeadingDegrees = Math.toDegrees(driveSubsystem.getMecanumDrive().pose.heading.log());

            // Calculate the difference between IMU Yaw and Pose Heading
            double yawDifference = imuYawDegrees - poseHeadingDegrees;

            // Normalize yaw difference to the range [-180, 180)
            yawDifference = normalizeAngle(yawDifference);

            // Display telemetry data
            telemetry.addData("IMU Yaw (Degrees)", "%.1f", imuYawDegrees);
            telemetry.addData("Pinpoint Pose Heading (Degrees)", "%.1f", poseHeadingDegrees);
            telemetry.addData("Yaw Difference (IMU - Pose)", "%.1f°", yawDifference);
        }

        /**
         * Normalizes an angle to the range [-180, 180) degrees.
         *
         * @param angle The angle in degrees to normalize.
         * @return The normalized angle in degrees.
         */
        private static double normalizeAngle(double angle) {
            angle = angle % 360;
            if (angle >= 180) {
                angle -= 360;
            } else if (angle < -180) {
                angle += 360;
            }
            return angle;
        }
    }

    private boolean slowModeEnabled = false;
    private boolean nitroModeEnabled = false;


    public void enableNitroMode() {
        nitroModeEnabled = true;
    }

    public void disableNitroMode() {
        nitroModeEnabled = false;
    }

    public void enableSlowMode() {
        slowModeEnabled = true;
    }

    public void disableSlowMode() {
        slowModeEnabled = false;
    }

    public boolean isNitroModeEnabled() {
        return nitroModeEnabled;
    }

    public boolean isSlowModeEnabled() {
        return slowModeEnabled;
    }

}