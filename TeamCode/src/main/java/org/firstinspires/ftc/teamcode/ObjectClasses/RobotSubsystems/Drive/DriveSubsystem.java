package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static java.lang.Math.abs;
import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@Config
public class DriveSubsystem extends SubsystemBase {

    public static TeleopParams.StickParams STICK_PARAMS = new TeleopParams.StickParams();
    public static TeleopParams.PIDParams PID_PARAMS = new TeleopParams.PIDParams();
    public static TeleopParams.RampParams RAMP_PARAMS = new TeleopParams.RampParams();

    private MecanumDrive mecanumDrive;

    public boolean fieldOrientedControl;
    public double yawOffsetDegrees;  // offset depending on alliance color

    public double drive;
    public double strafe;
    public double turn;

    private double current_drive_ramp=0;
    private double current_strafe_ramp=0;
    private double current_turn_ramp=0;

    public double leftYAdjusted;
    public double leftXAdjusted;
    public double rightXAdjusted;

    public double MAX_MOTOR_SPEED_RPS = 435.0 / 60.0;
    public double TICKS_PER_REV = 384.5;
    public double MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;

    public double leftFrontTargetSpeed;
    public double rightFrontTargetSpeed;
    public double leftBackTargetSpeed;
    public double rightBackTargetSpeed;
    public Encoder leftFrontEncoder, leftBackEncoder, rightBackEncoder, rightFrontEncoder;
    private DriveModeConfig.DriveMode lastDriveMode = null;
    private YawPitchRollAngles angles;

    @Config
    public static class DriveModeConfig {
        public static DriveMode selectedDriveMode = DriveMode.SPEED_CONTROL;

        // Enum needs to be static or a top-level class for it to work smoothly with older JDK versions.
        public enum DriveMode {
            SPEED_CONTROL,
            RR_SET_DRIVE_POWER,
            POWER_WITH_ENCODERS,
            POWER_WITHOUT_ENCODERS
        }
    }
    public DriveSubsystem(HardwareMap hardwareMap, Robot.RobotType robotType) {
        // Initialize appropriate drive system based on robot type
        switch (robotType) {
            case CHASSIS_19429_A_PINPOINT:
                DriveParams.configureChassis19429PinpointRRParams();
                mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
                initializeMotorEncoders();
                DriveParams.configureChassis19429ADirections(mecanumDrive, this);
                break;

            case CENTERSTAGE_PINPOINT:
                DriveParams.configureCenterstagePinpointRRParams();
                mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
                initializeMotorEncoders();
                DriveParams.configureCenterStageDirections(mecanumDrive, this);
                break;

            case CHASSIS_19429_HUB_TWO_DEAD_WHEELS:
                DriveParams.configureChassis19429TwoDeadWheelRRParams();
                mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                initializeMotorEncoders();
                DriveParams.configureChassis19429ADirections(mecanumDrive, this);
                mecanumDrive.localizer = new TwoDeadWheelLocalizer(hardwareMap, mecanumDrive.lazyImu.get(), MecanumDrive.PARAMS.inPerTick);
                ((TwoDeadWheelLocalizer) mecanumDrive.localizer).par.setDirection(DcMotorEx.Direction.REVERSE);
                ((TwoDeadWheelLocalizer) mecanumDrive.localizer).perp.setDirection(DcMotorEx.Direction.REVERSE);
                TwoDeadWheelLocalizer.PARAMS.parYTicks = -1450.0;
                TwoDeadWheelLocalizer.PARAMS.perpXTicks =  800.0;
                break;

            case CENTERSTAGE_HUB_TWO_DEAD_WHEELS:
                DriveParams.configureCenterstageTwoDeadWheelRRParams();
                mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                initializeMotorEncoders();
                DriveParams.configureCenterStageDirections(mecanumDrive, this);
                mecanumDrive.localizer = new TwoDeadWheelLocalizer(hardwareMap, mecanumDrive.lazyImu.get(), MecanumDrive.PARAMS.inPerTick);
                ((TwoDeadWheelLocalizer) mecanumDrive.localizer).par.setDirection(DcMotorEx.Direction.FORWARD);
                ((TwoDeadWheelLocalizer) mecanumDrive.localizer).perp.setDirection(DcMotorEx.Direction.FORWARD);
                TwoDeadWheelLocalizer.PARAMS.parYTicks = -1440.0;
                TwoDeadWheelLocalizer.PARAMS.perpXTicks =  820.0;
                break;

            case CENTERSTAGE_OTOS:
                //TODO This is not complete
                this.mecanumDrive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
                // No custom params for OTOS in this case, keep using default
                break;
        }
        mecanumDrive.lazyImu.get().resetYaw();
        configurePID();
    }

    public void init()
    {
        Robot.getInstance().registerSubsystem(Robot.SubsystemType.DRIVE);
        fieldOrientedControl=true; // Default to field-oriented control
        CalculateYawOffset();

    }

    public void periodic(){
        updatePIDFromDashboard();
        updateIMU();
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

    private void CalculateYawOffset() {
        // Calculate yaw offset based on alliance color
        // This offset assumes robot forward direction faces away from driver
        //      -For red start, the audience is on your left
        //      -For blue start, the audience is on your right
        //  See this: https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
        if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
            yawOffsetDegrees = 90;  // 90 degrees for blue side
        } else {
            yawOffsetDegrees = -90;  // -90 degrees for red side
        }
    }

    public MecanumDrive getMecanumDrive()
    {
        return mecanumDrive; // This could be a MecanumDrive/SparkFunDrive/PinpointDrive depending on initialization
    }

    /**
     * Adjust the drive, strafe, and turn values for robot-centric or field-centric control
     */
    public void setDriveStrafeTurnValues(double leftY, double leftX, double rightX) {
        boolean gamepadActive = driverGamepadIsActive(leftY, leftX, rightX);

        if (gamepadActive) {
            leftYAdjusted = leftY * STICK_PARAMS.DRIVE_SPEED_FACTOR;
            leftXAdjusted = leftX * STICK_PARAMS.STRAFE_SPEED_FACTOR;
            rightXAdjusted = rightX *   STICK_PARAMS.TURN_SPEED_FACTOR;
            if (fieldOrientedControl) {
                fieldOrientedControl(leftYAdjusted, leftXAdjusted);
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
    }

    /**
     * Implements field-centric control. Adjusts the robot's movement based on the heading.
     */
    public void fieldOrientedControl(double y, double x) {
        // Get the robot's current heading in radians (directly from the gyro)
        double botHeading = mecanumDrive.pose.heading.toDouble() + Math.toRadians(yawOffsetDegrees);  //this is in radians

        // Rotate the movement direction relative to the robot's adjusted heading
        leftXAdjusted = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        leftYAdjusted = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Optionally counteract imperfect strafing
        leftYAdjusted = Math.min(leftYAdjusted * 1.1, 1);
    }

    // Display basic telemetry for Driver Station
    @SuppressLint("DefaultLocale")
    public void displayBasicTelemetry(Telemetry telemetry) {
        //Display driving mode
        telemetry.addData("Drive Mode", DriveModeConfig.selectedDriveMode);

        // Display whether field-centric driving is enabled
        telemetry.addData("Field-Centric Driving", fieldOrientedControl ? "Enabled" : "Disabled");
        telemetry.addLine("");
        double roadrunnerPoseHeadingDegrees = Math.toDegrees(mecanumDrive.pose.heading.log());
        telemetry.addLine(String.format("(X: %.1f, Y: %.1f, Yaw: %.1f°)",
                mecanumDrive.pose.position.x, mecanumDrive.pose.position.y, roadrunnerPoseHeadingDegrees));
    }

    // Display verbose telemetry for Driver Station
    @SuppressLint("DefaultLocale")
    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive Mode", DriveModeConfig.selectedDriveMode);
        telemetry.addData("Field-Centric Driving", fieldOrientedControl ? "Enabled" : "Disabled");
        telemetry.addLine();
        telemetry.addLine("Motor Directions");
        telemetry.addData("Front Motors", "LF: %s    RF: %s",
                mecanumDrive.leftFront.getDirection(),
                mecanumDrive.rightFront.getDirection());
        telemetry.addData("Back Motors", "LB: %s    RB: %s",
                mecanumDrive.leftBack.getDirection(),
                mecanumDrive.rightBack.getDirection());
        telemetry.addLine(String.format("Power: LF(%.2f), LB(%.2f)",
                mecanumDrive.leftFront.getPower(), mecanumDrive.leftBack.getPower()));
        telemetry.addLine(String.format("Power: RF(%.2f), RB(%.2f)",
                mecanumDrive.rightFront.getPower(), mecanumDrive.rightBack.getPower()));
        telemetry.addLine();
        // Speed for each motor
                telemetry.addLine(String.format("Speed: LF(%d/%d)",
                        (int) mecanumDrive.leftFront.getVelocity(), (int) leftFrontTargetSpeed));
                telemetry.addLine(String.format("Speed: LB(%d/%d)",
                        (int) mecanumDrive.leftBack.getVelocity(), (int) leftBackTargetSpeed));
                telemetry.addLine(String.format("Speed: RF(%d/%d)",
                        (int) mecanumDrive.rightFront.getVelocity(), (int) rightFrontTargetSpeed));
                telemetry.addLine(String.format("Speed: RB(%d/%d)",
                        (int) mecanumDrive.rightBack.getVelocity(), (int) rightBackTargetSpeed));
    }


    // New method to display verbose encoder telemetry
    @SuppressLint("DefaultLocale")
    public void displayVerboseEncodersTelemetry(Telemetry telemetry) {
        MecanumDrive mecanumDrive = Robot.getInstance().getDriveSubsystem().getMecanumDrive();
        telemetry.addLine();
        // Motor Encoder Directions Header
        telemetry.addLine("Motor Encoder Directions");
        telemetry.addData("Front Encoders", "LF: %s    RF: %s",
                leftFrontEncoder.getDirection(),
                rightFrontEncoder.getDirection());
        telemetry.addData("Back Encoders", "LB: %s    RB: %s",
                leftBackEncoder.getDirection(),
                rightBackEncoder.getDirection());
        telemetry.addLine();

        // Motor Encoder Ticks
        telemetry.addLine("Motor Encoder Ticks");
        telemetry.addData("Front Encoder Ticks", "LF: %d    RF: %d",
                mecanumDrive.leftFront.getCurrentPosition(),
                mecanumDrive.rightFront.getCurrentPosition());
        telemetry.addData("Back Encoder Ticks", "LB: %d    RB: %d",
                mecanumDrive.leftBack.getCurrentPosition(),
                mecanumDrive.rightBack.getCurrentPosition());

        telemetry.addLine();
        if (mecanumDrive instanceof PinpointDrive) {
            PinpointDrive.Params pinpointParams = PinpointDrive.PARAMS;

            double roadrunnerPoseHeadingDegrees = Math.toDegrees(mecanumDrive.pose.heading.log());

            telemetry.addLine("Pinpoint Dead Wheels:");
            telemetry.addLine(String.format("(X: %.1f, Y: %.1f, Pinpoint: %.1f°)",
                    mecanumDrive.pose.position.x, mecanumDrive.pose.position.y, roadrunnerPoseHeadingDegrees));

            telemetry.addData("Direction", "Par(%s), Perp(%s)",
                    pinpointParams.xDirection, pinpointParams.yDirection);

            telemetry.addData("Ticks", "Par(%d), Perp(%d)",
                    ((PinpointDrive) mecanumDrive).pinpoint.getEncoderX(),
                    ((PinpointDrive) mecanumDrive).pinpoint.getEncoderY());

            // Add ticks if the parallel and perpendicular encoders are available in the pinpoint params
            telemetry.addData("Velocities", "Par(%d), Perp(%d)",
                    (int)  ((PinpointDrive) mecanumDrive).pinpoint.getVelocity().getX(DistanceUnit.INCH),
                    (int)  ((PinpointDrive) mecanumDrive).pinpoint.getVelocity().getY(DistanceUnit.INCH));

        } else if (mecanumDrive.localizer instanceof TwoDeadWheelLocalizer) {
            // Assuming these are from TwoDeadWheelLocalizer, retrieve and log directions
            TwoDeadWheelLocalizer localizer = (TwoDeadWheelLocalizer) mecanumDrive.localizer;

            telemetry.addData("Hub Dead Wheel Directions", "Parallel (%s), Perpendicular (%s)",
                    localizer.par.getDirection(), localizer.perp.getDirection());

            telemetry.addData("Hub Dead Wheel Ticks", "Parallel (%d), Perpendicular (%d)",
                    (int) localizer.par.getPositionAndVelocity().position,  (int) localizer.perp.getPositionAndVelocity().position);

            telemetry.addData("Hub Dead Wheel Velocities", "Parallel (%d), Perpendicular (%d)",
                    (int) localizer.par.getPositionAndVelocity().velocity,  (int) localizer.perp.getPositionAndVelocity().velocity);

        }
    }

    // Display IMU absolute yaw, FTC field yaw, and RoadRunner pose heading
    public void displayYawTelemetry(Telemetry telemetry) {
        // 1. IMU absolute yaw (in degrees)
        double imuYawDegrees = getInternalIMUYawDegrees();  // IMU's yaw from MecanumDrive

        // 2. IMU offset yaw (in degrees)
        double imuYawDegreesOffset = getInternalIMUYawDegreesWithOffsetApplied();

        // 3. RoadRunner pose heading (in degrees)
        double roadrunnerPoseHeadingDegrees = Math.toDegrees(mecanumDrive.pose.heading.log());

        // Display telemetry data
        telemetry.addData("IMU (Degrees)", "%.1f", imuYawDegrees);
        telemetry.addData("IMU w/ Offset (Degrees)", "%.1f", imuYawDegreesOffset);
        telemetry.addData("RoadRunner Pose Heading (Degrees)", "%.1f", roadrunnerPoseHeadingDegrees);

        // Optionally display any difference between field convention yaw and RoadRunner heading
        double yawDifference = imuYawDegreesOffset - roadrunnerPoseHeadingDegrees;
        telemetry.addData("IMU vs RoadRunner Yaw Difference", yawDifference);
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
        return normalizeAngle(angles.getYaw(AngleUnit.DEGREES) - yawOffsetDegrees);  // Return yaw in degrees
    }

    public void updateIMU() {
        angles = mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles(); // Access IMU
    }

    public TrajectoryActionBuilder mirroredActionBuilder(Pose2d beginPose) {
        Pose2d rotatedStartingPose = new Pose2d(
                -beginPose.position.x,
                -beginPose.position.y,
                beginPose.heading.inverse().log()
        );

        return new TrajectoryActionBuilder(
                (TimeTurn turn) -> mecanumDrive.new TurnAction(turn),
                (TimeTrajectory trajectory) -> mecanumDrive.new FollowTrajectoryAction(trajectory),
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(.25, .1, 1e-2
                        )
                ),
                rotatedStartingPose, 0.0,
                Robot.getInstance().getDriveSubsystem().getMecanumDrive().defaultTurnConstraints,
                Robot.getInstance().getDriveSubsystem().getMecanumDrive().defaultVelConstraint,
                Robot.getInstance().getDriveSubsystem().getMecanumDrive().defaultAccelConstraint,
                pose -> new Pose2dDual<>(
                        pose.position.x.unaryMinus(), pose.position.y.unaryMinus(), pose.heading.inverse()));
    }

    public void setMotorPower (double lF, double rF, double lB, double rB){
        mecanumDrive.leftFront.setPower(lF);
        mecanumDrive.rightFront.setPower(rF);
        mecanumDrive.leftBack.setPower(lB);
        mecanumDrive.rightBack.setPower(rB);
    }

    // ========== Helper/Utility Methods ==========
    private double Ramp(double target, double currentValue, double ramp_amount) {
        if (Math.abs(currentValue) + RAMP_PARAMS.RAMP_THRESHOLD < Math.abs(target)) {
            return Math.signum(target) * (Math.abs(currentValue) + ramp_amount);
        }  else
        {
            return target;
        }
    }

    public static class TeleopParams {

        public enum TeleopMode {
            DEFAULT,
            POWER_TEST,
            HIGH_SPEED
        }

        public static class StickParams {

            public double DEAD_ZONE = 0.2;
            public double DRIVE_SPEED_FACTOR = 0.82;
            public double STRAFE_SPEED_FACTOR = 1.0;
            public double TURN_SPEED_FACTOR = 1.0;
        }

        public static class RampParams {
            public double DRIVE_RAMP = 0.2;
            public double STRAFE_RAMP = 0.22;
            public double TURN_RAMP = 0.4;
            public double RAMP_THRESHOLD = 0.04;
        }

        public static class PIDParams {
            public double P = 0;
            public double D = 0;
            public double I = 0;
            public double F = 8;
        }

        // Inner class for Power Test configuration
        public static class TeleopPowerTestParams extends TeleopParams {
            public static double DRIVE_SPEED_FACTOR = 1.0;
            public static double STRAFE_SPEED_FACTOR = 1.0;
            public static double TURN_SPEED_FACTOR = 1.0;
            public static double DEAD_ZONE = 0.2;
            public static double DRIVE_RAMP = 0;
            public static double STRAFE_RAMP = 0;
            public static double TURN_RAMP = 0;
            public static double RAMP_THRESHOLD = 0;
            public static double P = 0;
            public static double D = 0;
            public static double I = 0;
            public static double F = 0;

            public TeleopPowerTestParams() {
                // Constructor can initialize specific values if needed, but static values should be used
            }
        }

        // Inner class for High Speed Teleop
        public static class HighSpeedTeleopParams extends TeleopParams {
            public static double DRIVE_SPEED_FACTOR = 1.2;
            public static double STRAFE_SPEED_FACTOR = 1.1;
            public static double TURN_SPEED_FACTOR = 1.1;
            public static double DEAD_ZONE = 0.15;
            public static double DRIVE_RAMP = 0.25;
            public static double STRAFE_RAMP = 0.25;
            public static double TURN_RAMP = 0.45;
            public static double RAMP_THRESHOLD = 0.05;
            public static double P = 0;
            public static double D = 0;
            public static double I = 0;
            public static double F = 10;

            public HighSpeedTeleopParams() {
                // Constructor can initialize specific values if needed, but static values should be used
            }
        }

        // Method to choose which params to use based on the enum
        public static TeleopParams getParamsFor(TeleopMode mode) {
            switch (mode) {
                case POWER_TEST:
                    return new TeleopPowerTestParams();
                case HIGH_SPEED:
                    return new HighSpeedTeleopParams();
                default:
                    return new TeleopParams();  // Return default if no match
            }
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
                case RR_SET_DRIVE_POWER:
                case POWER_WITH_ENCODERS:
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

            case RR_SET_DRIVE_POWER:
                rrDriveControl(drive, strafe, turn);
                break;

            case POWER_WITH_ENCODERS:
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
        if (drive==0 && strafe ==0 && turn==0) {
            //if power is not set to zero its jittery, doesn't work at all if we don't reset the motors back to run using encoders...
            leftFrontTargetSpeed=0;
            rightFrontTargetSpeed=0;
            leftBackTargetSpeed=0;
            rightBackTargetSpeed=0;

            mecanumDrive.leftFront.setVelocity(leftFrontTargetSpeed);
            mecanumDrive.leftBack.setVelocity(leftBackTargetSpeed);
            mecanumDrive.rightFront.setVelocity(rightFrontTargetSpeed);
            mecanumDrive.rightBack.setVelocity(rightBackTargetSpeed);

            mecanumDrive.leftFront.setPower(0);
            mecanumDrive.leftBack.setPower(0);
            mecanumDrive.rightFront.setPower(0);
            mecanumDrive.rightBack.setPower(0);

            current_drive_ramp=0;
            current_strafe_ramp=0;
            current_turn_ramp=0;
        } else
        {
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

            mecanumDrive.leftFront.setVelocity(leftFrontTargetSpeed);
            mecanumDrive.rightFront.setVelocity(rightFrontTargetSpeed);
            mecanumDrive.leftBack.setVelocity(leftBackTargetSpeed);
            mecanumDrive.rightBack.setVelocity(rightBackTargetSpeed);
        }

        mecanumDrive.updatePoseEstimate();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), mecanumDrive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void mecanumDrivePowerControl (double drive, double strafe, double turn){
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

        mecanumDrive.updatePoseEstimate();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), mecanumDrive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void rrDriveControl(double left_stick_y, double left_stick_x, double right_stick_x) {
        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        left_stick_y,
                        -left_stick_x
                ),
                -right_stick_x
        ));

        mecanumDrive.updatePoseEstimate();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), mecanumDrive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private void initializeMotorEncoders() {
        leftFrontEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.leftFront));
        leftBackEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.leftBack));
        rightBackEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.rightBack));
        rightFrontEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.rightFront));
    }

}





