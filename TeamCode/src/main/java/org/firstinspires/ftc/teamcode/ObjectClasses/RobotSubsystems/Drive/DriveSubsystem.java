package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

public class DriveSubsystem extends SubsystemBase {

    @Config
    public static class TeleopParams  {
        /** Set these drive parameters for faster TeleOp driving**/
        public double DRIVE_SPEED_FACTOR=.7;
        public double STRAFE_SPEED_FACTOR=.7;
        public double TURN_SPEED_FACTOR=.8;
        public double DEAD_ZONE = .2;

        public double DRIVE_RAMP = .2;
        public double STRAFE_RAMP = .22;
        public double TURN_RAMP = .4;
        public double RAMP_THRESHOLD = .04;

        public double P = 0;
        public double D = 0;
        public double I = 3;
        public double F = 8;
    }

    public static class ChassisTwoDeadWheelInternalIMUParams extends MecanumDrive.Params {
        public ChassisTwoDeadWheelInternalIMUParams() {
            logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD ;
            usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

            // drive model parameters
            this.inPerTick = 0.002996;
            //TODO: update this lateralInPerTick based on lateralRampLogger
            this.lateralInPerTick = 0.0023321641061384555;
            this. trackWidthTicks = 4410.968285794866;

            // feedforward parameters (in tick units)
            this.kS = 1.1061809171010473;
            this.kV = 0.0003917967378464592;
            this.kA = 0.00001;

            // path profile parameters (in inches)
            this.maxWheelVel = 50;
            this.minProfileAccel = -30;
            this.maxProfileAccel = 50;

            // turn profile parameters (in radians)
            this.maxAngVel = Math.PI; // shared with path
            this.maxAngAccel = Math.PI;

            // path controller gains
            this.axialGain = 10;
            this. lateralGain = 10;
            this.headingGain = 8; // shared with turn

            this.axialVelGain = 1.5;
            this.lateralVelGain = 0;
            this. headingVelGain = 1.1;
        }
    }

    public static class CenterStageTwoDeadWheelInternalIMUParams extends MecanumDrive.Params {
        public CenterStageTwoDeadWheelInternalIMUParams() {
            this.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD ;
            this.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

            this.inPerTick = 0.002996;
            this.lateralInPerTick = inPerTick; // inPerTick;
            this.trackWidthTicks = 0;

            this.kS = 1.3635356937629455;
            this.kV = 0.00038558904047469167;
            this.kA = 0.0;

            // path profile parameters (in inches)
            this.maxWheelVel = 50;
            this.minProfileAccel = -30;
            this.maxProfileAccel = 50;

            // turn profile parameters (in radians)
            this.maxAngVel = Math.PI; // shared with path
            this.maxAngAccel = Math.PI;

            // path controller gains
            this.axialGain = 0.0;
            this.lateralGain = 0.0;
            this.headingGain = 0.0; // shared with turn

            this.axialVelGain = 0.0;
            this.lateralVelGain = 0.0;
            this.headingVelGain = 0.0; // shared with turn
        }
    }

    public static class ChassisPinpointParams extends PinpointDrive.Params {
        public ChassisPinpointParams() {
            this.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD ;
            this.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

            this.inPerTick = 1;
            this.lateralInPerTick = inPerTick; // inPerTick;
            this.trackWidthTicks = 0;

            this.kS = 0.0;
            this.kV = 0.0;
            this.kA = 0.0;

            // path profile parameters (in inches)
            this.maxWheelVel = 50;
            this.minProfileAccel = -30;
            this.maxProfileAccel = 50;

            // turn profile parameters (in radians)
            this.maxAngVel = Math.PI; // shared with path
            this.maxAngAccel = Math.PI;

            // path controller gains
            this.axialGain = 0.0;
            this.lateralGain = 0.0;
            this.headingGain = 0.0; // shared with turn

            this.axialVelGain = 0.0;
            this.lateralVelGain = 0.0;
            this.headingVelGain = 0.0; // shared with turn
        }
    }

    public static class ChassisTwoDeadWheelInternalIMULocalizerParams extends TwoDeadWheelLocalizer.Params {
        public ChassisTwoDeadWheelInternalIMULocalizerParams() {
            // Override parYTicks and perpXTicks with values specific to ChassisInternalIMU
            this.parYTicks = -1450.0;
            this.perpXTicks = 800.0;
        }
    }

    public static class CenterStageTwoDeadWheelInternalIMULocalizerParams extends TwoDeadWheelLocalizer.Params {
        public CenterStageTwoDeadWheelInternalIMULocalizerParams() {
            // Override parYTicks and perpXTicks with values specific to CenterStageDEAD_WHEEL
            this.parYTicks = -1440.0;
            this.perpXTicks = 820.0;
        }
    }

    // Instance of MecanumDrive (or other types like PinpointDrive)
    private MecanumDrive mecanumDrive;
    public TeleopParams TELEOP_PARAMS;
    private MecanumDrive.Params PARAMS;

    public boolean fieldOrientedControl;
    public double yawOffset;  // offset depending on alliance color

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

    // Constants for motor ticks, wheel diameter, and gear ratio
    private static final double WHEEL_DIAMETER_INCHES = 4.0;  // Example wheel diameter in inches
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    private static final double GEAR_RATIO = 1.0;  // Adjust if you have a gear ratio

    public DriveSubsystem(HardwareMap hardwareMap) {
        TELEOP_PARAMS = new TeleopParams();
        // Initialize appropriate drive system based on robot type
        switch (Robot.getInstance().robotType) {

            case ROBOT_CHASSIS_TWO_DEAD_WHEEL_INTERNAL_IMU:
                mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                // After initializing the drive, override the parameters with your custom ones
                PARAMS = new ChassisTwoDeadWheelInternalIMUParams();  // Custom params for this robot type
                MecanumDrive.PARAMS = PARAMS;  // Set the static PARAMS field to your custom one

                // Set up localizer using the custom PARAMS
                mecanumDrive.localizer = new TwoDeadWheelLocalizer(hardwareMap, mecanumDrive.lazyImu.get(), PARAMS.inPerTick);
                TwoDeadWheelLocalizer.PARAMS = new ChassisTwoDeadWheelInternalIMULocalizerParams();

                setMotorAndEncoderDirectionsForChassisTwoDeadWheelInternalIMU();
                break;

            case ROBOT_CENTERSTAGE_TWO_DEAD_WHEEL_INTERNAL_IMU:
                mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                // After initializing the drive, override the parameters with your custom ones
                PARAMS = new CenterStageTwoDeadWheelInternalIMUParams();
                MecanumDrive.PARAMS = PARAMS;  // Set the static PARAMS field to your custom one

                // Set up localizer using the custom PARAMS
                this.mecanumDrive.localizer = new TwoDeadWheelLocalizer(hardwareMap, mecanumDrive.lazyImu.get(), MecanumDrive.PARAMS.inPerTick );
                TwoDeadWheelLocalizer.PARAMS = new CenterStageTwoDeadWheelInternalIMULocalizerParams();

                setMotorAndEncoderDirectionsForCenterStageTwoDeadWheelInternalIMU();
                break;

            case ROBOT_CHASSIS_PINPOINT:
                this.mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

                // After initializing the drive, override the parameters with your custom ones
                PARAMS = new ChassisPinpointParams();
                MecanumDrive.PARAMS = PARAMS;  // Set the static PARAMS field to your custom one

                setMotorAndEncoderDirectionsForChassisPinpoint();
                break;

            case ROBOT_CENTERSTAGE_OTOS:
                this.mecanumDrive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
                break;
        }
        configurePID();
    }

    public void init()
    {
        Robot.getInstance().registerSubsystem(Robot.SubsystemType.DRIVE);
        fieldOrientedControl=true; // Default to field-oriented control
        CalculateYawOffset();

    }

    private void CalculateYawOffset() {
        // Calculate yaw offset based on alliance color
        // This offset assumes robot forward direction faces away from driver
        //      -For red start, the audience is on your left
        //      -For blue start, the audience is on your right
        //  See this: https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
        yawOffset=0; // I think we actually don't need an offset this year
//        if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
//            yawOffset = Math.toRadians(90);  // 90 degrees for blue side
//        } else {
//            yawOffset = Math.toRadians(-90);  // -90 degrees for red side
//        }
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
            if (fieldOrientedControl) {
                fieldOrientedControl(leftY, leftX);
                rightXAdjusted = rightX * TELEOP_PARAMS.TURN_SPEED_FACTOR;
            } else {
                // Apply speed factors
                leftYAdjusted = leftY * TELEOP_PARAMS.DRIVE_SPEED_FACTOR;
                leftXAdjusted = leftX * TELEOP_PARAMS.STRAFE_SPEED_FACTOR;
                rightXAdjusted = rightX * TELEOP_PARAMS.TURN_SPEED_FACTOR;
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
        double averageSpeedInchesPerSecond = (averageTicksPerSecond / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE * GEAR_RATIO;

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


    // New method to display verbose encoder telemetry
    public void displayVerboseEncodersTelemetry(Telemetry telemetry) {
        MecanumDrive mecanumDrive = Robot.getInstance().getDriveSubsystem().getMecanumDrive();
        TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) mecanumDrive.localizer;

        // Log motor power and encoder positions
        telemetry.addData("Motor Power", "LF (%.2f), LB (%.2f), RF (%.2f), RB (%.2f)",
                mecanumDrive.leftFront.getPower(), mecanumDrive.leftBack.getPower(),
                mecanumDrive.rightFront.getPower(), mecanumDrive.rightBack.getPower());

        telemetry.addData("Motor Encoder Positions", "LF (%d), LB (%d), RF (%d), RB (%d)",
                mecanumDrive.leftFront.getCurrentPosition(), mecanumDrive.leftBack.getCurrentPosition(),
                mecanumDrive.rightFront.getCurrentPosition(), mecanumDrive.rightBack.getCurrentPosition());

        telemetry.addData("Motor Encoder Velocities", "LF (%d), LB (%d), RF (%d), RB (%d)",
                mecanumDrive.leftFront.getVelocity(), mecanumDrive.leftBack.getVelocity(),
                mecanumDrive.rightFront.getVelocity(), mecanumDrive.rightBack.getVelocity());

        // Log dead wheel encoder positions
        telemetry.addData("Dead Wheel Encoder Positions", "Parallel (%d), Perpendicular (%d)",
                dl.par.getPositionAndVelocity().position, dl.perp.getPositionAndVelocity().position);

        // Log dead wheel encoder velocities
        telemetry.addData("Dead Wheel Encoder Velocities", "Parallel (%d), Perpendicular (%d)",
                dl.par.getPositionAndVelocity().velocity, dl.perp.getPositionAndVelocity().velocity);

        // Log dead wheel encoder directions
        telemetry.addData("Dead Wheel Encoder Directions", "Parallel (%d), Perpendicular (%d)",
                dl.par.getDirection(), dl.perp.getDirection());
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
        return (Math.abs(leftY) > TELEOP_PARAMS.DEAD_ZONE ||
                Math.abs(leftX) > TELEOP_PARAMS.DEAD_ZONE ||
                Math.abs(rightX) > TELEOP_PARAMS.DEAD_ZONE);
    }

    public void configurePID() {
        // Set PID values for motors
        mecanumDrive.leftFront.setVelocityPIDFCoefficients(TELEOP_PARAMS.P, TELEOP_PARAMS.I, TELEOP_PARAMS.D, TELEOP_PARAMS.F);
        mecanumDrive.rightFront.setVelocityPIDFCoefficients(TELEOP_PARAMS.P, TELEOP_PARAMS.I, TELEOP_PARAMS.D, TELEOP_PARAMS.F);
        mecanumDrive.leftBack.setVelocityPIDFCoefficients(TELEOP_PARAMS.P, TELEOP_PARAMS.I, TELEOP_PARAMS.D, TELEOP_PARAMS.F);
        mecanumDrive.rightBack.setVelocityPIDFCoefficients(TELEOP_PARAMS.P, TELEOP_PARAMS.I, TELEOP_PARAMS.D, TELEOP_PARAMS.F);
    }
    // Helper methods to set motor and dead wheel encoder directions
    private void setMotorAndEncoderDirectionsForChassisTwoDeadWheelInternalIMU() {
        //set motor directions
        mecanumDrive.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        mecanumDrive.rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Set motor modes to reset encoders
        mecanumDrive.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders (important for velocity control)
        mecanumDrive.leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive. leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive.rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive.rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Set up motor encoders
        Encoder leftFrontEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.leftFront));
        Encoder leftBackEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.leftBack));
        Encoder rightBackEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.rightBack));
        Encoder rightFrontEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.rightFront));

        //  reverse encoders if needed - overriding these
        leftFrontEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        //This changes the dead wheel encoder directions
        ((TwoDeadWheelLocalizer) mecanumDrive.localizer).par.setDirection(DcMotorSimple.Direction.REVERSE);
        ((TwoDeadWheelLocalizer) mecanumDrive.localizer).perp.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setMotorAndEncoderDirectionsForCenterStageTwoDeadWheelInternalIMU() {
        mecanumDrive.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        mecanumDrive.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        mecanumDrive.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumDrive.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        ((TwoDeadWheelLocalizer) mecanumDrive.localizer).par.setDirection(DcMotorSimple.Direction.REVERSE);
        ((TwoDeadWheelLocalizer) mecanumDrive.localizer).perp.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setMotorAndEncoderDirectionsForChassisPinpoint() {
        //set motor directions
        mecanumDrive.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        mecanumDrive.rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Set motor modes to reset encoders
        mecanumDrive.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders (important for velocity control)
        mecanumDrive.leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive. leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive.rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanumDrive.rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Set up motor encoders
        Encoder leftFrontEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.leftFront));
        Encoder leftBackEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.leftBack));
        Encoder rightBackEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.rightBack));
        Encoder rightFrontEncoder = new OverflowEncoder(new RawEncoder(mecanumDrive.rightFront));

        //  reverse encoders if needed - overriding these
        leftFrontEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }



    /**
     * Method to get current yaw (heading) in degrees from the IMU.
     */
    public double getYawDegrees() {
        YawPitchRollAngles angles = mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles(); // Access IMU
        return angles.getYaw(AngleUnit.DEGREES);  // Return yaw in degrees
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




    public void setAllPower(double p) {setMotorPower(p,p,p,p);}

    public void setMotorPower (double lF, double rF, double lB, double rB){
        mecanumDrive.leftFront.setPower(lF);
        mecanumDrive.rightFront.setPower(rF);
        mecanumDrive.leftBack.setPower(lB);
        mecanumDrive.rightBack.setPower(rB);
    }


    public void rrDriveControlWithFeedback(double left_stick_y, double left_stick_x, double right_stick_x) {
        MecanumDrive drive = Robot.getInstance().getDriveSubsystem().getMecanumDrive();
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;

        // Calculate the desired wheel velocities using RoadRunner kinematics
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(new PoseVelocity2d(
                        new Vector2d(-left_stick_y, -left_stick_x),
                        -right_stick_x
                ), 1)
        );

        // Set target velocities using the motor's built-in PID controller
        drive.leftFront.setVelocity(wheelVels.leftFront.get(0));
        drive.leftBack.setVelocity(wheelVels.leftBack.get(0));
        drive.rightFront.setVelocity(wheelVels.rightFront.get(0));
        drive.rightBack.setVelocity(wheelVels.rightBack.get(0));

        // Update the robot's pose estimate
        drive.updatePoseEstimate();

        // Send telemetry for real-time feedback
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        // Send feedback to the dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    // ========== Helper/Utility Methods ==========
    private double Ramp(double target, double currentValue, double ramp_amount) {
        if (Math.abs(currentValue) + TELEOP_PARAMS.RAMP_THRESHOLD < Math.abs(target)) {
            return Math.signum(target) * (Math.abs(currentValue) + ramp_amount);
        }  else
        {
            return target;
        }
    }

    // ========== Main Logic/Control Methods ==========

    public void mecanumDriveSpeedControl(double drive, double strafe, double turn) {
        if (drive==0 && strafe ==0 && turn==0) {
            //if power is not set to zero its jittery, doesn't work at all if we don't reset the motors back to run using encoders...
            mecanumDrive.leftFront.setVelocity(0);
            mecanumDrive.leftBack.setVelocity(0);
            mecanumDrive.rightFront.setVelocity(0);
            mecanumDrive.rightBack.setVelocity(0);

            mecanumDrive.leftFront.setPower(0);
            mecanumDrive.leftBack.setPower(0);
            mecanumDrive.rightFront.setPower(0);
            mecanumDrive.rightBack.setPower(0);

            current_drive_ramp=0;
            current_strafe_ramp=0;
            current_turn_ramp=0;

        } else
        {
            Robot.getInstance().getDriveSubsystem().mecanumDrive.updatePoseEstimate();

            //If we see blue tags and we are red and we are driving toward them, then use the safetydrivespeedfactor to slow us down
            current_drive_ramp = Ramp(drive, current_drive_ramp, TELEOP_PARAMS.DRIVE_RAMP);
            current_strafe_ramp = Ramp(strafe, current_strafe_ramp, TELEOP_PARAMS.STRAFE_RAMP);
            current_turn_ramp = Ramp(turn, current_turn_ramp, TELEOP_PARAMS.TURN_RAMP);

            double dPercent = abs(current_drive_ramp) / (abs(current_drive_ramp) + abs(current_strafe_ramp) + abs(current_turn_ramp));
            double sPercent = abs(current_strafe_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));
            double tPercent = abs(current_turn_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));

            double leftFrontTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            double rightFrontTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));
            double leftBackTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            double rightBackTargetSpeed = MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));

            mecanumDrive.leftFront.setVelocity(leftFrontTargetSpeed);
            mecanumDrive.rightFront.setVelocity(rightFrontTargetSpeed);
            mecanumDrive.leftBack.setVelocity(leftBackTargetSpeed);
            mecanumDrive.rightBack.setVelocity(rightBackTargetSpeed);
        }
    }

    public void mecanumDrivePowerControl (){
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
        MecanumDrive drive = Robot.getInstance().getDriveSubsystem().getMecanumDrive();
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -left_stick_y,
                        -left_stick_x
                ),
                -right_stick_x
        ));

        drive.updatePoseEstimate();

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}


