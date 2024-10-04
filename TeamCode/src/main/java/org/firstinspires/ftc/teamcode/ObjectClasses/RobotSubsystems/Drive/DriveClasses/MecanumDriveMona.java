package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveClasses;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

/**
 * MecanumDriveMona class for handling different robot configurations (Chassis, CenterStage).
 * Extends MecanumDrive and provides robot-specific configurations for motor directions,
 * RoadRunner parameters, and speed control.
 */
public class MecanumDriveMona extends MecanumDrive  {

    // Extend Params to include Mona-specific speed parameters
    public static class MonaTeleopParams extends MecanumDrive.Params {
        public double MAX_MOTOR_SPEED_RPS = 435.0 / 60.0;
        public double TICKS_PER_REV = 384.5;
        public double MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;

        public double DRIVE_RAMP = .2;
        public double STRAFE_RAMP = .22;
        public double TURN_RAMP = .4;
        public double RAMP_THRESHOLD = .04;

        public double P = 0;
        public double D = 0;
        public double I = 3;
        public double F = 8;
    }

    // Getter method to expose MonaTeleopParams
    public MonaTeleopParams getMonaTeleopParams() {
        return MONA_PARAMS;
    }

    public static class ChassisTwoDeadWheelInternalIMUParams extends MonaTeleopParams {
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

    public static class CenterStageTwoDeadWheelInternalIMUParams extends MonaTeleopParams {
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

    // ========== Instance Variables ==========
    private MonaTeleopParams MONA_PARAMS;
    private double drive, strafe, turn;
    private double last_drive=0, last_strafe=0, last_turn=0;
    private double current_drive_ramp = 0, current_strafe_ramp=0, current_turn_ramp=0;

    // ========== Constructor ==========
    public MecanumDriveMona(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
    }

    public void init() {
        //TODO This should override the Roadrunner parameters depending on the robot type set in the OpMode.
        switch (Robot.getInstance().robotType) {
            //Override the Roadrunner parameters for the chassis bot and the centerstage robot
            //The normal IntoTheDeep robot parameters should be stored in the MecancumDrive class

            case ROBOT_CHASSIS_TWO_DEAD_WHEEL_INTERNAL_IMU:

                PARAMS = new ChassisTwoDeadWheelInternalIMUParams();
                TwoDeadWheelLocalizer.PARAMS = new ChassisTwoDeadWheelInternalIMULocalizerParams();
                setMotorAndEncoderDirectionsForChassisTwoDeadWheelInternalIMU();
                break;

            case ROBOT_CENTERSTAGE_TWO_DEAD_WHEEL_INTERNAL_IMU:
                PARAMS = new CenterStageTwoDeadWheelInternalIMUParams();
                TwoDeadWheelLocalizer.PARAMS = new CenterStageTwoDeadWheelInternalIMULocalizerParams();
                setMotorAndEncoderDirectionsForCenterStageTwoDeadWheelInternalIMU();
                break;
        }

        //Cast PARAMS as MonaParms we can use the same parameter object but access the parameters we added in this class.
        MONA_PARAMS = (MonaTeleopParams) PARAMS;  // Cast to MonaParams

        //set the PID values one time
        configurePID();
    }

    public void configurePID() {
        // Set PID values for motors
        leftFront.setVelocityPIDFCoefficients(MONA_PARAMS.P, MONA_PARAMS.I, MONA_PARAMS.D, MONA_PARAMS.F);
        rightFront.setVelocityPIDFCoefficients(MONA_PARAMS.P, MONA_PARAMS.I, MONA_PARAMS.D, MONA_PARAMS.F);
        leftBack.setVelocityPIDFCoefficients(MONA_PARAMS.P, MONA_PARAMS.I, MONA_PARAMS.D, MONA_PARAMS.F);
        rightBack.setVelocityPIDFCoefficients(MONA_PARAMS.P, MONA_PARAMS.I, MONA_PARAMS.D, MONA_PARAMS.F);
    }

    // ========== Main Logic/Control Methods ==========

    public void mecanumDriveSpeedControl(double drive, double strafe, double turn) {
        if (drive==0 && strafe ==0 && turn==0) {
            //if power is not set to zero its jittery, doesn't work at all if we don't reset the motors back to run using encoders...
            leftFront.setVelocity(0);
            leftBack.setVelocity(0);
            rightFront.setVelocity(0);
            rightBack.setVelocity(0);

            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            current_drive_ramp=0;
            current_strafe_ramp=0;
            current_turn_ramp=0;

        } else
        {
            Robot.getInstance().getDriveSubsystem().mecanumDrive.updatePoseEstimate();

            //If we see blue tags and we are red and we are driving toward them, then use the safetydrivespeedfactor to slow us down
            current_drive_ramp = Ramp(drive, current_drive_ramp, MONA_PARAMS.DRIVE_RAMP);
            current_strafe_ramp = Ramp(strafe, current_strafe_ramp, MONA_PARAMS.STRAFE_RAMP);
            current_turn_ramp = Ramp(turn, current_turn_ramp, MONA_PARAMS.TURN_RAMP);

            double dPercent = abs(current_drive_ramp) / (abs(current_drive_ramp) + abs(current_strafe_ramp) + abs(current_turn_ramp));
            double sPercent = abs(current_strafe_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));
            double tPercent = abs(current_turn_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));

            double leftFrontTargetSpeed = MONA_PARAMS.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            double rightFrontTargetSpeed = MONA_PARAMS.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));
            double leftBackTargetSpeed = MONA_PARAMS.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            double rightBackTargetSpeed = MONA_PARAMS.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));

            leftFront.setVelocity(leftFrontTargetSpeed);
            rightFront.setVelocity(rightFrontTargetSpeed);
            leftBack.setVelocity(leftBackTargetSpeed);
            rightBack.setVelocity(rightBackTargetSpeed);

            last_drive=drive;
            last_strafe=strafe;
            last_turn=turn;
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

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
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
        if (Math.abs(currentValue) + MONA_PARAMS.RAMP_THRESHOLD < Math.abs(target)) {
            return Math.signum(target) * (Math.abs(currentValue) + ramp_amount);
        }  else
        {
            return target;
        }
    }

    public void setAllPower(double p) {setMotorPower(p,p,p,p);}

    public void setMotorPower (double lF, double rF, double lB, double rB){
        leftFront.setPower(lF);
        rightFront.setPower(rF);
        leftBack.setPower(lB);
        rightBack.setPower(rB);
    }

    public TrajectoryActionBuilder mirroredActionBuilder(Pose2d beginPose) {
        Pose2d rotatedStartingPose = new Pose2d(
                -beginPose.position.x,
                -beginPose.position.y,
                beginPose.heading.inverse().log()
        );

        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
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

    // Helper methods to set motor and dead wheel encoder directions
    private void setMotorAndEncoderDirectionsForChassisTwoDeadWheelInternalIMU() {
        //set motor directions
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Set motor modes to reset encoders
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders (important for velocity control)
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Set up motor encoders
        Encoder leftFrontEncoder = new OverflowEncoder(new RawEncoder(leftFront));
        Encoder leftBackEncoder = new OverflowEncoder(new RawEncoder(leftBack));
        Encoder rightBackEncoder = new OverflowEncoder(new RawEncoder(rightBack));
        Encoder rightFrontEncoder = new OverflowEncoder(new RawEncoder(rightFront));

        //  reverse encoders if needed - overriding these
        leftFrontEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        //This changes the dead wheel encoder directions
        ((TwoDeadWheelLocalizer) this.localizer).par.setDirection(DcMotorSimple.Direction.REVERSE);
        ((TwoDeadWheelLocalizer) this.localizer).perp.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setMotorAndEncoderDirectionsForCenterStageTwoDeadWheelInternalIMU() {
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        ((TwoDeadWheelLocalizer) this.localizer).par.setDirection(DcMotorSimple.Direction.REVERSE);
        ((TwoDeadWheelLocalizer) this.localizer).perp.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Method to get current yaw (heading) in degrees from the IMU.
     */
    public double getYawDegrees() {
        YawPitchRollAngles angles = lazyImu.get().getRobotYawPitchRollAngles(); // Access IMU
        return angles.getYaw(AngleUnit.DEGREES);  // Return yaw in degrees
    }

}


