package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveClasses;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class MecanumDriveMona extends MecanumDrive  {

    private MonaTeleopParams MONA_PARAMS;

    private double drive, strafe, turn;
    private double last_drive=0, last_strafe=0, last_turn=0;
    private double current_drive_ramp = 0, current_strafe_ramp=0, current_turn_ramp=0;
    private double leftFrontTargetSpeed, rightFrontTargetSpeed, leftBackTargetSpeed, rightBackTargetSpeed;

    public MecanumDriveMona(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
    }

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

    public static class ChassisInternalIMUParams extends MonaTeleopParams {
        public ChassisInternalIMUParams() {
            RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD ;
            RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                    RevHubOrientationOnRobot.UsbFacingDirection.UP;

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

    public static class CenterStageDEAD_WHEEL_Params extends MonaTeleopParams {
        public CenterStageDEAD_WHEEL_Params() {
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

    public void init() {
        //TODO This should override the Roadrunner parameters depending on the robot type set in the OpMode.
        switch (Robot.getInstance().robotType) {
            //Override the Roadrunner parameters for the chassis bot and the centerstage robot
            //The normal IntoTheDeep robot parameters should be stored in the MecancumDrive class
            case ROBOT_CHASSIS_INTERNAL_IMU:
                PARAMS = new ChassisInternalIMUParams();
                break;
            case ROBOT_CENTERSTAGE_DEAD_WHEEL_INTERNAL_IMU:
                PARAMS = new CenterStageDEAD_WHEEL_Params();
                break;
            default:
                PARAMS = new MonaTeleopParams();
                break;
        }


        //By casting PARAMS as MonaParms we can use the same parameter object but access the parameters we added in this class.
        MONA_PARAMS = (MonaTeleopParams) PARAMS;  // Cast to MonaParams

        //set the PID values one time
        leftFront.setVelocityPIDFCoefficients(MONA_PARAMS.P, MONA_PARAMS.I, MONA_PARAMS.D, MONA_PARAMS.F);
        rightFront.setVelocityPIDFCoefficients(MONA_PARAMS.P, MONA_PARAMS.I, MONA_PARAMS.D, MONA_PARAMS.F);
        leftBack.setVelocityPIDFCoefficients(MONA_PARAMS.P, MONA_PARAMS.I, MONA_PARAMS.D, MONA_PARAMS.F);
        rightBack.setVelocityPIDFCoefficients(MONA_PARAMS.P, MONA_PARAMS.I, MONA_PARAMS.D, MONA_PARAMS.F);
    }

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

            leftFrontTargetSpeed = MONA_PARAMS.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightFrontTargetSpeed = MONA_PARAMS.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));
            leftBackTargetSpeed = MONA_PARAMS.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightBackTargetSpeed = MONA_PARAMS.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));

            leftFront.setVelocity(leftFrontTargetSpeed);
            rightFront.setVelocity(rightFrontTargetSpeed);
            leftBack.setVelocity(leftBackTargetSpeed);
            rightBack.setVelocity(rightBackTargetSpeed);

            last_drive=drive;
            last_strafe=strafe;
            last_turn=turn;
        }
    }

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

//    public void fieldOrientedControl (double leftY, double leftX){
//        double y = leftY;
//        double x = leftX;
//        double botHeading;
//
//        //This should make it so field centric driving works for both alliance colors
//        if (MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED) {
//            botHeading = Math.toRadians(Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees - 90);
//        } else {
//            botHeading = Math.toRadians(Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees + 90);
//        }
//
//        // Rotate the movement direction counter to the bot's rotation
//        leftXAdjusted = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        leftYAdjusted = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        leftYAdjusted = Math.min( leftYAdjusted * 1.1, 1);  // Counteract imperfect strafing
//    }

    public TrajectoryActionBuilder mirroredActionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
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
                        pose.position.x.unaryMinus(), pose.position.y.unaryMinus(), pose.heading.inverse()));
    }

    public void LoopDriverStationTelemetry(Telemetry telemetry) {
        telemetry.addData("Alliance Color: ", MatchConfig.finalAllianceColor);
        telemetry.addData("Side of Field: ", MatchConfig.finalSideOfField);
        telemetry.addLine("TeleOp Time " + JavaUtil.formatNumber(MatchConfig.teleOpTimer.seconds(), 4, 1) + " / 120 seconds");
        telemetry.addData("Loop Time ", JavaUtil.formatNumber(MatchConfig.loopTimer.milliseconds(), 4, 1) + " milliseconds");

        telemetry.addLine();
        telemetry.addData("Current Pose", "X %5.2f, Y %5.2f, heading %5.2f ",
                Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.x,
                Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.y,
                Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.heading.log());

        telemetry.addLine();

        Robot.getInstance().getActiveOpMode().telemetry.addLine("Yaw Angle Absolute (Degrees)" + JavaUtil.formatNumber(Robot.getInstance().getGyroSubsystem().currentAbsoluteYawDegrees, 5, 2));
        Robot.getInstance().getActiveOpMode().telemetry.addLine("Yaw Angle Relative (Degrees)" + JavaUtil.formatNumber(Robot.getInstance().getGyroSubsystem().currentRelativeYawDegrees, 5, 2));
    }
}


