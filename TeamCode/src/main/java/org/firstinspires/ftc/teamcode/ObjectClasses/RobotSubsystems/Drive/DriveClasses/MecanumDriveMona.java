package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveClasses;

import static com.example.sharedconstants.FieldConstants.BLUE_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.BLUE_BACKSTAGE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RED_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RED_BACKSTAGE_START_POSE;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.example.sharedconstants.RobotDriveAdapter;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class MecanumDriveMona extends MecanumDrive implements RobotDriveAdapter  {
    private double drive, strafe, turn;
    private double last_drive=0, last_strafe=0, last_turn=0;
    private double current_drive_ramp = 0, current_strafe_ramp=0, current_turn_ramp=0;
    private double leftFrontTargetSpeed, rightFrontTargetSpeed, leftBackTargetSpeed, rightBackTargetSpeed;

    public MecanumDriveMona(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
    }

    @Config
    static class Params {
        // Mona Drive Parameters
        // TODO: tune

        static double MAX_MOTOR_SPEED_RPS = 435.0 / 60.0;
        static double TICKS_PER_REV = 384.5;
        static double MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;

        /** Set Mona motor parameters for faster TeleOp driving**/
        static double DRIVE_RAMP = .2; //ken ramp
        static double STRAFE_RAMP = .22;
        static double TURN_RAMP = .4;

        static double RAMP_THRESHOLD = .04; // This is the threshold at which we just clamp to the target drive/strafe/turn value

        //it looks to me like just using a feedforward of 12.5 gets the actual speed to match the target. The PID doesn't seem to really do anything.
        static double P =0; // default = 10
        static double D =0; // default = 0
        static double I =0; // default = 3
        static double F =8; // default = 0
    }

//    public MecanumDriveMona(HardwareMap hardwareMap, Pose2d pose, AprilTagProcessor aprilTagProcessor) {
//        super(hardwareMap, pose, aprilTagProcessor);
//    }

    public void init() {
        //set the PID values one time
        leftFront.setVelocityPIDFCoefficients(Params.P, Params.I, Params.D, Params.F);
        rightFront.setVelocityPIDFCoefficients(Params.P, Params.I, Params.D, Params.F);
        leftBack.setVelocityPIDFCoefficients(Params.P, Params.I, Params.D, Params.F);
        rightBack.setVelocityPIDFCoefficients(Params.P, Params.I, Params.D, Params.F);
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
            current_drive_ramp = Ramp(drive, current_drive_ramp, Params.DRIVE_RAMP);
            current_strafe_ramp = Ramp(strafe, current_strafe_ramp, Params.STRAFE_RAMP);
            current_turn_ramp = Ramp(turn, current_turn_ramp, Params.TURN_RAMP);

            double dPercent = abs(current_drive_ramp) / (abs(current_drive_ramp) + abs(current_strafe_ramp) + abs(current_turn_ramp));
            double sPercent = abs(current_strafe_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));
            double tPercent = abs(current_turn_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));

            leftFrontTargetSpeed = Params.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightFrontTargetSpeed = Params.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));
            leftBackTargetSpeed = Params.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightBackTargetSpeed = Params.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));

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
        if (Math.abs(currentValue) + Params.RAMP_THRESHOLD < Math.abs(target)) {
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


}


