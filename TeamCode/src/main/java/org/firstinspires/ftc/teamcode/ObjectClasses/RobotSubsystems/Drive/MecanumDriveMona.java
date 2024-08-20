package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.AprilTagDrive;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;

@Config
public final class MecanumDriveMona {

    public static class ParamsDriveTrainConstants {
        // DriveTrain physical constants
        public static double MAX_MOTOR_SPEED_RPS = 435.0 / 60.0;
        public static double TICKS_PER_REV = 384.5;
        public static double MAX_SPEED_TICK_PER_SEC = MAX_MOTOR_SPEED_RPS * TICKS_PER_REV;
    }

    public static DriveSubsystem.ParamsMona MotorParameters = new DriveSubsystem.ParamsMona();
    public static DriveSubsystem.ParamsRRMona MotorParametersRR = new DriveSubsystem.ParamsRRMona();
    public static ParamsDriveTrainConstants DriveTrainConstants = new ParamsDriveTrainConstants();

    public double drive, strafe, turn;
    public double last_drive=0, last_strafe=0, last_turn=0;
    public double current_drive_ramp = 0, current_strafe_ramp=0, current_turn_ramp=0;
    public double aprilTagDrive, aprilTagStrafe, aprilTagTurn;
    public double leftFrontTargetSpeed, rightFrontTargetSpeed, leftBackTargetSpeed, rightBackTargetSpeed;

    public MecanumKinematics kinematics;
    public MotorFeedforward feedforward;
    public TurnConstraints defaultTurnConstraints;
    public VelConstraint defaultVelConstraint;
    public AccelConstraint defaultAccelConstraint;

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public VoltageSensor voltageSensor;
    public Localizer localizer;
    public Pose2d pose;

    public final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public void init() {
        HardwareMap hardwareMap = Robot.getInstance().getActiveOpMode().hardwareMap;

        if (this.pose ==null) {
            this.pose = new Pose2d(0, 0, 0);
        }

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "LFDrive");
        leftBack = hardwareMap.get(DcMotorEx.class, "LBDrive");
        rightBack = hardwareMap.get(DcMotorEx.class, "RBDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "RFDrive");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //set the PID values one time
        leftFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        rightFront.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        leftBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);
        rightBack.setVelocityPIDFCoefficients(MotorParameters.P, MotorParameters.I, MotorParameters.D, MotorParameters.F);

        localizer = new DriveLocalizer(this, hardwareMap);

        //Initialize the Roadrunner parameters (kinematics, feedforward, etc.)
        SetRoadRunnerParameters();
    }

    public void SetRoadRunnerParameters() {
        kinematics = new MecanumKinematics(
                MotorParametersRR.inPerTick * MotorParametersRR.trackWidthTicks, MotorParametersRR.inPerTick / MotorParametersRR.lateralInPerTick);

        feedforward = new MotorFeedforward(MotorParametersRR.kS, MotorParametersRR.kV / MotorParametersRR.inPerTick, MotorParametersRR.kA / MotorParametersRR.inPerTick);

        defaultTurnConstraints = new TurnConstraints(
                MotorParametersRR.maxAngVel, -MotorParametersRR.maxAngAccel, MotorParametersRR.maxAngAccel);

        defaultVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        kinematics.new WheelVelConstraint(MotorParametersRR.maxWheelVel),
                        new AngularVelConstraint(MotorParametersRR.maxAngVel)
                ));

        defaultAccelConstraint = new ProfileAccelConstraint(MotorParametersRR.minProfileAccel, MotorParametersRR.maxProfileAccel);

        FlightRecorder.write("MECANUM_RR_PARAMS", MotorParametersRR);
        FlightRecorder.write("MECANUM_PID_PARAMS", MotorParameters);
    }


    public void mecanumDriveSpeedControl(double drive, double strafe, double turn) {

        //I believe this is the best spot to put this for tracking our pose
        //We had this method call in the periodic() of the Drivesystem, but that means it can be called twice a loop if we run any RR actions (e.g.follow a trajectory)
        //putting it here, should avoid that double call

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
            //safetydrivespeedfactor is set when we lookforapriltags based on the closest backdrop apriltag we see (for the oposite alliance color)
            if (Robot.getInstance().getVisionSubsystem().blueBackdropAprilTagFoundRecently &&
                    MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.RED &&
                    drive > .2) {
                drive = Math.min(drive, DriveSubsystem.driveParameters.safetyDriveSpeedFactor);
            }
            //If we see red tags and we are blue and we are driving toward them, then use the safetydrivespeedfactor to slow us down
            else if (Robot.getInstance().getVisionSubsystem().redBackdropAprilTagFoundRecently &&
                    MatchConfig.finalAllianceColor == InitVisionProcessor.AllianceColor.BLUE &&
                    drive > .2) {
                drive = Math.min(drive, DriveSubsystem.driveParameters.safetyDriveSpeedFactor);
            }

            current_drive_ramp = Ramp(drive, current_drive_ramp, MotorParameters.DRIVE_RAMP);
            current_strafe_ramp = Ramp(strafe, current_strafe_ramp, MotorParameters.STRAFE_RAMP);
            current_turn_ramp = Ramp(turn, current_turn_ramp, MotorParameters.TURN_RAMP);

            double dPercent = abs(current_drive_ramp) / (abs(current_drive_ramp) + abs(current_strafe_ramp) + abs(current_turn_ramp));
            double sPercent = abs(current_strafe_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));
            double tPercent = abs(current_turn_ramp) / (abs(current_drive_ramp) + abs(current_turn_ramp) + abs(current_strafe_ramp));

            leftFrontTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightFrontTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));
            leftBackTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (-current_strafe_ramp * sPercent) + (current_turn_ramp * tPercent));
            rightBackTargetSpeed = DriveTrainConstants.MAX_SPEED_TICK_PER_SEC * ((current_drive_ramp * dPercent) + (current_strafe_ramp * sPercent) + (-current_turn_ramp * tPercent));

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
        if (Math.abs(currentValue) + MotorParameters.RAMP_THRESHOLD < Math.abs(target)) {
            return Math.signum(target) * (Math.abs(currentValue) + ramp_amount);
        }  else
        {
            return target;
        }
    }



    // This is what updates the pose estimate of the robot using the localizer
    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
        return twist.velocity().value();
    }

    public void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
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



}


