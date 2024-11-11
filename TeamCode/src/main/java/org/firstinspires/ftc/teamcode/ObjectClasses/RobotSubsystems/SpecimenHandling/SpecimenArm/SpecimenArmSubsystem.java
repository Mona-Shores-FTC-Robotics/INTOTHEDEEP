package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@Config
public class SpecimenArmSubsystem extends SubsystemBase {

    public static class SpecimenArmParams {

        //Flip parameters
        public double CCW_FLIP_TIME_MS = 300;
        public double CONSTANT_POWER_FOR_CCW_FLIP = .95;
        public double CW_FLIP_TIME_MS = 300;
        public double CONSTANT_POWER_FOR_CW_FLIP = -0.7;
        public double ZERO_POWER_TIME = 800;

        //Gamepad parameters
        public double GAMEPAD_STICK_SCALE_FACTOR = 1.0;
        public double DEAD_ZONE = 0.05;

        //PID parameters
        public double P = .015, I = 0.0, D = 0.0; // PID coefficients
        public double ANGLE_TOLERANCE_THRESHOLD_DEGREES = 1.0;

        //Arm Feedforward parameters
        public double kS = 0.075, kCos = .17, kV = .08, kA = .05; // Feedforward coefficients

        //Maximum Power for clipping motor output
        public double MAX_POWER = 0.7;

        // Mechanical Parameters}
        public double GEAR_RATIO = 1.0;
        public double MOTOR_TICKS_PER_DEGREE = 1993.6 / 360.0;
        public double TICKS_PER_DEGREE = MOTOR_TICKS_PER_DEGREE * GEAR_RATIO; // 4.181

        //Preset Angles
        public double CCW_HOME = 270;
        public double SPECIMEN_PICKUP_ANGLE = 216;
        public double CW_HOME = 35.23;

        // Motion Profile Parameters
        public double MAX_PROFILE_ACCELERATION = 135; // degrees per second² (Adjust as needed)
        public double MAX_PROFILE_VELOCITY = 90;     // degrees per second (Adjust as needed)
        public double TIMEOUT_TIME_SECONDS = 5;
    }
    public enum SpecimenArmStates {
        MOTION_PROFILE,
        CCW_ARM_HOME,
        CW_ARM_HOME,
        SPECIMEN_PICKUP,
        ARM_MANUAL,
        FLIPPING_TO_CCW,
        FLIPPING_TO_CW,
        ZERO_POWER;
        private double angle;

        static {
            CCW_ARM_HOME.angle = SPECIMEN_ARM_PARAMS.CCW_HOME;
            SPECIMEN_PICKUP.angle = SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE;
            CW_ARM_HOME.angle = SPECIMEN_ARM_PARAMS.CW_HOME;
        }

        public void setArmAngle(double t) {
            this.angle = t;
        }

        public double getArmAngle() {
            return this.angle;
        }
    }

    public static SpecimenArmParams SPECIMEN_ARM_PARAMS = new SpecimenArmParams();
    public DcMotorEx arm;

    // Arm Encoder Variables
    private final Encoder armEncoder;
    private double currentTicks;

    //State Variables
    private SpecimenArmStates currentState;
    private SpecimenArmStates targetState;
    public SpecimenArmStates lastState;

    //Angle Variables
    private double currentAngleDegrees;
    private double targetAngleDegrees;

    //Feedforward
    private ArmFeedforward armFeedforward;
    private double feedforwardPower;

    // Detect our battery voltage to scale our feedforward parameters
    private static final double NOMINAL_VOLTAGE = 12.0;
    VoltageSensor voltageSensor;

    // PID Controller
    public PIDController pidController;
    private double pidPower;
    private double totalPower;

    //Store the clipped power as a variable for telemetry
    private double clippedPower;

    // Trapezoidal Motion Profile
    private TrapezoidalMotionProfile motionProfile;
    private double profileStartPosition;
    private double motionProfileTotalAngleChange;
    private double desiredAngleDegrees;
    private double targetVelocity;
    private double targetAcceleration;
    private double motionProfileTotalTime;

    // Timer to track elapsed time for motion profile
    private final ElapsedTime motionProfileTimer;
    private final ElapsedTime flipArmTimer;

    public SpecimenArmSubsystem(final HardwareMap hMap, final String name) {
        arm = hMap.get(DcMotorEx.class, name);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.REVERSE);
        currentState = SpecimenArmStates.CCW_ARM_HOME;
        armEncoder = new OverflowEncoder(new RawEncoder(arm));
        armEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        motionProfileTimer = new ElapsedTime();
        flipArmTimer = new ElapsedTime();
    }

    public void init() {
        voltageSensor = Robot.getInstance().getActiveOpMode().hardwareMap.voltageSensor.iterator().next();
        double batteryVoltage = voltageSensor.getVoltage();
        double voltageScale = NOMINAL_VOLTAGE / batteryVoltage;

        armFeedforward = new ArmFeedforward(
                SPECIMEN_ARM_PARAMS.kS * voltageScale,
                SPECIMEN_ARM_PARAMS.kCos,
                SPECIMEN_ARM_PARAMS.kV * voltageScale,
                SPECIMEN_ARM_PARAMS.kA * voltageScale
        );

        pidController = new PIDController(SPECIMEN_ARM_PARAMS.P, SPECIMEN_ARM_PARAMS.I, SPECIMEN_ARM_PARAMS.D);
        pidController.setTolerance(SPECIMEN_ARM_PARAMS.ANGLE_TOLERANCE_THRESHOLD_DEGREES);
        pidController.reset();
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        currentState = lastState = SpecimenArmStates.CCW_ARM_HOME;
        currentAngleDegrees= SPECIMEN_ARM_PARAMS.CCW_HOME;
        targetAngleDegrees = SPECIMEN_ARM_PARAMS.CCW_HOME;
        pidController.setSetPoint(targetAngleDegrees);
    }

    @Override
    public void periodic() {
        // Retrieve current position from the encoder
        currentTicks = armEncoder.getPositionAndVelocity().position;
        currentAngleDegrees = calculateCurrentArmAngleInDegrees();
        switch(currentState)
        {
            case MOTION_PROFILE:
                handleMotionProfile();
                break;
            case ARM_MANUAL:
                handleManualControl();
                break;
            case FLIPPING_TO_CW:
                if (flipArmTimer.milliseconds() > SPECIMEN_ARM_PARAMS.CW_FLIP_TIME_MS) {
                    arm.setPower(0);
                    setCurrentState(SpecimenArmStates.CW_ARM_HOME);
                }
                break;
            case FLIPPING_TO_CCW:
                if (flipArmTimer.milliseconds() > SPECIMEN_ARM_PARAMS.CCW_FLIP_TIME_MS) {
                    arm.setPower(0);
                    setCurrentState(SpecimenArmStates.ZERO_POWER);
                    flipArmTimer.reset();
                }
                break;
            case ZERO_POWER:
                if (flipArmTimer.milliseconds() > SPECIMEN_ARM_PARAMS.ZERO_POWER_TIME) {
                    setCurrentState(SpecimenArmStates.CCW_ARM_HOME);
                }
                break;
            case CCW_ARM_HOME:
            case CW_ARM_HOME:
            case SPECIMEN_PICKUP:
            default:
                maintainPosition();
                break;
        }
         updateParameters();
         updateDashboardTelemetry();
    }

    public void flipCCWFastAction() {
        flipArmTimer.reset();
        currentState=SpecimenArmStates.FLIPPING_TO_CCW;
        targetState=SpecimenArmStates.CCW_ARM_HOME;
        arm.setPower(SpecimenArmSubsystem.SPECIMEN_ARM_PARAMS.CONSTANT_POWER_FOR_CCW_FLIP);
    }

    public void flipCWFastAction() {
        flipArmTimer.reset();
        currentState=SpecimenArmStates.FLIPPING_TO_CW;
        targetState=SpecimenArmStates.CW_ARM_HOME;
        arm.setPower(SpecimenArmSubsystem.SPECIMEN_ARM_PARAMS.CONSTANT_POWER_FOR_CW_FLIP);
    }

    public void setManualTargetState(double armInput) {
        lastState = currentState;
        targetState = SpecimenArmStates.ARM_MANUAL;
        currentState = SpecimenArmStates.ARM_MANUAL;

        // Calculate the change in angle based on input and scale factor
        double deltaAngle = armInput * SPECIMEN_ARM_PARAMS.GAMEPAD_STICK_SCALE_FACTOR;

        // Calculate the new target angle based on the target angle
        targetAngleDegrees += deltaAngle;

        // Clip target angle to within allowed range
        targetAngleDegrees = Range.clip(targetAngleDegrees, SPECIMEN_ARM_PARAMS.CW_HOME, SPECIMEN_ARM_PARAMS.CCW_HOME);

        // Update the PID controller's setpoint with the new target angle
        pidController.setSetPoint(targetAngleDegrees);
    }
    private void handleManualControl() {
        //depending on which side of the slop we are on we need an offset
            pidPower = pidController.calculate(currentAngleDegrees); // Pass current position, not error
            feedforwardPower = armFeedforward.calculate(
                    Math.toRadians(currentAngleDegrees),
                    0,
                    0
            );

        // Total power
        totalPower = pidPower + feedforwardPower;

        // Clip the total power to the allowable range
        clippedPower = Range.clip(totalPower, -SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);

        // Set the motor power to move the arm
        arm.setPower(clippedPower);
    }
    public void setTargetStateWithMotionProfile(SpecimenArmStates state) {
        currentState=SpecimenArmStates.MOTION_PROFILE;
        targetState = state;
        profileStartPosition=currentAngleDegrees;
        targetAngleDegrees = state.getArmAngle();
        targetAngleDegrees = Range.clip(targetAngleDegrees, SPECIMEN_ARM_PARAMS.CW_HOME, SPECIMEN_ARM_PARAMS.CCW_HOME);
        motionProfileTotalAngleChange = targetAngleDegrees - profileStartPosition;

        //Generate the motion profile
        motionProfile = new TrapezoidalMotionProfile(
                SPECIMEN_ARM_PARAMS.MAX_PROFILE_ACCELERATION,  //Provide in degrees per second^2
                SPECIMEN_ARM_PARAMS.MAX_PROFILE_VELOCITY, //Provide in degrees per second
                Math.abs(motionProfileTotalAngleChange) //Provide in degrees
        );

        motionProfileTotalTime = motionProfile.getTotalTimeMilliseconds();

        //Start the timer
        motionProfileTimer.reset();
    }
    private void handleMotionProfile() {
        double elapsedTimeMilliseconds = motionProfileTimer.milliseconds();

        // Get target angle, velocity, and acceleration from the motion profile for the current time
        double motionProfileDeltaAngleDegrees = motionProfile.getMotionProfileAngleInDegrees(elapsedTimeMilliseconds);
        desiredAngleDegrees = profileStartPosition + (motionProfileDeltaAngleDegrees * Math.signum(motionProfileTotalAngleChange));
        desiredAngleDegrees = Range.clip(desiredAngleDegrees, SPECIMEN_ARM_PARAMS.CW_HOME, SPECIMEN_ARM_PARAMS.CCW_HOME);
        targetVelocity = motionProfile.getVelocity (elapsedTimeMilliseconds) * Math.signum (motionProfileTotalAngleChange);
        targetAcceleration = motionProfile.getAcceleration (elapsedTimeMilliseconds) * Math.signum (motionProfileTotalAngleChange);
        // PID control for position
        pidPower = pidController.calculate (currentAngleDegrees, desiredAngleDegrees);
        // Feedforward for arm to counteract gravity and friction; also account for velocity/acceleration setpoints
        feedforwardPower = armFeedforward.calculate (
                Math.toRadians (currentAngleDegrees),
                Math.toRadians (targetVelocity),
                Math.toRadians (targetAcceleration)
        );

        totalPower = pidPower + feedforwardPower;

        // Clip the total power to the allowable range
        clippedPower = Range.clip(totalPower, -SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);

        // Set the motor power to move the arm
        arm.setPower(clippedPower);

        // Check if motion profile is finished
        if (motionProfile.isFinished(elapsedTimeMilliseconds)) {
            motionProfile = null; // Motion complete
            setCurrentState(targetState);
        }
    }
    private void maintainPosition() {
        // PID control for position
        pidPower = pidController.calculate (currentAngleDegrees); // Pass current position, not error
        feedforwardPower = armFeedforward.calculate (
                Math.toRadians (currentAngleDegrees),
                0,
                0
        );
        totalPower = pidPower + feedforwardPower;
        clippedPower = Range.clip (totalPower, - SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);
        arm.setPower (clippedPower);
    }

    public SpecimenArmStates getCurrentState() {
        return currentState;
    }
    public void setCurrentState(SpecimenArmStates state) {
        pidController.setSetPoint(state.getArmAngle());
        currentState = state;
    }
    private double calculateCurrentArmAngleInDegrees() {
        // Calculate the base angle from encoder ticks
        double angle = currentTicks / SPECIMEN_ARM_PARAMS.TICKS_PER_DEGREE;
        // Add the starting offset - zero encoder ticks should correspond to CCW Home position
        angle += SPECIMEN_ARM_PARAMS.CCW_HOME;

        // Normalize the angle to 0–360
        angle = (angle % 360 + 360) % 360;

        return angle;
    }
    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }
    public double getCurrentAngleDegrees() {
        return currentAngleDegrees;
    }
    public void updateParameters() {
        updateArmAngles(SpecimenArmStates.CCW_ARM_HOME, SPECIMEN_ARM_PARAMS.CCW_HOME);
        updateArmAngles(SpecimenArmStates.CW_ARM_HOME, SPECIMEN_ARM_PARAMS.CW_HOME);
        updateArmAngles(SpecimenArmStates.SPECIMEN_PICKUP, SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE);
        updatePIDCoefficients();
    }
    private void updateArmAngles(SpecimenArmStates armState, double newArmAngle) {
        if (armState.getArmAngle() != newArmAngle) {
            armState.setArmAngle(newArmAngle);
        }
    }
    private void updatePIDCoefficients() {
        // Get the latest PID values from parameters
        double p = SPECIMEN_ARM_PARAMS.P;
        double i = SPECIMEN_ARM_PARAMS.I;
        double d = SPECIMEN_ARM_PARAMS.D;

        // Only update the PID controller if there's a change
        if (pidController.getP() != p || pidController.getI() != i || pidController.getD() != d) {
            pidController.setP(p);
            pidController.setI(i);
            pidController.setD(d);
        }

    }

    public void displayBasicTelemetry(Telemetry telemetry) {
        @SuppressLint("DefaultLocale")
        String telemetryData = String.format("%s | Angle: %.2f", currentState, currentAngleDegrees);

        if (currentState != targetState) {
            telemetryData += String.format(" | Target State: %s", targetState);
        }

        telemetry.addLine(telemetryData);
    }
    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addData("specimenArm/Current State", currentState);
        telemetry.addData("specimenArm/Target State", targetState);
        telemetry.addData("specimenArm/Motor Power", arm.getPower());
        telemetry.addData("specimenArm/Motor Velocity", arm.getVelocity());
    }
    @SuppressLint("DefaultLocale")
    public void updateDashboardTelemetry() {
        // Display state and target state information on one line
        String stateOverview = String.format("State: %s | Target State: %s", currentState, targetState);
        MatchConfig.telemetryPacket.addLine(stateOverview);

        MatchConfig.telemetryPacket.put("specimenArm/state/current", String.format("%s", currentState));
        MatchConfig.telemetryPacket.put("specimenArm/state/target", String.format("%s", targetState));

        // Display current and target positions on a separate line
        String positionOverview;
        positionOverview = String.format("Position: %.2f | Target Position: %.2f", currentAngleDegrees, targetAngleDegrees);
        MatchConfig.telemetryPacket.addLine(positionOverview);
        // Add power overview on its own line
        String powerSummary = String.format("PID: %.2f | FF: %.2f | Clipped: %.2f", pidPower, feedforwardPower, clippedPower);
        MatchConfig.telemetryPacket.addLine(powerSummary);

        // Detailed telemetry for real-time analysis
        MatchConfig.telemetryPacket.put("specimenArm/velocity/Target Arm Velocity", String.format("%.2f", targetVelocity));
        MatchConfig.telemetryPacket.put("specimenArm/velocity/Target Arm Acceleration", String.format("%.2f", targetAcceleration));
        MatchConfig.telemetryPacket.put("specimenArm/velocity/Total Time for Motion Profile (s)", String.format("%.2f", motionProfileTotalTime/1000));

        MatchConfig.telemetryPacket.put("specimenArm/angles/Current Arm Angle", String.format("%.2f", currentAngleDegrees));
        MatchConfig.telemetryPacket.put("specimenArm/angles/Target Arm Angle", String.format("%.2f", targetAngleDegrees));
        MatchConfig.telemetryPacket.put("specimenArm/angles/Motion Profile Desired Arm Angle", String.format("%.2f", desiredAngleDegrees));
    }
}
