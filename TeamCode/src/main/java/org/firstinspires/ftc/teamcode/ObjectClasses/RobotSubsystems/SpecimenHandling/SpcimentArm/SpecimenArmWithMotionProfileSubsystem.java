package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@Config
public class SpecimenArmWithMotionProfileSubsystem extends SubsystemBase {

    public static class SpecimenArmParams {
        public long DELAY_UNTIL_POWER_ZERO_MILLISECONDS = 400;
        //Gamepad parameters
        public double GAMEPAD_STICK_SCALE_FACTOR = 1.0;
        public double DEAD_ZONE = 0.05;

        //PID parameters
        public double P = 0.0085, I = 0.0, D = 0.0; // PID coefficients
        public double ANGLE_TOLERANCE_THRESHOLD_DEGREES = 1.0;

        //Arm Feedforward parameters
        public double kS = 0.075, kCos = 0.222, kV = .08, kA = .05; // Feedforward coefficients

        public double INTERNAL_P;
        public double INTERNAL_F;
        //Maximum Power for clipping motor output
        public double MAX_POWER = 0.7;

        // Mechanical Parameters}
        public double GEAR_RATIO = 2.8;
        public double MOTOR_TICKS_PER_DEGREE = 537.7 / 360.0;
        public double TICKS_PER_DEGREE = MOTOR_TICKS_PER_DEGREE * GEAR_RATIO; // 4.181

        //Preset Angles
        public double CCW_HOME = 244.0;
        public double SPECIMEN_PICKUP_ANGLE = 190.0;
        public double SPECIMEN_DELIVERY_ANGLE = 105;
        public double SLOP_SWITCH_ANGLE = 110.0;
        public double CW_HOME = 35.23;

        //Fudge factor to try and fix positions on CW side of chain slop
        public double CHAIN_SLOP_OFFSET_DEGREES = -5;

        // Motion Profile Parameters
        public double MAX_PROFILE_ACCELERATION = 135; // degrees per second² (Adjust as needed)
        public double MAX_PROFILE_VELOCITY = 90;     // degrees per second (Adjust as needed)
        public double TIMEOUT_TIME_SECONDS = 5;

        public double CONSTANT_VELOCITY=0;
        public double CONSTANT_POWER=0;
    }
    public enum SpecimenArmStates {
        CCW_ARM_HOME, CW_ARM_HOME, SPECIMEN_PICKUP, SPECIMEN_DELIVERY, ARM_MANUAL, CONSTANT_VELOCITY, CONSTANT_POWER, OFF;
        private double angle;

        static {
            CCW_ARM_HOME.angle = SPECIMEN_ARM_PARAMS.CCW_HOME;
            SPECIMEN_PICKUP.angle = SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE;
            SPECIMEN_DELIVERY.angle = SPECIMEN_ARM_PARAMS.SPECIMEN_DELIVERY_ANGLE;
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

    //Angle Variables
    private double currentAngleDegrees;
    private double fudgedCurrentAngleDegrees=Double.NaN;
    private double targetAngleDegrees;

    //Feedforward
    private ArmFeedforward armFeedforward;
    private double feedforwardPower;

    // Detect our battery voltage to scale our feedforward parameters
    private static final double NOMINAL_VOLTAGE = 12.0;
    VoltageSensor voltageSensor;

    // PID Controller
    private PIDController pidController;
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
    private final ElapsedTime timer;

    public SpecimenArmWithMotionProfileSubsystem(final HardwareMap hMap, final String name) {
        arm = hMap.get(DcMotorEx.class, name);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        currentState = SpecimenArmStates.CCW_ARM_HOME;
        armEncoder = new OverflowEncoder(new RawEncoder(arm));
        armEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        timer = new ElapsedTime();

    }

    public void init() {
        arm.setVelocityPIDFCoefficients(SPECIMEN_ARM_PARAMS.INTERNAL_P,0,0,SPECIMEN_ARM_PARAMS.INTERNAL_F);

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

        //TODO Where should the arm start? Might have to rethink how things are set up depending on if the arm is starting at CW_HOME or CCW_HOME
        currentState = SpecimenArmStates.CCW_ARM_HOME;
        currentAngleDegrees= SPECIMEN_ARM_PARAMS.CCW_HOME;
        targetAngleDegrees = SPECIMEN_ARM_PARAMS.CCW_HOME;
        pidController.setSetPoint(targetAngleDegrees);
    }

    @Override
    public void periodic() {
        // Retrieve current position from the encoder
        currentTicks = armEncoder.getPositionAndVelocity().position;
        currentAngleDegrees = calculateCurrentArmAngleInDegrees();

         if (currentState==SpecimenArmStates.ARM_MANUAL) {
             handleManualControl();
         } else if (currentState==SpecimenArmStates.CONSTANT_POWER || currentState==SpecimenArmStates.CONSTANT_VELOCITY ||
                currentState==SpecimenArmStates.OFF)
         {
             //DO NOTHING
         } else if (motionProfile != null) {
             handleMotionProfile();
        } else {
            maintainPosition();
        }
        updateParameters();
        updateDashboardTelemetry();
    }

    public void setManualTargetState(double armInput) {
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
        if (currentAngleDegrees<= SPECIMEN_ARM_PARAMS.SLOP_SWITCH_ANGLE) {
            fudgedCurrentAngleDegrees= currentAngleDegrees+ SPECIMEN_ARM_PARAMS.CHAIN_SLOP_OFFSET_DEGREES;
            pidPower = pidController.calculate(fudgedCurrentAngleDegrees);
            feedforwardPower = armFeedforward.calculate(
                    Math.toRadians(fudgedCurrentAngleDegrees),
                    0,
                    0
            );
        } else{
            fudgedCurrentAngleDegrees=Double.NaN;
            pidPower = pidController.calculate(currentAngleDegrees); // Pass current position, not error
            feedforwardPower = armFeedforward.calculate(
                    Math.toRadians(currentAngleDegrees),
                    0,
                    0
            );
        }

        // Total power
        totalPower = pidPower + feedforwardPower;

        // Clip the total power to the allowable range
        clippedPower = Range.clip(totalPower, -SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);

        // Set the motor power to move the arm
        arm.setPower(clippedPower);
    }
    public void setTargetStateWithMotionProfile(SpecimenArmStates state) {
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
        timer.reset();
    }
    private void handleMotionProfile() {
        double elapsedTimeMilliseconds = timer.milliseconds();

        // Get target angle, velocity, and acceleration from the motion profile for the current time
        double motionProfileDeltaAngleDegrees = motionProfile.getMotionProfileAngleInDegrees(elapsedTimeMilliseconds);
        desiredAngleDegrees = profileStartPosition + (motionProfileDeltaAngleDegrees * Math.signum(motionProfileTotalAngleChange));
        desiredAngleDegrees = Range.clip(desiredAngleDegrees, SPECIMEN_ARM_PARAMS.CW_HOME, SPECIMEN_ARM_PARAMS.CCW_HOME);
        targetVelocity = motionProfile.getVelocity(elapsedTimeMilliseconds) * Math.signum(motionProfileTotalAngleChange);
        targetAcceleration  = motionProfile.getAcceleration(elapsedTimeMilliseconds) * Math.signum(motionProfileTotalAngleChange);

        //If we are CW of 90 degrees, we need to add an offset to handle the chain slop
        if (currentAngleDegrees<= SPECIMEN_ARM_PARAMS.SLOP_SWITCH_ANGLE) {
            fudgedCurrentAngleDegrees=currentAngleDegrees+ SPECIMEN_ARM_PARAMS.CHAIN_SLOP_OFFSET_DEGREES;
            // PID control for position
            pidPower = pidController.calculate(fudgedCurrentAngleDegrees, desiredAngleDegrees);
            feedforwardPower = armFeedforward.calculate(
                    Math.toRadians(fudgedCurrentAngleDegrees),
                    Math.toRadians(targetVelocity),
                    Math.toRadians(targetAcceleration)
            );
        } else{
            fudgedCurrentAngleDegrees=Double.NaN;
            // PID control for position
            pidPower = pidController.calculate(currentAngleDegrees, desiredAngleDegrees);
            // Feedforward for arm to counteract gravity and friction; also account for velocity/acceleration setpoints
            feedforwardPower = armFeedforward.calculate(
                    Math.toRadians(currentAngleDegrees),
                    Math.toRadians(targetVelocity),
                    Math.toRadians(targetAcceleration)
            );
        }

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
        //depending on which side of the slop we are on we need an offset
        if (currentAngleDegrees<= SPECIMEN_ARM_PARAMS.SLOP_SWITCH_ANGLE) {
            fudgedCurrentAngleDegrees=currentAngleDegrees+ SPECIMEN_ARM_PARAMS.CHAIN_SLOP_OFFSET_DEGREES;
            pidPower = pidController.calculate(fudgedCurrentAngleDegrees); // Pass current position, not error
            feedforwardPower = armFeedforward.calculate(
                    Math.toRadians(fudgedCurrentAngleDegrees),
                    0,
                    0
            );
        } else{
            fudgedCurrentAngleDegrees=Double.NaN;
            // PID control for position
            pidPower = pidController.calculate(currentAngleDegrees); // Pass current position, not error
            feedforwardPower = armFeedforward.calculate(
                    Math.toRadians(currentAngleDegrees),
                    0,
                    0
            );
        }
        totalPower = pidPower + feedforwardPower;
        clippedPower = Range.clip(totalPower, -SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);
        arm.setPower(clippedPower);
    }
    public SpecimenArmStates getCurrentState() {
        return currentState;
    }
    public void setCurrentState(SpecimenArmStates state) {
        currentState = state;
    }
    public boolean isArmAtTarget() {
        return currentState == targetState;
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
        updateArmAngles(SpecimenArmStates.SPECIMEN_DELIVERY, SPECIMEN_ARM_PARAMS.SPECIMEN_DELIVERY_ANGLE);
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

        double internalP = SPECIMEN_ARM_PARAMS.INTERNAL_P;
        double internalF = SPECIMEN_ARM_PARAMS.INTERNAL_F;

        // Only update the PID controller if there's a change
        if (pidController.getP() != p || pidController.getI() != i || pidController.getD() != d) {
            pidController.setP(p);
            pidController.setI(i);
            pidController.setD(d);
        }

        if (arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p != internalP
                || arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p != internalF)  {
            arm.setVelocityPIDFCoefficients(internalP, 0,0, internalF);
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

        // Display current and target positions on a separate line
        String positionOverview;
        if (Double.isNaN(fudgedCurrentAngleDegrees)){
            positionOverview = String.format("Position: %.2f | Target Position: %.2f", currentAngleDegrees, targetAngleDegrees);
        } else{
            positionOverview = String.format("Fudged Position: %.2f | Target Position: %.2f", fudgedCurrentAngleDegrees, targetAngleDegrees);
        }

        MatchConfig.telemetryPacket.addLine(positionOverview);

        // Add power overview on its own line
        String powerSummary = String.format("PID: %.2f | FF: %.2f | Clipped: %.2f", pidPower, feedforwardPower, clippedPower);
        MatchConfig.telemetryPacket.addLine(powerSummary);

        MatchConfig.telemetryPacket.put("specimenArm/constant/Power", String.format("%.2f", arm.getPower()));
        MatchConfig.telemetryPacket.put("specimenArm/constant/Velocity", String.format("%.2f", arm.getVelocity(AngleUnit.DEGREES)));

        // Detailed telemetry for real-time analysis
        MatchConfig.telemetryPacket.put("specimenArm/velocity/Target Arm Velocity", String.format("%.2f", targetVelocity));
        MatchConfig.telemetryPacket.put("specimenArm/velocity/Target Arm Acceleration", String.format("%.2f", targetAcceleration));
        MatchConfig.telemetryPacket.put("specimenArm/velocity/Total Time for Motion Profile (s)", String.format("%.2f", motionProfileTotalTime/1000));

        MatchConfig.telemetryPacket.put("specimenArm/angles/Current Arm Angle", String.format("%.2f", currentAngleDegrees));
        MatchConfig.telemetryPacket.put("specimenArm/angles/Target Arm Angle", String.format("%.2f", targetAngleDegrees));
        MatchConfig.telemetryPacket.put("specimenArm/angles/Motion Profile Desired Arm Angle", String.format("%.2f", desiredAngleDegrees));
    }


}
