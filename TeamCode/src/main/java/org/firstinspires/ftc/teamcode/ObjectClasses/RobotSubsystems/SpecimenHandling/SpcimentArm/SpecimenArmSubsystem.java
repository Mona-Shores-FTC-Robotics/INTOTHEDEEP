package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm;

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
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.TrapezoidalPIDController;

@Config
public class SpecimenArmSubsystem extends SubsystemBase {

    public static class SpecimenArmParams {
        public double SCALE_FACTOR = 1.0;
        public double DEAD_ZONE = 0.05;
        public double MAX_POWER = .7;
        public double P = 0.0085, I = 0.0, D = 0.0;
        public double kS = 0.11, kCos = 0.38, kV = 0.0, kA = 0.0; // Feedforward coefficients

        public double TICKS_PER_DEGREE = 0.8;
        public double STARTING_ANGLE_OFFSET_DEGREES = 245.0;
        public double HORIZONTAL_ANGLE = 180.0;

        public double MIN_ANGLE = 70.0;
        public double MAX_ANGLE = STARTING_ANGLE_OFFSET_DEGREES;

        public double TIMEOUT_TIME_SECONDS = 3;
        public double SPECIMEN_PICKUP_ANGLE = 190.0;
        public double SPECIMEN_DELIVERY_ANGLE = 80;
        public double SPECIMEN_STAGING_ANGLE = 115;
        public double THRESHOLD = 1.0;
    }

    public enum SpecimenArmStates {
        HOME, SPECIMEN_PICKUP, SPECIMEN_DELIVERY, MANUAL, SPECIMEN_STAGING, HORIZONTAL;
        public double angle;
        static {
            HOME.angle = SPECIMEN_ARM_PARAMS.STARTING_ANGLE_OFFSET_DEGREES;
            SPECIMEN_PICKUP.angle = SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE;
            SPECIMEN_STAGING.angle = SPECIMEN_ARM_PARAMS.SPECIMEN_STAGING_ANGLE;
            SPECIMEN_DELIVERY.angle = SPECIMEN_ARM_PARAMS.SPECIMEN_DELIVERY_ANGLE;
            HORIZONTAL.angle = SPECIMEN_ARM_PARAMS.HORIZONTAL_ANGLE;
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
    private SpecimenArmStates currentState;
    private SpecimenArmStates targetState;
    private double currentTicks;
    private double currentVelocity;
    private double currentAngleDegrees;
    private double targetAngleDegrees;
    private ArmFeedforward armFeedforward;
    private double feedforwardPower;
    private double clippedPower;
    private Encoder armEncoder;

    //    private TrapezoidalPIDController trapezoidalPIDController;
    private PIDController pidController;
    private double pidPower;
    private double totalPower;

    // Variables for telemetry

    private TrapezoidalPIDController.MovementDirection movementDirection;
    private double newManualTargetAngle;

    public SpecimenArmSubsystem(final HardwareMap hMap, final String name) {
        arm = hMap.get(DcMotorEx.class, name);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        currentState = SpecimenArmStates.HOME;
        armEncoder =  new OverflowEncoder(new RawEncoder(arm));
        armEncoder.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void init() {
        pidController = new PIDController(SPECIMEN_ARM_PARAMS.P, SPECIMEN_ARM_PARAMS.I, SPECIMEN_ARM_PARAMS.D);
        pidController.setTolerance(SPECIMEN_ARM_PARAMS.THRESHOLD);

        armFeedforward = new ArmFeedforward(
                SPECIMEN_ARM_PARAMS.kS,
                SPECIMEN_ARM_PARAMS.kCos,
                SPECIMEN_ARM_PARAMS.kV,
                SPECIMEN_ARM_PARAMS.kA
        );
        targetState = SpecimenArmStates.HOME;
        setManualTargetState(targetState.getArmAngle());
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        initToHorizontal(); // Initialize to a test state
    }

    public void periodic() {
        currentTicks = armEncoder.getPositionAndVelocity().position;
        currentVelocity = armEncoder.getPositionAndVelocity().velocity;

        // Step 1: Calculate feedforward power based on angle (for gravity compensation)
        currentAngleDegrees = getCurrentArmAngleInDegrees();
        double currentAngleRadians = Math.toRadians(currentAngleDegrees);
        feedforwardPower = armFeedforward.calculate(currentAngleRadians, 0, 0);

        // Step 2: Calculate pid Power
        pidPower = pidController.calculate(currentAngleDegrees);

        totalPower = feedforwardPower + pidPower;

        // Clip the total power to the allowable range
        clippedPower = Range.clip(totalPower, -SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);

        // Set the motor power to move the arm
        arm.setPower(clippedPower);

        updateArmState();
        updateParameters();
        updateDashboardTelemetry();
    }


    public void updateParameters() {
        updateArmAngles(SpecimenArmStates.HOME, SPECIMEN_ARM_PARAMS.STARTING_ANGLE_OFFSET_DEGREES);
        updateArmAngles(SpecimenArmStates.SPECIMEN_PICKUP, SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE);
        updateArmAngles(SpecimenArmStates.SPECIMEN_STAGING, SPECIMEN_ARM_PARAMS.SPECIMEN_STAGING_ANGLE);
        updateArmAngles(SpecimenArmStates.SPECIMEN_DELIVERY, SPECIMEN_ARM_PARAMS.SPECIMEN_DELIVERY_ANGLE);
        updatePIDCoefficients();
    }

    public void setTargetAngle(double inputAngle) {
        // Normalize and clip the input angle
        double normalizedAngle = normalizeAngle(inputAngle);
        targetAngleDegrees = Range.clip(normalizedAngle, SPECIMEN_ARM_PARAMS.MIN_ANGLE, SPECIMEN_ARM_PARAMS.MAX_ANGLE);

        // Set the movement direction and setpoint in the PID controller
        pidController.setSetPoint(targetAngleDegrees);
        pidController.reset();
    }

    public boolean isArmAtTarget() {
        return pidController.atSetPoint();
    }

    public void updateArmState() {
        if (isArmAtTarget()) {
            setCurrentState(targetState);
        }
    }

    public void setTargetState(SpecimenArmStates state) {
        targetState = state;
        setTargetAngle(state.getArmAngle());
    }

    public SpecimenArmStates getCurrentState() {
        return currentState;
    }

    public void setCurrentState(SpecimenArmStates state) {
        currentState = state;
    }

    public SpecimenArmStates getTargetState() {
        return targetState;
    }

    public double getCurrentTicks() {
        return currentTicks;
    }

    public void setManualTargetState(double armInput) {
        // Calculate the change in angle based on input and scale factor
        double deltaAngle = armInput * SPECIMEN_ARM_PARAMS.SCALE_FACTOR;

        // Calculate the new target angle based on the current angle
        newManualTargetAngle = targetAngleDegrees + deltaAngle;

        newManualTargetAngle = Range.clip(newManualTargetAngle, SPECIMEN_ARM_PARAMS.MIN_ANGLE, SPECIMEN_ARM_PARAMS.MAX_ANGLE);

        // Update the MANUAL state with the new angle
        SpecimenArmStates.MANUAL.setArmAngle(newManualTargetAngle);

        // Set the target state to MANUAL
        setTargetState(SpecimenArmStates.MANUAL);
    }

    private void updatePIDCoefficients() {
        // Get the latest PIDF values from parameters
        double p = SPECIMEN_ARM_PARAMS.P;
        double i = SPECIMEN_ARM_PARAMS.I;
        double d = SPECIMEN_ARM_PARAMS.D;

        // Only update the PIDF controller if there's a change
//        if (trapezoidalPIDController.getP() != p || trapezoidalPIDController.getI() != i ||
//                trapezoidalPIDController.getD() != d) {
//            // Set the new PIDF values to the controller
//            trapezoidalPIDController.setP(p);
//            trapezoidalPIDController.setI(i);
//            trapezoidalPIDController.setD(d);
//        }
        if (pidController.getP() != p || pidController.getI() != i ||
                pidController.getD() != d) {
            // Set the new PIDF values to the controller
            pidController.setP(p);
            pidController.setI(i);
            pidController.setD(d);
        }
    }

    private void updateArmAngles(SpecimenArmStates armState, double newArmAngle) {
        if (armState.getArmAngle() != newArmAngle) {
            armState.setArmAngle(newArmAngle);
        }
    }

    // Basic telemetry display in a single line with a descriptive label
    public void displayBasicTelemetry(Telemetry telemetry) {
        @SuppressLint("DefaultLocale")
        String telemetryData = String.format("State: %s | Angle: %.2f", currentState, currentAngleDegrees);

        if (currentState != targetState) {
            telemetryData += String.format(" | Target State: %s", targetState);
        }

        telemetry.addData("Specimen Arm Status", telemetryData);
    }

    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addData("specimenArm/Current State", currentState);
        telemetry.addData("specimenArm/Target State", targetState);
        telemetry.addData("specimenArm/Motor Power", arm.getPower());
    }

    @SuppressLint("DefaultLocale")
    public void updateDashboardTelemetry() {
        // Display an overview of the current and target states and positions
        String statusOverview = String.format("State: %s | Target: %s | Position: %.2f | Target Position: %.2f",
                currentState.toString(), targetState.toString(), currentAngleDegrees, pidController.getSetPoint());
        MatchConfig.telemetryPacket.put("specimenArm/Status Overview", statusOverview);

        // Detailed telemetry for real-time analysis
        MatchConfig.telemetryPacket.put("specimenArm/angles/Current Arm Angle", String.format("%.2f", currentAngleDegrees));
        MatchConfig.telemetryPacket.put("specimenArm/angles/Target Arm Angle", String.format("%.2f", targetAngleDegrees));
        MatchConfig.telemetryPacket.put("specimenArm/Current Arm Velocity", String.format("%.2f", currentVelocity));

        // Add combined power overview
        String powerSummary = String.format(
                "PID: %.2f | FF: %.2f | Clipped: %.2f",
                pidPower,
                feedforwardPower,
                clippedPower
        );
        MatchConfig.telemetryPacket.put("specimenArm/Power Overview", powerSummary);
    }

    private double getCurrentArmAngleInDegrees() {
        // Calculate the base angle from encoder ticks
        double angle = currentTicks / SPECIMEN_ARM_PARAMS.TICKS_PER_DEGREE;

        angle += SPECIMEN_ARM_PARAMS.STARTING_ANGLE_OFFSET_DEGREES;

        // Normalize the angle to 0–360
        angle = (angle % 360 + 360) % 360;

        return angle;
    }

    public void initToHorizontal() {
        setTargetState(SpecimenArmStates.HORIZONTAL);
    }

    public static double normalizeAngle(double angle) {
        return (angle % 360 + 360) % 360; // Normalize to stay within 0–360 degrees
    }

    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }
}
