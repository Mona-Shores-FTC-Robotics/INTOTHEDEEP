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

@Config
public class SpecimenArmSubsystem extends SubsystemBase {

    public static class SpecimenArmParams {
        public double SCALE_FACTOR = 1.0;
        public double DEAD_ZONE = 0.05;
        public double MAX_POWER = 0.7;
        public double P = 0.0, I = 0.0, D = 0.0; // PID coefficients
        public double kS = 0.1, kCos = .1, kV = 0.0, kA = 0.0; // Feedforward coefficients

        public double TICKS_PER_DEGREE = 0.8;
        public double STARTING_ANGLE_OFFSET_DEGREES = 244.0;
        public double HORIZONTAL_ANGLE = 180.0;

        public double MIN_ANGLE = 50.0;
        public double MAX_ANGLE = STARTING_ANGLE_OFFSET_DEGREES;

        public double TIMEOUT_TIME_SECONDS = 3;
        public double SPECIMEN_PICKUP_ANGLE = 190.0;
        public double SPECIMEN_DELIVERY_ANGLE = 80;
        public double SPECIMEN_STAGING_ANGLE = 115;
        public double THRESHOLD = 1.0;

        // Motion Profile Parameters
        public double MAX_ACCELERATION = 30.0; // degrees per second² (Adjust as needed)
        public double MAX_VELOCITY = 30.;     // degrees per second (Adjust as needed)

        private final double MANUAL_UPDATE_INTERVAL = 0.1; // seconds


    }

    public enum SpecimenArmStates {
        ARM_HOME, SPECIMEN_PICKUP, SPECIMEN_DELIVERY, ARM_MANUAL, SPECIMEN_STAGING, HORIZONTAL;
        private double angle;

        static {
            ARM_HOME.angle = SPECIMEN_ARM_PARAMS.STARTING_ANGLE_OFFSET_DEGREES;
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
    private final Encoder armEncoder;

    // PID Controller
    private PIDController pidController;
    private double pidPower;
    private double totalPower;

    // Trapezoidal Motion Profile
    private TrapezoidalMotionProfile motionProfile;
    private double profileStartPosition;
    private double profileDistance;

    // Timer to track elapsed time
    private final com.qualcomm.robotcore.util.ElapsedTime timer;

    // Control Mode
    private enum ControlMode {
        AUTOMATED,
        MANUAL
    }
    private ControlMode controlMode = ControlMode.AUTOMATED;

    // Variables for telemetry and rate limiting
    private double lastManualUpdateTime = 0;

    public SpecimenArmSubsystem(final HardwareMap hMap, final String name) {
        arm = hMap.get(DcMotorEx.class, name);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        currentState = SpecimenArmStates.ARM_HOME;
        armEncoder = new OverflowEncoder(new RawEncoder(arm));
        armEncoder.setDirection(DcMotorEx.Direction.FORWARD);

        // Initialize timer
        timer = new com.qualcomm.robotcore.util.ElapsedTime();
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
        targetState = SpecimenArmStates.ARM_HOME;
        setTargetState(targetState); // Use setTargetState instead of setManualTargetState
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        currentTicks = armEncoder.getPositionAndVelocity().position;
        currentVelocity = armEncoder.getPositionAndVelocity().velocity;

        // Convert ticks to degrees
        currentAngleDegrees = getCurrentArmAngleInDegrees();

        if (controlMode == ControlMode.AUTOMATED && motionProfile != null) {
            double elapsedTime = timer.seconds();
            double targetPositionDelta = motionProfile.getPosition(elapsedTime);
            double targetVelocity = motionProfile.getVelocity(elapsedTime);

            // Determine the direction of movement based on profileDistance
            Direction movementDirection = getMovementDirection(profileDistance);
            double directionMultiplier = 0.0;

            switch (movementDirection) {
                case COUNTER_CLOCKWISE:
                    directionMultiplier = 1.0;
                    break;
                case CLOCKWISE:
                    directionMultiplier = -1.0;
                    break;
                case NONE:
                    directionMultiplier = 0.0;
                    break;
            }

            // Desired absolute position
            double desiredPosition = profileStartPosition + (targetPositionDelta * directionMultiplier);

            // Ensure desiredPosition does not exceed limits
            desiredPosition = Range.clip(desiredPosition, SPECIMEN_ARM_PARAMS.MIN_ANGLE, SPECIMEN_ARM_PARAMS.MAX_ANGLE);

            // PID control for position
            double positionError = desiredPosition - currentAngleDegrees;
            pidPower = pidController.calculate(positionError);

            // Feedforward based on desired velocity and direction
            double feedforward = armFeedforward.calculate(
                    Math.toRadians(desiredPosition),
                    Math.toRadians(targetVelocity * directionMultiplier),
                    0
            );

            // Total power
            totalPower = pidPower + feedforward;

            // Clip the total power to the allowable range
            clippedPower = Range.clip(totalPower, -SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);

            // Set the motor power to move the arm
            arm.setPower(clippedPower);

            // Check if motion profile is finished
            if (motionProfile.isFinished(elapsedTime)) {
                motionProfile = null; // Motion complete
                setCurrentState(targetState);
                pidController.setSetPoint(targetAngleDegrees); // Ensure PID setpoint is correct
                // No need to set arm power to zero
            }
        } else if (controlMode == ControlMode.MANUAL) {
            handleManualControl();
        } else {
            maintainPosition();
        }

        updateArmState();
        updateParameters();
        updateDashboardTelemetry();
    }

    /**
     * Initiates movement to a target state using trapezoidal motion profiling.
     *
     * @param state The target state to move to.
     */
    public void setTargetState(SpecimenArmStates state) {
        targetState = state;
        profileStartPosition = currentAngleDegrees;
        double profileTargetPosition = state.getArmAngle();
        profileDistance = profileTargetPosition - profileStartPosition;

        // Update targetAngleDegrees with clipping
        targetAngleDegrees = Range.clip(profileTargetPosition, SPECIMEN_ARM_PARAMS.MIN_ANGLE, SPECIMEN_ARM_PARAMS.MAX_ANGLE);

        // Initialize motion profile with absolute distance
        double absoluteDistance = Math.abs(profileDistance);

        motionProfile = new TrapezoidalMotionProfile(
                SPECIMEN_ARM_PARAMS.MAX_ACCELERATION,
                SPECIMEN_ARM_PARAMS.MAX_VELOCITY,
                absoluteDistance
        );

        // Start the timer
        timer.reset();

        // Set control mode to automated
        controlMode = ControlMode.AUTOMATED;

        // Optionally, log or store the movement direction if needed
        // For example:
        // this.currentMovementDirection = movementDirection;
    }

    private enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE,
        NONE
    }

    private Direction getMovementDirection(double distance) {
        if (Math.abs(distance) < SPECIMEN_ARM_PARAMS.THRESHOLD) {
            return Direction.NONE;
        } else if (distance > 0) {
            return Direction.COUNTER_CLOCKWISE;
        } else {
            return Direction.CLOCKWISE;
        }
    }

    public SpecimenArmStates getCurrentState() {
        return currentState;
    }

    public void setCurrentState(SpecimenArmStates state) {
        currentState = state;
    }

    /**
     * Initiates manual movement by adjusting the target angle based on input.
     *
     * @param armInput The input value to adjust the arm's angle.
     */
    public void setManualTargetState(double armInput) {
        double currentTime = timer.seconds();
        if (currentTime - lastManualUpdateTime < SPECIMEN_ARM_PARAMS.MANUAL_UPDATE_INTERVAL) {
            // Ignore input if within the cooldown period
            return;
        }

        lastManualUpdateTime = currentTime;
        controlMode = ControlMode.MANUAL; // Switch to manual mode

        // Calculate the change in angle based on input and scale factor
        double deltaAngle = armInput * SPECIMEN_ARM_PARAMS.SCALE_FACTOR;

        // Calculate the new target angle based on the target angle
        double newManualTargetAngle = targetAngleDegrees + deltaAngle;

        double clippedManualTargetAngle = Range.clip(newManualTargetAngle, SPECIMEN_ARM_PARAMS.MIN_ANGLE, SPECIMEN_ARM_PARAMS.MAX_ANGLE);

        // Update the MANUAL state with the new angle
        SpecimenArmStates.ARM_MANUAL.setArmAngle(clippedManualTargetAngle);

        // Set the target state to MANUAL
        setTargetState(SpecimenArmStates.ARM_MANUAL);
    }

    /**
     * Checks if the arm has reached the target state.
     *
     * @return True if the arm is at the target state, false otherwise.
     */
    public boolean isArmAtTarget() {
        return currentState == targetState;
    }

    public void updateArmState() {
        if (isArmAtTarget()) {
            setCurrentState(targetState);
        }
    }

    public void updateParameters() {
        updateArmAngles(SpecimenArmStates.ARM_HOME, SPECIMEN_ARM_PARAMS.STARTING_ANGLE_OFFSET_DEGREES);
        updateArmAngles(SpecimenArmStates.SPECIMEN_PICKUP, SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE);
        updateArmAngles(SpecimenArmStates.SPECIMEN_STAGING, SPECIMEN_ARM_PARAMS.SPECIMEN_STAGING_ANGLE);
        updateArmAngles(SpecimenArmStates.SPECIMEN_DELIVERY, SPECIMEN_ARM_PARAMS.SPECIMEN_DELIVERY_ANGLE);
        updatePIDCoefficients();
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

    private void updateArmAngles(SpecimenArmStates armState, double newArmAngle) {
        if (armState.getArmAngle() != newArmAngle) {
            armState.setArmAngle(newArmAngle);
        }
    }

    // Basic telemetry display in a single line with a descriptive label
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
    }

    @SuppressLint("DefaultLocale")
    public void updateDashboardTelemetry() {
        // Display an overview of the current and target states and positions
        String statusOverview = String.format("State: %s | Target: %s | Position: %.2f | Target Position: %.2f",
                currentState.toString(), targetState.toString(), currentAngleDegrees, targetAngleDegrees);
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

    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }

    public double getCurrentTicks() {
        return currentTicks;
    }


    public double getCurrentAngleDegrees() {
        return currentAngleDegrees;
    }

    /**
     * Handles manual control by maintaining the target angle through PID.
     */
    private void handleManualControl() {
        // Calculate PID power based on the current target angle
        double positionError = targetAngleDegrees - currentAngleDegrees;
        pidPower = pidController.calculate(positionError);

        // Feedforward based on the current angle (for gravity compensation)
        double currentAngleRadians = Math.toRadians(currentAngleDegrees);
        double feedforward = armFeedforward.calculate(currentAngleRadians, 0, 0);

        // Total power
        totalPower = pidPower + feedforward;

        // Clip the total power to the allowable range
        clippedPower = Range.clip(totalPower, -SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);

        // Set the motor power to move the arm
        arm.setPower(clippedPower);
    }

    /**
     * Maintains the current position using PID and feedforward.
     */
    private void maintainPosition() {
        // Calculate feedforward power based on angle (for gravity compensation)
        double currentAngleRadians = Math.toRadians(currentAngleDegrees);
        feedforwardPower = armFeedforward.calculate(currentAngleRadians, 0, 0);

        // Calculate PID power to maintain position
        pidPower = pidController.calculate(targetAngleDegrees - currentAngleDegrees);

        totalPower = feedforwardPower + pidPower;

        // Clip the total power to the allowable range
        clippedPower = Range.clip(totalPower, -SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);

        // Set the motor power to move the arm
        arm.setPower(clippedPower);
    }
}
