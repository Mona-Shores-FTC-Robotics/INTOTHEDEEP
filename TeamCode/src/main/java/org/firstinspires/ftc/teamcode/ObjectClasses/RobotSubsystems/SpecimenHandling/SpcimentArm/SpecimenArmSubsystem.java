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
        public double CHAIN_SLOP_OFFSET_DEGREES = -17;
        //gamepad parameters
        public double SCALE_FACTOR = 1.0;
        public double DEAD_ZONE = 0.05;

        public double P = 0.0085, I = 0.0, D = 0.0; // PID coefficients

        //Arm Feedforward parameters
        public double kS = 0.075, kCos = 0.222, kV = .08, kA = .05; // Feedforward coefficients

        public double MAX_POWER = 0.7;

        // Mechanical Constraints
        public double GEAR_RATIO = 2.8;
        public double MOTOR_TICKS_PER_DEGREE = 537.7 / 360.0;
        public double TICKS_PER_DEGREE = MOTOR_TICKS_PER_DEGREE * GEAR_RATIO; // 4.181

        //Angles
        public double CCW_HOME = 244.0;
        public double SPECIMEN_PICKUP_ANGLE = 190.0;
        public double SPECIMEN_DELIVERY_ANGLE = 105;
        public double SLOP_SWITCH_ANGLE = 110.0;
        public double CW_HOME = 35.23;

        public double THRESHOLD = 1.0;

        // Motion Profile Parameters
        public double MAX_PROFILE_ACCELERATION = 135; // degrees per second² (Adjust as needed)
        public double MAX_PROFILE_VELOCITY = 90;     // degrees per second (Adjust as needed)
        public double TIMEOUT_TIME_SECONDS = 5;
    }

    public enum SpecimenArmStates {
        CCW_ARM_HOME, CW_ARM_HOME, SPECIMEN_PICKUP, SPECIMEN_DELIVERY, ARM_MANUAL;
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
    private SpecimenArmStates currentState;
    private SpecimenArmStates targetState;
    private double currentTicks;
    private double currentAngleDegrees;
    private double fudgedCurrentAngleDegrees=Double.NaN;
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
    private double desiredPosition;
    private double targetVelocity;
    private double targetAcceleration;

    // Class-level variables
    private double motionProfileTotalTime;

    // Timer to track elapsed time
    private final com.qualcomm.robotcore.util.ElapsedTime timer;

    // Control Mode
    private enum ControlMode {
        AUTOMATED,
        MANUAL
    }
    private ControlMode controlMode = ControlMode.AUTOMATED;

    public SpecimenArmSubsystem(final HardwareMap hMap, final String name) {
        arm = hMap.get(DcMotorEx.class, name);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        currentState = SpecimenArmStates.CCW_ARM_HOME;
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

        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        currentState = SpecimenArmStates.CCW_ARM_HOME;
        currentAngleDegrees= SPECIMEN_ARM_PARAMS.CCW_HOME;
        targetAngleDegrees = SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE;
        targetState = SpecimenArmStates.SPECIMEN_PICKUP;
        setTargetState(targetState); // Use setTargetState instead of setManualTargetState
    }

    @Override
    public void periodic() {
        // Retrieve current position from the encoder
        currentTicks = armEncoder.getPositionAndVelocity().position;
        currentAngleDegrees = getCurrentArmAngleInDegrees();

        if (controlMode == ControlMode.AUTOMATED && motionProfile != null) {
            motionProfileTotalTime = motionProfile.getTotalTime();
            double elapsedTime = timer.milliseconds();

            // Get target position delta, velocity, and acceleration from the motion profile
            double targetPositionDelta = motionProfile.getPosition(elapsedTime);

            // Retrieve the target velocity and acceleration from the motion profile
            targetVelocity = motionProfile.getVelocity(elapsedTime) * Math.signum(profileDistance);
            targetAcceleration  = motionProfile.getAcceleration(elapsedTime) * Math.signum(profileDistance);

            // Determine the desired position along the motion profile
            desiredPosition = profileStartPosition + (targetPositionDelta * Math.signum(profileDistance));

            // Clip desired position to limits
            desiredPosition = Range.clip(desiredPosition, SPECIMEN_ARM_PARAMS.CW_HOME, SPECIMEN_ARM_PARAMS.CCW_HOME);

            // Update the PID controller's setpoint to follow the motion profile dynamically
            pidController.setSetPoint(desiredPosition);

            if (currentAngleDegrees<= SPECIMEN_ARM_PARAMS.SLOP_SWITCH_ANGLE) {
                fudgedCurrentAngleDegrees=currentAngleDegrees+ SPECIMEN_ARM_PARAMS.CHAIN_SLOP_OFFSET_DEGREES;
                // PID control for position
                pidPower = pidController.calculate(fudgedCurrentAngleDegrees); // Pass current position, not error
                feedforwardPower = armFeedforward.calculate(
                    Math.toRadians(fudgedCurrentAngleDegrees),
                    Math.toRadians(targetVelocity),
                    Math.toRadians(targetAcceleration)
                );
            } else{
                fudgedCurrentAngleDegrees=Double.NaN;
                // PID control for position
                pidPower = pidController.calculate(currentAngleDegrees); // Pass current position, not error
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
            if (motionProfile.isFinished(elapsedTime)) {
                motionProfile = null; // Motion complete
                setCurrentState(targetState);
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
        // Set control mode to automated
        controlMode = ControlMode.AUTOMATED;

        targetState = state;
        profileStartPosition = currentAngleDegrees;
        double profileTargetPosition = state.getArmAngle();
        profileDistance = profileTargetPosition - profileStartPosition;

        // Update targetAngleDegrees with clippingge
        targetAngleDegrees = Range.clip(profileTargetPosition, SPECIMEN_ARM_PARAMS.CW_HOME, SPECIMEN_ARM_PARAMS.CCW_HOME);

        // Initialize motion profile with absolute distance
        double absoluteDistance = Math.abs(profileDistance);

        motionProfile = new TrapezoidalMotionProfile(
                SPECIMEN_ARM_PARAMS.MAX_PROFILE_ACCELERATION,
                SPECIMEN_ARM_PARAMS.MAX_PROFILE_VELOCITY,
                absoluteDistance
        );

        // Start the timer
        timer.reset();


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
        controlMode = ControlMode.MANUAL; // Switch to manual mode
        targetState = SpecimenArmStates.ARM_MANUAL;
        currentState = SpecimenArmStates.ARM_MANUAL;

        // Calculate the change in angle based on input and scale factor
        double deltaAngle = armInput * SPECIMEN_ARM_PARAMS.SCALE_FACTOR;

        // Calculate the new target angle based on the target angle
        targetAngleDegrees += deltaAngle;

        // Clip target angle to within allowed range
        targetAngleDegrees = Range.clip(targetAngleDegrees, SPECIMEN_ARM_PARAMS.CW_HOME, SPECIMEN_ARM_PARAMS.CCW_HOME);

        // Update the PID controller's setpoint with the new target angle
        pidController.setSetPoint(targetAngleDegrees);
    }

    /**
     * Handles manual control by maintaining the target angle through PID.
     */
    private void handleManualControl() {
        //depending on which side of the slop we are on we need an offset
        if (currentAngleDegrees<= SPECIMEN_ARM_PARAMS.SLOP_SWITCH_ANGLE) {
            fudgedCurrentAngleDegrees= currentAngleDegrees+ SPECIMEN_ARM_PARAMS.CHAIN_SLOP_OFFSET_DEGREES;
            pidPower = pidController.calculate(fudgedCurrentAngleDegrees); // Pass current position, not error
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
        updateArmAngles(SpecimenArmStates.CCW_ARM_HOME, SPECIMEN_ARM_PARAMS.CCW_HOME);
        updateArmAngles(SpecimenArmStates.CW_ARM_HOME, SPECIMEN_ARM_PARAMS.CW_HOME);
        updateArmAngles(SpecimenArmStates.SPECIMEN_PICKUP, SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_ANGLE);
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

        // Detailed telemetry for real-time analysis
        MatchConfig.telemetryPacket.put("specimenArm/velocity/Target Arm Velocity", String.format("%.2f", targetVelocity));
        MatchConfig.telemetryPacket.put("specimenArm/velocity/Target Arm Acceleration", String.format("%.2f", targetAcceleration));
        MatchConfig.telemetryPacket.put("specimenArm/velocity/Total Time for Motion Profile (s)", String.format("%.2f", motionProfileTotalTime/1000));

        MatchConfig.telemetryPacket.put("specimenArm/angles/Current Arm Angle", String.format("%.2f", currentAngleDegrees));
        MatchConfig.telemetryPacket.put("specimenArm/angles/Target Arm Angle", String.format("%.2f", targetAngleDegrees));
        MatchConfig.telemetryPacket.put("specimenArm/angles/Motion Profile Desired Arm Angle", String.format("%.2f", desiredPosition));
    }

    private double getCurrentArmAngleInDegrees() {
        // Calculate the base angle from encoder ticks
        double angle = currentTicks / SPECIMEN_ARM_PARAMS.TICKS_PER_DEGREE;

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



    /**
     * Maintains the current position using PID and feedforward.
     */
    private void maintainPosition() {
        //depending on which side of the slop we are on we need an offset
        if (currentAngleDegrees<= SPECIMEN_ARM_PARAMS.SLOP_SWITCH_ANGLE) {
            fudgedCurrentAngleDegrees=currentAngleDegrees+ SPECIMEN_ARM_PARAMS.CHAIN_SLOP_OFFSET_DEGREES;

            // PID control for position
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
        // Calculate PID power to maintain position
        pidPower = pidController.calculate(currentAngleDegrees);

        totalPower = pidPower + feedforwardPower;

        // Clip the total power to the allowable range
        clippedPower = Range.clip(totalPower, -SPECIMEN_ARM_PARAMS.MAX_POWER, SPECIMEN_ARM_PARAMS.MAX_POWER);

        // Set the motor power to move the arm
        arm.setPower(clippedPower);
    }
}
