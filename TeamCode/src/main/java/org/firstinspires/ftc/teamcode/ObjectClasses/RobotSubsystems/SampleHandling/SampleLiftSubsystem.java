package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@Config
public class SampleLiftSubsystem extends SubsystemBase {

    // Static instance of LiftParams
    public static SampleLiftParams LIFT_PARAMS = new SampleLiftParams();

    public static class SampleLiftParams {

        public double SCALE_FACTOR_FOR_MANUAL_LIFT = 150; // Configurable factor for lift scaling
        public double LIFT_DEAD_ZONE_FOR_MANUAL_LIFT = 0.05; // Dead zone threshold for manual control

        // Unified control parameters
        public double LIFT_POWER = 0.5; // Unified power for extending and retracting
        public double VEL_P = 5.0, VEL_I = 0.0, VEL_D = 0.0, VEL_F = 38.0; // Unified PID coefficients
        public double POS_P = 5.0;

        public int MAX_DELTA_TICKS = 50; // Maximum incremental change for manual adjustment
        public double HOLDING_POWER = 0.5; // Power used for holding position

        public final int MAX_TARGET_TICKS = 2800;
        public final int MIN_TARGET_TICKS = 0;

        public int LIFT_HEIGHT_TICK_THRESHOLD = 45;

        public double TIMEOUT_TIME_SECONDS = 2; //this is the time after which a move action/command will give up

        public int HOME_HEIGHT_TICKS = 25;
        public int HIGH_BASKET_TICKS = 2500;
        public int LOW_BASKET_TICKS = 1500;
    }

    public enum SampleLiftStates {
        ZERO, HIGH_BASKET, LOW_BASKET, HOME, MANUAL;

        public int ticks;

        static {
            ZERO.ticks = 0;
            HIGH_BASKET.ticks = LIFT_PARAMS.HIGH_BASKET_TICKS;
            LOW_BASKET.ticks = LIFT_PARAMS.LOW_BASKET_TICKS;
            HOME.ticks = LIFT_PARAMS.HOME_HEIGHT_TICKS;
        }

        public void setLiftHeightTicks(int t) {
            this.ticks = t;
        }

        public int getLiftHeightTicks() {
            return this.ticks;
        }
    }

    // Motor and state variables
    public DcMotorEx lift;
    private SampleLiftStates currentState;
    private SampleLiftStates targetState;
    private int currentTicks;
    private int targetTicks;


    // Constructor to initialize the lift motor
    public SampleLiftSubsystem(final HardwareMap hMap, final String name) {
        lift = hMap.get(DcMotorEx.class, name);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void init() {
        Robot.getInstance().registerSubsystem(Robot.SubsystemType.SAMPLE_LIFT);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Apply unified PID coefficients from LIFT_PARAMS
        lift.setVelocityPIDFCoefficients(LIFT_PARAMS.VEL_P, LIFT_PARAMS.VEL_I, LIFT_PARAMS.VEL_D, LIFT_PARAMS.VEL_F);
        lift.setPositionPIDFCoefficients(LIFT_PARAMS.POS_P);

        // Set the ZeroPowerBehavior to BRAKE to maintain position when power is set to zero
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setPower(LIFT_PARAMS.LIFT_POWER);  // Set default power from params

        currentState = SampleLiftStates.ZERO;
        targetState = SampleLiftStates.ZERO;
        targetTicks = currentState.ticks;

        lift.setTargetPosition(currentState.ticks);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void periodic() {
        // Dynamically update the lift height ticks based on LIFT_PARAMS if they have changed
        updateLiftHeightTicks(SampleLiftStates.HOME, LIFT_PARAMS.HOME_HEIGHT_TICKS);
        updateLiftHeightTicks(SampleLiftStates.HIGH_BASKET, LIFT_PARAMS.HIGH_BASKET_TICKS);
        updateLiftHeightTicks(SampleLiftStates.LOW_BASKET, LIFT_PARAMS.LOW_BASKET_TICKS);

        // Update PID coefficients during runtime if they have changed
        updateLiftVelocityPIDFCoefficients(LIFT_PARAMS.VEL_P, LIFT_PARAMS.VEL_I, LIFT_PARAMS.VEL_D, LIFT_PARAMS.VEL_F);
        updateLiftPositionPIDFCoefficients(LIFT_PARAMS.POS_P);

        // Update the current position of the lift
        currentTicks = lift.getCurrentPosition();

        // Add lift data to telemetry packet with clear and consistent keys for sampleLift
        MatchConfig.telemetryPacket.put("sampleLift/Current State", currentState.toString());
        MatchConfig.telemetryPacket.put("sampleLift/Target State", targetState.toString());
        MatchConfig.telemetryPacket.put("sampleLift/Current Position Ticks", currentTicks);
        MatchConfig.telemetryPacket.put("sampleLift/Target Position Ticks", targetTicks);

        // Provide a status overview to quickly understand the current operation
        String statusOverview = String.format("State: %s, Target: %s, Position: %d, Target: %d",
                currentState.toString(), targetState.toString(), currentTicks, targetTicks);
        MatchConfig.telemetryPacket.put("sampleLift/Status Overview", statusOverview);
    }


    public void setTargetState(SampleLiftStates state) {
        targetState = state;
        lift.setTargetPosition(state.ticks);
    }

    public SampleLiftStates getCurrentState() {
        return currentState;
    }

    public void setCurrentState(SampleLiftStates state) {
        currentState = state;
    }

    public int getTargetTicks() {
        return targetTicks;
    }

    public SampleLiftStates getTargetState() {
        return targetState;
    }

    public void setTargetTicks(int ticks) {
        this.targetTicks = Range.clip(ticks, LIFT_PARAMS.MIN_TARGET_TICKS, LIFT_PARAMS.MAX_TARGET_TICKS);
    }

    public int getCurrentTicks() {
        return currentTicks;
    }

    private void updateLiftVelocityPIDFCoefficients(double p, double i, double d, double f) {
        PIDFCoefficients currentVelocityCoefficients = lift.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (currentVelocityCoefficients.p != p || currentVelocityCoefficients.i != i ||
                currentVelocityCoefficients.d != d || currentVelocityCoefficients.f != f) {
            lift.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    private void updateLiftPositionPIDFCoefficients(double p) {
        // Get the current position PID coefficient used by the motor
        double currentPositionCoefficient = lift.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p;

        // If the desired PID coefficient (p) is different from the current one, update it
        if (currentPositionCoefficient != p) {
            lift.setPositionPIDFCoefficients(p);
        }
    }

    private void updateLiftHeightTicks(SampleLiftStates liftState, int newHeightTicks) {
        if (liftState.getLiftHeightTicks() != newHeightTicks) {
            liftState.setLiftHeightTicks(newHeightTicks);
        }
    }

    public void displayBasicTelemetry(Telemetry telemetry) {
        telemetry.addData("sampleLift/Current State", currentState);
        telemetry.addData("sampleLift/Current Position Ticks", currentTicks);

        // Optional: Display the target state if different from the current state
        if (currentState != targetState) {
            telemetry.addData("sampleLift/Target State", targetState);
        }
    }

    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addData("sampleLift/Current State", currentState);
        telemetry.addData("sampleLift/Target State", targetState);
        telemetry.addData("sampleLift/Current Position Ticks", currentTicks);
        telemetry.addData("sampleLift/Target Position Ticks", targetTicks);
        telemetry.addData("sampleLift/Motor Power", lift.getPower());
    }
}