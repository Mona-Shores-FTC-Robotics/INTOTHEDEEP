package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket;

import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public class SampleLiftBucketSubsystem extends SubsystemBase {
    // Static instance of LIFT_PARAMS
    public static SampleLiftParams SAMPLE_LIFT_PARAMS = new SampleLiftParams();

    public static class SampleLiftParams {
        public double SCALE_FACTOR_FOR_MANUAL_LIFT = 50;
        public double LIFT_DEAD_ZONE_FOR_MANUAL_LIFT = 0.05;
        public double LIFT_POWER = 0.5;
        public double VEL_P = 5.0, VEL_I = 0.0, VEL_D = 0.0, VEL_F = 38.0;
        public double POS_P = 5.0;
        public final int MAX_TARGET_TICKS = 1750;
        public final int MIN_TARGET_TICKS = 100;
        public double TIMEOUT_TIME_SECONDS = 3;
        public int HOME_HEIGHT_TICKS = 100;
        public int HIGH_BASKET_TICKS = 1700;
        public int LOW_BASKET_TICKS = 1100;
        public int LIFT_HEIGHT_TICK_THRESHOLD = 45;


        // Bucket servo params
        public double BUCKET_REST_POS = -0.8;
        public double BUCKET_HIGH_POS = 0.8;

    }


    public enum BucketStates {
        BUCKET_REST_POS(SAMPLE_LIFT_PARAMS.BUCKET_REST_POS),
        BUCKET_HIGH_POS(SAMPLE_LIFT_PARAMS.BUCKET_HIGH_POS);

        public double position;

        BucketStates(double position) {
            this.position = position;
        }

        // Dynamically update the intake power if parameters change
        public void updateBucketPosition(double newPosition) {
            this.position = newPosition;
        }
    }

    public enum SampleLiftStates {
        ZERO, HIGH_BASKET, LOW_BASKET, HOME, MANUAL;
        public int ticks;
        static {
            ZERO.ticks = 0;
            HIGH_BASKET.ticks = SAMPLE_LIFT_PARAMS.HIGH_BASKET_TICKS;
            LOW_BASKET.ticks = SAMPLE_LIFT_PARAMS.LOW_BASKET_TICKS;
            HOME.ticks = SAMPLE_LIFT_PARAMS.HOME_HEIGHT_TICKS;
        }
        public void setLiftHeightTicks(int t) {
            this.ticks = t;
        }
        public int getLiftHeightTicks() {
            return this.ticks;
        }
    }

    public DcMotorEx lift;
    private final Servo bucket;  // Continuous rotation servo
    private SampleLiftStates currentLiftState;
    private SampleLiftStates targetLiftState;
    private int currentTicks;
    private int targetTicks;
    private double currentPower;
    private BucketStates currentBucketState;
    private BucketStates targetBucketState;

    public SampleLiftBucketSubsystem(final HardwareMap hMap, final String liftName, final String bucketName) {
        lift = hMap.get(DcMotorEx.class, liftName);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setVelocityPIDFCoefficients(SAMPLE_LIFT_PARAMS.VEL_P, SAMPLE_LIFT_PARAMS.VEL_I, SAMPLE_LIFT_PARAMS.VEL_D, SAMPLE_LIFT_PARAMS.VEL_F);
        lift.setPositionPIDFCoefficients(SAMPLE_LIFT_PARAMS.POS_P);
        lift.setPower(SAMPLE_LIFT_PARAMS.LIFT_POWER);
        currentLiftState = SampleLiftStates.ZERO;

        if (bucketName != null){
            bucket = hMap.get(Servo.class, bucketName);
        } else bucket = null;
    }
    // Overloaded constructor without color sensor
    public SampleLiftBucketSubsystem(final HardwareMap hMap, final String liftName) {
        this(hMap, liftName, null);  // Calls the main constructor with no color sensor
    }

    public void init() {
        //TODO do we need to split into initAuto() and initTeleop()?
        targetLiftState = SampleLiftStates.HOME;
        setTargetTicks(currentLiftState.ticks);  // Use setTargetTicks to initialize
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (bucket != null){
            setTargetBucketState(BucketStates.BUCKET_REST_POS);
        }
    }

    public void periodic() {
        currentTicks = lift.getCurrentPosition();
        currentPower = lift.getPower();
        updateLiftState();
        updateParameters();  // This lets us use the dashboard changes for tuning
        updateDashboardTelemetry();
    }

    // **updateLiftPIDs() Method**
    public void updateParameters() {
        if (SAMPLE_LIFT_PARAMS.LIFT_POWER != currentPower) {
            lift.setPower(SAMPLE_LIFT_PARAMS.LIFT_POWER);
        }
        updateLiftHeightTicks(SampleLiftStates.HOME, SAMPLE_LIFT_PARAMS.HOME_HEIGHT_TICKS);
        updateLiftHeightTicks(SampleLiftStates.HIGH_BASKET, SAMPLE_LIFT_PARAMS.HIGH_BASKET_TICKS);
        updateLiftHeightTicks(SampleLiftStates.LOW_BASKET, SAMPLE_LIFT_PARAMS.LOW_BASKET_TICKS);
        updateLiftVelocityPIDFCoefficients(SAMPLE_LIFT_PARAMS.VEL_P, SAMPLE_LIFT_PARAMS.VEL_I, SAMPLE_LIFT_PARAMS.VEL_D, SAMPLE_LIFT_PARAMS.VEL_F);
        updateLiftPositionPIDFCoefficients(SAMPLE_LIFT_PARAMS.POS_P);
    }

    public void setTargetTicks(int ticks) {
        // Clip the ticks to ensure they're within valid limits
        targetTicks = Range.clip(ticks, SAMPLE_LIFT_PARAMS.MIN_TARGET_TICKS, SAMPLE_LIFT_PARAMS.MAX_TARGET_TICKS);

        // Set the motor's target position
        lift.setTargetPosition(targetTicks);
    }

    public boolean isLiftAtTarget() {
        return Math.abs(currentTicks - targetTicks) < SAMPLE_LIFT_PARAMS.LIFT_HEIGHT_TICK_THRESHOLD;
    }

    public void updateLiftState() {
        if (isLiftAtTarget()) {
            setCurrentLiftState(targetLiftState);
        }
    }

    public void setTargetLiftState(SampleLiftStates state) {
        targetLiftState = state;
        setTargetTicks(state.getLiftHeightTicks());
    }
    public void setTargetBucketState(BucketStates state){
        targetBucketState = state;
        // TODO Delay time to move to new state?
        currentBucketState = state;
        bucket.setPosition(targetBucketState.position);
    }

    public SampleLiftStates getCurrentLiftState() {
        return currentLiftState;
    }

    public void setCurrentLiftState(SampleLiftStates state) {
        currentLiftState = state;
    }

    public int getTargetTicks() {
        return targetTicks;
    }

    public SampleLiftStates getTargetLiftState() {
        return targetLiftState;
    }

    public int getCurrentTicks() {
        return currentTicks;
    }

    // Add a method to handle manual input for the lift
    public void setManualTargetState(double liftInput) {
        targetLiftState = SampleLiftStates.MANUAL;
        // Calculate the new target ticks based on input
        int deltaTicks = (int) Math.round(liftInput * SAMPLE_LIFT_PARAMS.SCALE_FACTOR_FOR_MANUAL_LIFT);

        // Calculate the new target ticks based on the current target position
        int newTargetTicks = getTargetTicks() + deltaTicks;

        setTargetTicks(newTargetTicks);  // Use setTargetTicks to ensure valid bounds
    }

    private void updateLiftVelocityPIDFCoefficients(double p, double i, double d, double f) {
        PIDFCoefficients currentVelocityCoefficients = lift.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (currentVelocityCoefficients.p != p || currentVelocityCoefficients.i != i ||
                currentVelocityCoefficients.d != d || currentVelocityCoefficients.f != f) {
            lift.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    private void updateLiftPositionPIDFCoefficients(double p) {
        double currentPositionCoefficient = lift.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).p;
        if (currentPositionCoefficient != p) {
            lift.setPositionPIDFCoefficients(p);
        }
    }

    private void updateLiftHeightTicks(SampleLiftStates liftState, int newHeightTicks) {
        if (liftState.getLiftHeightTicks() != newHeightTicks) {
            liftState.setLiftHeightTicks(newHeightTicks);
        }
    }

    // Basic telemetry display in a single line with a descriptive label
    public void displayBasicTelemetry(Telemetry telemetry) {
        @SuppressLint("DefaultLocale")
        String telemetryData = String.format("State: %s | Position: %d", currentLiftState, currentTicks);

        if (currentLiftState != targetLiftState) {
            telemetryData += String.format(" | Target State: %s", targetLiftState);
        }

        telemetry.addData("Sample Lift Status", telemetryData);
    }

    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addData("sampleLift/Current State", currentLiftState);
        telemetry.addData("sampleLift/Target State", targetLiftState);
        telemetry.addData("sampleLift/Current Position Ticks", currentTicks);
        telemetry.addData("sampleLift/Target Position Ticks", targetTicks);
        telemetry.addData("sampleLift/Motor Power", lift.getPower());
    }

    public void updateDashboardTelemetry() {
        MatchConfig.telemetryPacket.put("sampleLift/Current State", currentLiftState.toString());
        MatchConfig.telemetryPacket.put("sampleLift/Target State", targetLiftState.toString());
        MatchConfig.telemetryPacket.put("sampleLift/Current Position Ticks", currentTicks);
        MatchConfig.telemetryPacket.put("sampleLift/Target Position Ticks", targetTicks);
        @SuppressLint("DefaultLocale") String statusOverview = String.format("State: %s, Target: %s, Position: %d, Target: %d",
                currentLiftState.toString(), targetLiftState.toString(), currentTicks, targetTicks);
        MatchConfig.telemetryPacket.put("sampleLift/Status Overview", statusOverview);
    }
}
