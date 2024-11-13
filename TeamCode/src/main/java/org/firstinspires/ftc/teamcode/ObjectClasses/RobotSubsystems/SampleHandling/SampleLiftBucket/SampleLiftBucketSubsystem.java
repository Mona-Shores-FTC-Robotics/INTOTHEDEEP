package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket;

import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleButtonHandling;

@Config
public class SampleLiftBucketSubsystem extends SubsystemBase {
    // Static instance of LIFT_PARAMS
    public static SampleLiftParams SAMPLE_LIFT_PARAMS = new SampleLiftParams();

    public static class SampleLiftParams {

        public double POS_P_DOWNWARD = .2;
        public double DOWNWARD_F_MULTIPLIER = 0.2;
        public  double DUMP_TIME_MS = 800;
        public double SCALE_FACTOR_FOR_MANUAL_LIFT = 50;
        public double LIFT_DEAD_ZONE_FOR_MANUAL_LIFT = 0.05;
        public double LIFT_POWER = 0.5;
        public double VEL_P = 5.0, VEL_I = 0.0, VEL_D = 0.0, VEL_F = 30.0;
        public double POS_P = 5.0;
        public final int MAX_TARGET_TICKS = 1650;
        public final int MIN_TARGET_TICKS = 0;
        public double TIMEOUT_TIME_SECONDS = 3;
        public int HOME_HEIGHT_TICKS = 0;
        public int HIGH_BASKET_TICKS = 1200;
        public int LOW_BASKET_TICKS = 850;
        public int LIFT_HEIGHT_TICK_THRESHOLD = 45;

        // Bucket servo params
        public double BUCKET_SCORE_POS = 0;
        public double BUCKET_INTAKE_POS = 0.95;
        public double BUCKET_SAFE_DESCENT_POS4 = .65;
        public double BUCKET_SAFE_DESCENT_POS3 = .55;
        public double BUCKET_SAFE_DESCENT_POS2 = .45;
        public double BUCKET_SAFE_DESCENT_POS1 = .35;

        // Dumper servo params
        public double DUMPER_HOME_POS = .7;
        public double DUMPER_DUMP_POS = .2;
    }

    public enum SampleLiftStates {
        HIGH_BASKET, LOW_BASKET, LIFT_HOME, MANUAL, LOW_BASKET_DOWN;
        public int ticks;
        static {

            HIGH_BASKET.ticks = SAMPLE_LIFT_PARAMS.HIGH_BASKET_TICKS;
            LOW_BASKET_DOWN.ticks = SAMPLE_LIFT_PARAMS.LOW_BASKET_TICKS;
            LOW_BASKET.ticks = SAMPLE_LIFT_PARAMS.LOW_BASKET_TICKS;
            LIFT_HOME.ticks = SAMPLE_LIFT_PARAMS.HOME_HEIGHT_TICKS;
        }
        public void setLiftHeightTicks(int t) {
            this.ticks = t;
        }
        public int getLiftHeightTicks() {
            return this.ticks;
        }
    }

    public enum BucketStates {
        BUCKET_INTAKE_POS(SAMPLE_LIFT_PARAMS.BUCKET_INTAKE_POS),
        BUCKET_SCORE_POS(SAMPLE_LIFT_PARAMS.BUCKET_SCORE_POS),
        BUCKET_SAFE_DESCENT1(SAMPLE_LIFT_PARAMS.BUCKET_SAFE_DESCENT_POS1),
        BUCKET_SAFE_DESCENT2(SAMPLE_LIFT_PARAMS.BUCKET_SAFE_DESCENT_POS2),
        BUCKET_SAFE_DESCENT3(SAMPLE_LIFT_PARAMS.BUCKET_SAFE_DESCENT_POS3),
        BUCKET_SAFE_DESCENT4(SAMPLE_LIFT_PARAMS.BUCKET_SAFE_DESCENT_POS4);

        public double position;

        BucketStates(double position) {
            this.position = position;
        }

        // Dynamically update the intake power if parameters change
        public void updateBucketPosition(double newPosition) {
            this.position = newPosition;
        }
        public double getPosition(BucketStates state) {
            return state.position;
        }

    }
    public enum DumperStates {
        DUMPER_HOME(SAMPLE_LIFT_PARAMS.DUMPER_HOME_POS),
        DUMPER_DUMP(SAMPLE_LIFT_PARAMS.DUMPER_DUMP_POS);

        public double position;

        DumperStates(double position) {
            this.position = position;
        }
        public double getPosition(DumperStates state) {
            return state.position;
        }

        // Dynamically update the intake power if parameters change
        public void updateDumperPosition(double newPosition) {
            this.position = newPosition;
        }
    }

    public DcMotorEx lift;
    public final Servo bucket;  // Continuous rotation servo
    private final Servo dumper;

    private int currentTicks;
    private int targetTicks;
    private double currentPower;

    private SampleLiftStates currentLiftState;
    private SampleLiftStates targetLiftState;

    ElapsedTime dumperTimer;
    public boolean hasDumped;

    private BucketStates currentBucketState;
    private DumperStates currentDumperState;
    private double targetBucketPosition;
    private double currentBucketPosition;
    private double bucketStepIncrement;
    private ElapsedTime bucketTimer = new ElapsedTime();
    private boolean movingToTarget = false;

    public SampleLiftBucketSubsystem(final HardwareMap hMap, final String liftName, final String bucketName, final String dumperName) {
        lift = hMap.get(DcMotorEx.class, liftName);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setVelocityPIDFCoefficients(SAMPLE_LIFT_PARAMS.VEL_P, SAMPLE_LIFT_PARAMS.VEL_I, SAMPLE_LIFT_PARAMS.VEL_D, SAMPLE_LIFT_PARAMS.VEL_F);
        lift.setPositionPIDFCoefficients(SAMPLE_LIFT_PARAMS.POS_P);
        lift.setPower(SAMPLE_LIFT_PARAMS.LIFT_POWER);
        currentLiftState = SampleLiftStates.LIFT_HOME;

        if (bucketName != null){
            bucket = hMap.get(Servo.class, bucketName);
        } else bucket = null;

        if (dumperName != null){
            dumper = hMap.get(Servo.class, dumperName);
        } else dumper = null;
    }
    // Overloaded constructor without color sensor
    public SampleLiftBucketSubsystem(final HardwareMap hMap, final String liftName) {
        this(hMap, liftName, null, null);  // Calls the main constructor with no color sensor
    }

    public void init() {
        targetLiftState = SampleLiftStates.LIFT_HOME;
        setTargetTicks(currentLiftState.ticks);  // Use setTargetTicks to initialize
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (bucket != null){
            setCurrentBucketState(BucketStates.BUCKET_INTAKE_POS);
        }
        if (dumper != null){
            hasDumped=false;
            dumperTimer = new ElapsedTime();
            setCurrentDumperState(DumperStates.DUMPER_HOME);
        }
    }

    public void periodic() {
        currentTicks = lift.getCurrentPosition();
        currentPower = lift.getPower();
        if (currentDumperState == DumperStates.DUMPER_DUMP && dumperTimer.milliseconds()> SAMPLE_LIFT_PARAMS.DUMP_TIME_MS){
            setCurrentDumperState(DumperStates.DUMPER_HOME);
            hasDumped=true;
        }
        // Check if we're in the middle of a bucket movement
        if (movingToTarget && bucketTimer.milliseconds() > 10) { // Adjust delay (10 ms) as needed for smoothness
            currentBucketPosition += bucketStepIncrement;  // Increment position
            bucket.setPosition(currentBucketPosition);     // Move bucket to new position
            bucketTimer.reset();  // Reset timer for the next step

            // Stop moving if we've reached or surpassed the target position
            if ((bucketStepIncrement > 0 && currentBucketPosition >= targetBucketPosition) ||
                    (bucketStepIncrement < 0 && currentBucketPosition <= targetBucketPosition)) {
                bucket.setPosition(targetBucketPosition);  // Ensure final position is exact
                movingToTarget = false;  // Stop further updates
            }
        }

        updateLiftState();
        updateParameters();  // This lets us use the dashboard changes for tuning
        updateDashboardTelemetry();
    }

    public void dumpSampleInBucket(){
            dumperTimer.reset();
            setCurrentDumperState(DumperStates.DUMPER_DUMP);
    }

    // **updateLiftPIDs() Method**
    public void updateParameters() {
        if (SAMPLE_LIFT_PARAMS.LIFT_POWER != currentPower) {
            lift.setPower(SAMPLE_LIFT_PARAMS.LIFT_POWER);
        }
        updateDumperParameters(DumperStates.DUMPER_HOME, SAMPLE_LIFT_PARAMS.DUMPER_HOME_POS);
        updateDumperParameters(DumperStates.DUMPER_DUMP, SAMPLE_LIFT_PARAMS.DUMPER_DUMP_POS);
        updateBucketParameters(BucketStates.BUCKET_INTAKE_POS, SAMPLE_LIFT_PARAMS.BUCKET_INTAKE_POS);
        updateBucketParameters(BucketStates.BUCKET_SCORE_POS, SAMPLE_LIFT_PARAMS.BUCKET_SCORE_POS);

        updateLiftHeightTicks(SampleLiftStates.LIFT_HOME, SAMPLE_LIFT_PARAMS.HOME_HEIGHT_TICKS);
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
    public void setCurrentBucketState(BucketStates state){
        currentBucketState = state;
        bucket.setPosition(state.position);
    }

    public void setCurrentDumperState(DumperStates state){
        currentDumperState = state;
        dumper.setPosition(state.position);
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


    private void updateDumperParameters(DumperStates state, double position) {
        if (state.getPosition(state) != position) {
            state.updateDumperPosition(position);
        }
    }

    private void updateBucketParameters(BucketStates state, double position) {
        if (state.getPosition(state) != position) {
            state.updateBucketPosition(position);
        }
    }


    public BucketStates getCurrentBucketState() { return currentBucketState;}
    public DumperStates getCurrentDumperState() { return currentDumperState;}

    public void setBucketTargetPosition(double targetPosition, int numSteps) {
        targetBucketPosition = targetPosition;
        currentBucketPosition = bucket.getPosition();  // Starting position
        bucketStepIncrement = (targetBucketPosition - currentBucketPosition) / numSteps;
        movingToTarget = true;
        bucketTimer.reset();  // Start timing
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
        MatchConfig.telemetryPacket.put("dumper/Current State", currentDumperState.toString());
        MatchConfig.telemetryPacket.put("bucket/Current State", currentBucketState.toString());

        MatchConfig.telemetryPacket.put("sampleLift/Current State", currentLiftState.toString());
        MatchConfig.telemetryPacket.put("sampleLift/Target State", targetLiftState.toString());
        MatchConfig.telemetryPacket.put("sampleLift/Current Position Ticks", currentTicks);
        MatchConfig.telemetryPacket.put("sampleLift/Target Position Ticks", targetTicks);
        @SuppressLint("DefaultLocale") String statusOverview = String.format("State: %s, Target: %s, Position: %d, Target: %d",
                currentLiftState.toString(), targetLiftState.toString(), currentTicks, targetTicks);
        MatchConfig.telemetryPacket.put("sampleLift/Status Overview", statusOverview);

        // Retrieve and print the PIDF coefficients
        PIDFCoefficients positionCoefficients = lift.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        MatchConfig.telemetryPacket.put("sampleLift/Position P", positionCoefficients.p);
        MatchConfig.telemetryPacket.put("sampleLift/Position I", positionCoefficients.i);
        MatchConfig.telemetryPacket.put("sampleLift/Position D", positionCoefficients.d);
        MatchConfig.telemetryPacket.put("sampleLift/Position F", positionCoefficients.f);

        PIDFCoefficients velocityCoefficients = lift.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        MatchConfig.telemetryPacket.put("sampleLift/Velocity P", velocityCoefficients.p);
        MatchConfig.telemetryPacket.put("sampleLift/Velocity I", velocityCoefficients.i);
        MatchConfig.telemetryPacket.put("sampleLift/Velocity D", velocityCoefficients.d);
        MatchConfig.telemetryPacket.put("sampleLift/Velocity F", velocityCoefficients.f);

    }
}
