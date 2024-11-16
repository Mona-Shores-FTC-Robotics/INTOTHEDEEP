package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket;

import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public class SampleLiftBucketSubsystem extends SubsystemBase {
    // Static instance of LIFT_PARAMS

    public static SampleLiftParams SAMPLE_LIFT_PARAMS = new SampleLiftParams();



    public static class SampleLiftParams {
        public double BUCKET_INCREMENT_TIME = 1.0;
        public double KA = .001;
        public double KV = .03;
        public double KG = .04;
        public double KS = 0;

        public  double DUMP_TIME_MS = 800;
        public double SCALE_FACTOR_FOR_MANUAL_LIFT = 50;
        public double LIFT_DEAD_ZONE_FOR_MANUAL_LIFT = 0.05;
        public double LIFT_POWER = 0.5;

        public final int MAX_TARGET_TICKS = 1650;
        public final int MIN_TARGET_TICKS = 0;
        public double TIMEOUT_TIME_SECONDS = 3;
        public int HOME_HEIGHT_TICKS = 0;
        public int HIGH_BASKET_TICKS = 1200;
        public int LOW_BASKET_TICKS = 850;
        public int LIFT_HEIGHT_TICK_THRESHOLD = 30;

        public double VEL_P=0.0000004, VEL_I=0, VEL_D=0;

        // Bucket servo params
        public double BUCKET_SCORE_POS = 0;
        public double BUCKET_INTAKE_POS = .73;

        // Dumper servo params
        public double DUMPER_HOME_POS = 0.41 ;
        public double DUMPER_PRESCORE_POS =.45;
        public double DUMPER_DUMP_POS = 0.65 ;

        public double UPWARD_VELOCITY = 35;     // Ticks per second (adjust as needed)
        public double DOWNWARD_VELOCITY = -1.265;  // Ticks per second (negative for downward)
        public double UPWARD_ACCELERATION = 25;    // Ticks per second squared (adjust as needed)
        public double DOWNWARD_ACCELERATION = -2; // Ticks per second squared (negative for downward)
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
        MOVING_TO_SCORE_POSITION(0),
        MOVING_TO_INTAKE_POSITION(0),
        MOVING_TO_PRE_SCORE_POSITION(0);

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
        DUMPER_DUMP(SAMPLE_LIFT_PARAMS.DUMPER_DUMP_POS),
        DUMPER_PRESCORE(SAMPLE_LIFT_PARAMS.DUMPER_PRESCORE_POS);

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

    private SampleLiftStates currentLiftState;
    private SampleLiftStates targetLiftState;

    ElapsedTime dumperTimer;
    public boolean hasDumped;

    private BucketStates currentBucketState;
    private DumperStates currentDumperState;
    private double targetBucketPosition;
    private double currentBucketPosition;
    private double bucketStepIncrement;
    private final ElapsedTime bucketTimer = new ElapsedTime();
    private boolean movingToTarget = false;

    private ElevatorFeedforward elevatorFeedforward;
    private final PIDController pidController;

    private double pidOutput;
    private double feedforwardOutput;
    private double totalOutput;

    public SampleLiftBucketSubsystem(final HardwareMap hMap, final String liftName, final String bucketName, final String dumperName) {
        lift = hMap.get(DcMotorEx.class, liftName);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorEx.Direction.FORWARD);

        elevatorFeedforward= new ElevatorFeedforward(
                SAMPLE_LIFT_PARAMS.KS,
                SAMPLE_LIFT_PARAMS.KG,
                SAMPLE_LIFT_PARAMS.KV,
                SAMPLE_LIFT_PARAMS.KA);

        pidController = new PIDController(SAMPLE_LIFT_PARAMS.VEL_P, SAMPLE_LIFT_PARAMS.VEL_I, SAMPLE_LIFT_PARAMS.VEL_D);
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
        // Set the motor to RUN_WITHOUT_ENCODER since we're handling control manually
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(0);
        currentLiftState = SampleLiftStates.LIFT_HOME;
        targetLiftState = SampleLiftStates.LIFT_HOME;
        pidController.reset();
        pidController.setSetPoint(currentLiftState.ticks);
        setTargetTicks(currentLiftState.ticks);



        if (bucket != null) {
            setCurrentBucketState(BucketStates.BUCKET_INTAKE_POS);
            currentBucketPosition=BucketStates.BUCKET_INTAKE_POS.position;
            bucket.setPosition(BucketStates.BUCKET_INTAKE_POS.position);
        }
        if (dumper != null) {
            hasDumped = false;
            dumperTimer = new ElapsedTime();
            setCurrentDumperState(DumperStates.DUMPER_HOME);
        }
    }

    public void periodic() {
        currentTicks = lift.getCurrentPosition();

        // Handle dumper timing
        if (currentDumperState == DumperStates.DUMPER_DUMP && dumperTimer.milliseconds() > SAMPLE_LIFT_PARAMS.DUMP_TIME_MS) {
            setCurrentDumperState(DumperStates.DUMPER_HOME);
            hasDumped = true;
        }

        // Handle bucket movement
        if (movingToTarget && bucketTimer.milliseconds() > SAMPLE_LIFT_PARAMS.BUCKET_INCREMENT_TIME) { // Adjust delay as needed
            currentBucketPosition += bucketStepIncrement;
            bucket.setPosition(currentBucketPosition);
            bucketTimer.reset();

            if (currentBucketState==BucketStates.MOVING_TO_INTAKE_POSITION &&
                    currentBucketPosition >= targetBucketPosition )
            {
                bucket.setPosition(BucketStates.BUCKET_INTAKE_POS.position);
                currentBucketState=BucketStates.BUCKET_INTAKE_POS;
                movingToTarget = false;
            } else if (currentBucketState==BucketStates.MOVING_TO_SCORE_POSITION &&
                    currentBucketPosition <= targetBucketPosition)
            {
                bucket.setPosition(BucketStates.BUCKET_SCORE_POS.position);
                currentBucketState=BucketStates.BUCKET_SCORE_POS;
                movingToTarget = false;
            }


        }

        // Update lift control
        updateLiftControl();

        updateLiftState();
        updateParameters();
        updateDashboardTelemetry();
    }

    private void updateLiftControl() {
        currentTicks = lift.getCurrentPosition();

        // Get the PID correction based on the current position
        pidOutput = pidController.calculate(currentTicks);

        // Calculate position difference
        int positionDifference = targetTicks - currentTicks;
        double desiredVelocity;
        double desiredAcceleration;
        // Determine desired velocity and acceleration based on threshold
        if (Math.abs(positionDifference) > SAMPLE_LIFT_PARAMS.LIFT_HEIGHT_TICK_THRESHOLD) {
            if (positionDifference > 0) {
                // Moving up
                desiredVelocity = SAMPLE_LIFT_PARAMS.UPWARD_VELOCITY;
                desiredAcceleration = SAMPLE_LIFT_PARAMS.UPWARD_ACCELERATION;
            } else {
                // Moving down
                desiredVelocity = SAMPLE_LIFT_PARAMS.DOWNWARD_VELOCITY;
                desiredAcceleration = SAMPLE_LIFT_PARAMS.DOWNWARD_ACCELERATION;
            }
        } else {
            // Holding position
            desiredVelocity = 0;
            desiredAcceleration = 0;
        }

        // Calculate the feedforward output
        feedforwardOutput = elevatorFeedforward.calculate(desiredVelocity, desiredAcceleration);

        // Combine feedforward and PID outputs
        totalOutput = pidOutput + feedforwardOutput;

        // Clip the total output to safe power limits
        totalOutput = Range.clip(totalOutput, -1.0, 1.0);

        // Set power to the lift motor
        lift.setPower(totalOutput);
    }

    public void dumpSampleInBucket(){
            dumperTimer.reset();
            setCurrentDumperState(DumperStates.DUMPER_DUMP);
    }

    // **updateLiftPIDs() Method**
    public void updateParameters() {
        // Update PID coefficients if they change
        pidController.setPID(SAMPLE_LIFT_PARAMS.VEL_P, SAMPLE_LIFT_PARAMS.VEL_I, SAMPLE_LIFT_PARAMS.VEL_D);

        elevatorFeedforward = new ElevatorFeedforward(
                SAMPLE_LIFT_PARAMS.KS,
                SAMPLE_LIFT_PARAMS.KG,
                SAMPLE_LIFT_PARAMS.KV,
                SAMPLE_LIFT_PARAMS.KA);

        updateDumperParameters(DumperStates.DUMPER_HOME, SAMPLE_LIFT_PARAMS.DUMPER_HOME_POS);
        updateDumperParameters(DumperStates.DUMPER_DUMP, SAMPLE_LIFT_PARAMS.DUMPER_DUMP_POS);
        updateDumperParameters(DumperStates.DUMPER_PRESCORE, SAMPLE_LIFT_PARAMS.DUMPER_PRESCORE_POS);
        updateBucketParameters(BucketStates.BUCKET_INTAKE_POS, SAMPLE_LIFT_PARAMS.BUCKET_INTAKE_POS);
        updateBucketParameters(BucketStates.BUCKET_SCORE_POS, SAMPLE_LIFT_PARAMS.BUCKET_SCORE_POS);

        updateLiftHeightTicks(SampleLiftStates.LIFT_HOME, SAMPLE_LIFT_PARAMS.HOME_HEIGHT_TICKS);
        updateLiftHeightTicks(SampleLiftStates.HIGH_BASKET, SAMPLE_LIFT_PARAMS.HIGH_BASKET_TICKS);
        updateLiftHeightTicks(SampleLiftStates.LOW_BASKET, SAMPLE_LIFT_PARAMS.LOW_BASKET_TICKS);
    }

    public void setTargetTicks(int ticks) {
        // Clip the ticks to ensure they're within valid limits
        targetTicks = Range.clip(ticks, SAMPLE_LIFT_PARAMS.MIN_TARGET_TICKS, SAMPLE_LIFT_PARAMS.MAX_TARGET_TICKS);

        // Reset the PID controller and set the new setpoint
        pidController.reset();
        pidController.setSetPoint(targetTicks);
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
    }

    public void setCurrentDumperState(DumperStates state){
        currentDumperState = state;
        dumper.setPosition(state.position);
    }

    public void moveDumperToPreScore() {
        currentDumperState= DumperStates.DUMPER_PRESCORE;
        dumper.setPosition(currentDumperState.position);
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

    public void setBucketTargetPositionWithSteps(double targetPosition, int numSteps) {
        targetBucketPosition = targetPosition;
        currentBucketPosition = bucket.getPosition();  // Starting position
        bucketStepIncrement = (targetBucketPosition - currentBucketPosition) / numSteps;
        movingToTarget = true;
        bucketTimer.reset();  // Start timing
    }



    // Basic telemetry display in a single line with a descriptive label
    public void displayBasicTelemetry(Telemetry telemetry) {
        @SuppressLint("DefaultLocale")
        String telemetryData = String.format("%s | Lift Position: %d", currentLiftState, currentTicks);

        if (currentLiftState != targetLiftState) {
            telemetryData += String.format(" | Target State: %s", targetLiftState);
        }

        telemetry.addLine(telemetryData);
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

        MatchConfig.telemetryPacket.put("sampleLift/Current Position Ticks", currentTicks);
        MatchConfig.telemetryPacket.put("sampleLift/Target Position Ticks", targetTicks);

        // Line 1: Lift States
        @SuppressLint("DefaultLocale")
        String statesLine = String.format(
                "Lift State: %s | Target State: %s",
                currentLiftState.toString(),
                targetLiftState.toString()
        );
        MatchConfig.telemetryPacket.put("SampleLift/States", statesLine);

        // Line 2: Positions
        @SuppressLint("DefaultLocale") String positionsLine = String.format(
                "Current Pos: %d | Target Pos: %d",
                currentTicks,
                targetTicks
        );
        MatchConfig.telemetryPacket.put("SampleLift/Positions", positionsLine);

        // Line 3: Power Calculations
        @SuppressLint("DefaultLocale") String powerLine = String.format(
                "PID Out: %.6f | FF Out: %.6f | Total Out: %.6f",
                pidOutput,
                feedforwardOutput,
                totalOutput
        );
        MatchConfig.telemetryPacket.put("SampleLift/Power", powerLine);
    }


    public void moveLiftToHighBasket() {
        setTargetLiftState(SampleLiftStates.HIGH_BASKET);
    }

    public void moveLiftToHome() {
        setTargetLiftState(SampleLiftStates.LIFT_HOME);
    }

    public void moveLiftToLowBasket() {
        setTargetLiftState(SampleLiftStates.LOW_BASKET);
    }

    public void setDumperToHomePosition() {
        setCurrentDumperState(DumperStates.DUMPER_HOME);
    }
    public void setDumperToDumpPosition() {
        setCurrentDumperState(DumperStates.DUMPER_DUMP);
    }


    public void setBucketToIntakePosition() {
        currentBucketState=BucketStates.MOVING_TO_INTAKE_POSITION;
        setBucketTargetPositionWithSteps(SAMPLE_LIFT_PARAMS.BUCKET_INTAKE_POS, 35);
    }

    public void setBucketToScorePosition() {
        currentBucketState=BucketStates.MOVING_TO_SCORE_POSITION;
        setBucketTargetPositionWithSteps(SAMPLE_LIFT_PARAMS.BUCKET_SCORE_POS, 10);
    }

}
