package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket;

import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
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
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ConfigurableParameters;
import org.firstinspires.ftc.teamcode.messages.MonaShoresMessages.SampleBucketServoArmMessage;
import org.firstinspires.ftc.teamcode.messages.MonaShoresMessages.SampleDumperMessage;
import org.firstinspires.ftc.teamcode.messages.MonaShoresMessages.SampleLiftMessage;

@Config
public class SampleLiftBucketSubsystem extends SubsystemBase {

    public static class SampleLiftParams extends ConfigurableParameters {
        public double BUCKET_INCREMENT_TIME, KA, KV, KG, KS;
        public double DUMP_TIME_MS, SCALE_FACTOR_FOR_MANUAL_LIFT, LIFT_DEAD_ZONE_FOR_MANUAL_LIFT, LIFT_POWER;
        public int MAX_TARGET_TICKS, MIN_TARGET_TICKS, HOME_HEIGHT_TICKS, HIGH_BASKET_TICKS, LOW_BASKET_TICKS, LIFT_HEIGHT_TICK_THRESHOLD;
        public double TIMEOUT_TIME_SECONDS, VEL_P, VEL_I, VEL_D;
        public double BUCKET_SCORE_POS, BUCKET_INTAKE_POS;
        public double DUMPER_HOME_POS, DUMPER_PRESCORE_POS, DUMPER_DUMP_POS;
        public double UPWARD_VELOCITY, DOWNWARD_VELOCITY, UPWARD_ACCELERATION, DOWNWARD_ACCELERATION;

        @Override
        public void loadDefaultsForRobotType(Robot.RobotType robotType) {
            if (haveRobotSpecificParametersBeenLoaded()) return; // Skip reloading if already customized

            switch (robotType) {
                case INTO_THE_DEEP_19429:

                    // Feedforward Coefficients
                    KA = 0.0010;    KV = 0.002;    KG = 0.043;    KS = 0.0025;
                    VEL_P = 0.00009;    VEL_I = 0.05;    VEL_D = .000025;

                    LIFT_POWER = 0.5;
                    TIMEOUT_TIME_SECONDS = 3.0;
                    MAX_TARGET_TICKS = 1135;    MIN_TARGET_TICKS = 0;     LIFT_HEIGHT_TICK_THRESHOLD = 15;
                    HOME_HEIGHT_TICKS = 0;      HIGH_BASKET_TICKS = 1115;   LOW_BASKET_TICKS = 850;
                    SCALE_FACTOR_FOR_MANUAL_LIFT = 50;    LIFT_DEAD_ZONE_FOR_MANUAL_LIFT = 0.05;

                    UPWARD_VELOCITY = 180;    DOWNWARD_VELOCITY = 5;    UPWARD_ACCELERATION = 120;    DOWNWARD_ACCELERATION = 2;

                    // Bucket Servo Positions
                    BUCKET_SCORE_POS = .32;    BUCKET_INTAKE_POS = .85;
                    BUCKET_INCREMENT_TIME = 1.0;

                    // Dumper Positions
                    DUMPER_HOME_POS = 0.678;    DUMPER_PRESCORE_POS = 0.74;    DUMPER_DUMP_POS = 0.98;
                    DUMP_TIME_MS = 800;

                    break;

                case INTO_THE_DEEP_20245:

                    // Feedforward Coefficients
                    KA = 0;    KV = 0;    KG = .045;    KS = 0;
                    VEL_P = .001;    VEL_I = .095;    VEL_D = 0.0;

                    // Lift Parameters
                    LIFT_POWER = 0.5;
                    TIMEOUT_TIME_SECONDS = 3.0;
                    SCALE_FACTOR_FOR_MANUAL_LIFT = 50;    LIFT_DEAD_ZONE_FOR_MANUAL_LIFT = 0.05;
                    MAX_TARGET_TICKS = 1150;    MIN_TARGET_TICKS = 0;     LIFT_HEIGHT_TICK_THRESHOLD = 15;
                    HOME_HEIGHT_TICKS = 0;      HIGH_BASKET_TICKS = 1125;   LOW_BASKET_TICKS = 850;
                    UPWARD_VELOCITY = 5;    DOWNWARD_VELOCITY = -18;    UPWARD_ACCELERATION = 2;    DOWNWARD_ACCELERATION = -1;

                    // Bucket Positions
                    BUCKET_SCORE_POS = 0.2;    BUCKET_INTAKE_POS = .775;

                    // Timing and Increment
                    BUCKET_INCREMENT_TIME = 1.0;

                    // Dumper Positions
                    DUMPER_HOME_POS = 0.75;    DUMPER_PRESCORE_POS = .81;    DUMPER_DUMP_POS = 0.98;
                    DUMP_TIME_MS = 800;

                    break;
                default:
                    throw new IllegalArgumentException("Unknown robot type: " + robotType);
            }
            markRobotSpecificParametersLoaded(); // Flag parameters as customized
        }
    }

    public static SampleLiftParams SAMPLE_LIFT_PARAMS = new SampleLiftParams();


    public enum SampleLiftStates {
        HIGH_BASKET, LOW_BASKET, LIFT_HOME, MANUAL, LOW_BASKET_DOWN;

        public int getLiftHeightTicks() {
            switch (this) {
                case HIGH_BASKET:
                    return SAMPLE_LIFT_PARAMS.HIGH_BASKET_TICKS;
                case LOW_BASKET:
                    return SAMPLE_LIFT_PARAMS.LOW_BASKET_TICKS;
                case LIFT_HOME:
                    return  SAMPLE_LIFT_PARAMS.HOME_HEIGHT_TICKS;
                default:
                    throw new IllegalStateException("Power not defined for state: " + this);
            }
        }
    }

    public enum BucketStates {
        BUCKET_INTAKE_POS,
        BUCKET_SCORE_POS,
        MOVING_TO_SCORE_POSITION,
        MOVING_TO_INTAKE_POSITION,
        MOVING_TO_PRE_SCORE_POSITION;

        public double getBucketPosition() {
            switch (this) {
                case BUCKET_INTAKE_POS:
                    return SAMPLE_LIFT_PARAMS.BUCKET_INTAKE_POS;
                case BUCKET_SCORE_POS:
                    return SAMPLE_LIFT_PARAMS.BUCKET_SCORE_POS;
                default:
                    throw new IllegalStateException("Power not defined for state: " + this);
            }
        }
    }

    public enum DumperStates {
        DUMPER_HOME,
        DUMPER_DUMP,
        DUMPER_PRESCORE;

        public double getDumperPosition() {
            switch (this) {
                case DUMPER_HOME:
                    return SAMPLE_LIFT_PARAMS.DUMPER_HOME_POS;
                case DUMPER_PRESCORE:
                    return SAMPLE_LIFT_PARAMS.DUMPER_PRESCORE_POS;
                case DUMPER_DUMP:
                    return SAMPLE_LIFT_PARAMS.DUMPER_DUMP_POS;
                default:
                    throw new IllegalStateException("Power not defined for state: " + this);
            }
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

    // Cache for current parameters to detect changes
    private double currentVelP = SAMPLE_LIFT_PARAMS.VEL_P;
    private double currentVelI = SAMPLE_LIFT_PARAMS.VEL_I;
    private double currentVelD = SAMPLE_LIFT_PARAMS.VEL_D;

    private double currentKs = SAMPLE_LIFT_PARAMS.KS;
    private double currentKg = SAMPLE_LIFT_PARAMS.KG;
    private double currentKv = SAMPLE_LIFT_PARAMS.KV;
    private double currentKa = SAMPLE_LIFT_PARAMS.KA;

    public SampleLiftBucketSubsystem(final HardwareMap hMap, final Robot.RobotType robotType, final String liftName, final String bucketName, final String dumperName) {
        SAMPLE_LIFT_PARAMS.loadDefaultsForRobotType(robotType);
        lift = hMap.get(DcMotorEx.class, liftName);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorEx.Direction.FORWARD);

        elevatorFeedforward= new ElevatorFeedforward(
                SAMPLE_LIFT_PARAMS.KS,
                SAMPLE_LIFT_PARAMS.KG,
                SAMPLE_LIFT_PARAMS.KV,
                SAMPLE_LIFT_PARAMS.KA);
//todo manual lift has to also raise bucket arm
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
    public SampleLiftBucketSubsystem(final HardwareMap hMap, Robot.RobotType robotType, final String liftName) {
        this(hMap, robotType, liftName, null, null);  // Calls the main constructor with no color sensor
    }

    public void init() {
        // Set the motor to RUN_WITHOUT_ENCODER since we're handling control manually
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(0);
        currentLiftState = SampleLiftStates.LIFT_HOME;
        targetLiftState = SampleLiftStates.LIFT_HOME;
        pidController.reset();
        pidController.setSetPoint(currentLiftState.getLiftHeightTicks());
        setTargetTicks(currentLiftState.getLiftHeightTicks());

        if (bucket != null) {
            setCurrentBucketState(BucketStates.BUCKET_INTAKE_POS);
            bucket.setPosition(currentBucketState.getBucketPosition());
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
                bucket.setPosition(BucketStates.BUCKET_INTAKE_POS.getBucketPosition());
                currentBucketState=BucketStates.BUCKET_INTAKE_POS;
                movingToTarget = false;
            } else if (currentBucketState==BucketStates.MOVING_TO_SCORE_POSITION &&
                    currentBucketPosition <= targetBucketPosition)
            {
                bucket.setPosition(BucketStates.BUCKET_SCORE_POS.getBucketPosition());
                currentBucketState=BucketStates.BUCKET_SCORE_POS;
                movingToTarget = false;
            }

        }

        // Update lift control
        updateLiftControl();
        updateLiftState();

        FlightRecorder.write("SAMPLE_DUMPER", new SampleDumperMessage(currentDumperState));
        FlightRecorder.write("SAMPLE_BUCKET_ARM", new SampleBucketServoArmMessage(currentBucketState));
        FlightRecorder.write("SAMPLE_LIFT", new SampleLiftMessage(currentLiftState));

        updateParameters();
        updateDashboardTelemetry();
    }

    private void updateLiftControl() {
        currentTicks = lift.getCurrentPosition();

        // Get the PID correction based on the current position
        pidOutput = pidController.calculate(currentTicks);

        // Calculate position difference
        int positionDifference = targetTicks - currentTicks;
        double desiredVelocity = 0;
        double desiredAcceleration= 0;
        // Determine desired velocity and acceleration based on threshold
        if (targetLiftState==SampleLiftStates.LIFT_HOME) {
            // Moving up
            desiredVelocity = -SAMPLE_LIFT_PARAMS.DOWNWARD_VELOCITY;        // Negative value
            desiredAcceleration = -SAMPLE_LIFT_PARAMS.DOWNWARD_ACCELERATION; // Negative value
        } else if (targetLiftState==SampleLiftStates.HIGH_BASKET)
        {
            desiredVelocity = SAMPLE_LIFT_PARAMS.UPWARD_VELOCITY;        // Positive value
            desiredAcceleration = SAMPLE_LIFT_PARAMS.UPWARD_ACCELERATION; // Positive value
        } else {
            desiredVelocity=0;
            desiredAcceleration=0;
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
        // Update PID coefficients only if they change
        if (currentVelP != SAMPLE_LIFT_PARAMS.VEL_P ||
                currentVelI != SAMPLE_LIFT_PARAMS.VEL_I ||
                currentVelD != SAMPLE_LIFT_PARAMS.VEL_D) {

            pidController.setPID(SAMPLE_LIFT_PARAMS.VEL_P, SAMPLE_LIFT_PARAMS.VEL_I, SAMPLE_LIFT_PARAMS.VEL_D);

            // Update cache
            currentVelP = SAMPLE_LIFT_PARAMS.VEL_P;
            currentVelI = SAMPLE_LIFT_PARAMS.VEL_I;
            currentVelD = SAMPLE_LIFT_PARAMS.VEL_D;
        }

        // Update feedforward parameters only if they change
        if (currentKs != SAMPLE_LIFT_PARAMS.KS ||
                currentKg != SAMPLE_LIFT_PARAMS.KG ||
                currentKv != SAMPLE_LIFT_PARAMS.KV ||
                currentKa != SAMPLE_LIFT_PARAMS.KA) {

            elevatorFeedforward = new ElevatorFeedforward(
                    SAMPLE_LIFT_PARAMS.KS,
                    SAMPLE_LIFT_PARAMS.KG,
                    SAMPLE_LIFT_PARAMS.KV,
                    SAMPLE_LIFT_PARAMS.KA
            );

            // Update cache
            currentKs = SAMPLE_LIFT_PARAMS.KS;
            currentKg = SAMPLE_LIFT_PARAMS.KG;
            currentKv = SAMPLE_LIFT_PARAMS.KV;
            currentKa = SAMPLE_LIFT_PARAMS.KA;
        }
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
        dumper.setPosition(state.getDumperPosition());
    }

    public void moveDumperToPreScore() {
        currentDumperState= DumperStates.DUMPER_PRESCORE;
        dumper.setPosition(currentDumperState.getDumperPosition());
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

    public BucketStates getCurrentBucketState() { return currentBucketState;}

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

    //todo consider adjusting the bucket servo arm goes down/up to help with auto?
    public void setBucketToIntakePosition() {
        currentBucketState=BucketStates.MOVING_TO_INTAKE_POSITION;
        setBucketTargetPositionWithSteps(SAMPLE_LIFT_PARAMS.BUCKET_INTAKE_POS, 35);
    }

    public void setBucketToScorePosition() {
        currentBucketState=BucketStates.MOVING_TO_SCORE_POSITION;
        setBucketTargetPositionWithSteps(SAMPLE_LIFT_PARAMS.BUCKET_SCORE_POS, 10);
    }

}
