package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public class SpecimenArmSubsystem extends SubsystemBase {
    // Static instance of LIFT_PARAMS
    public static SpecimenArmParams SPECIMEN_ARM_PARAMS = new SpecimenArmParams();

    public static class SpecimenArmParams {
        public double SCALE_FACTOR = 50;
        public double DEAD_ZONE = 0.05;
        public double POWER = 0.5;
        public double VEL_P = 1.0, VEL_I = 0.0, VEL_D = 0.0, VEL_F = 0.0;
        public double POS_P = 0.0;
        public final int MAX_TARGET_TICKS = 800;
        public final int MIN_TARGET_TICKS = 100;
        public double TIMEOUT_TIME_SECONDS = 3;
        public int HOME_HEIGHT_TICKS = 100;
        public int SPECIMEN_PICKUP = 750;
        public int SPECIMEN_DELIVERY = 300;
        public int THRESHOLD = 45;
    }
//todo FIX THIS
    public enum SpecimenArmStates {
        ZERO, SPECIMEN_PICKUP, SPECIMEN_DELIVERY, HOME, MANUAL;
        public int ticks;
        static {
            ZERO.ticks = 0;
            SPECIMEN_PICKUP.ticks = SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP;
            SPECIMEN_DELIVERY.ticks = SPECIMEN_ARM_PARAMS.SPECIMEN_DELIVERY;
            HOME.ticks = SPECIMEN_ARM_PARAMS.HOME_HEIGHT_TICKS;
        }
        public void setLiftHeightTicks(int t) {
            this.ticks = t;
        }
        public int getLiftHeightTicks() {
            return this.ticks;
        }
    }

    public DcMotorEx lift;
    private SpecimenArmStates currentState;
    private SpecimenArmStates targetState;
    private int currentTicks;
    private int targetTicks;
    private double currentPower;

    public SpecimenArmSubsystem(final HardwareMap hMap, final String name) {
        lift = hMap.get(DcMotorEx.class, name);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setVelocityPIDFCoefficients(SPECIMEN_ARM_PARAMS.VEL_P, SPECIMEN_ARM_PARAMS.VEL_I, SPECIMEN_ARM_PARAMS.VEL_D, SPECIMEN_ARM_PARAMS.VEL_F);
        lift.setPositionPIDFCoefficients(SPECIMEN_ARM_PARAMS.POS_P);
        lift.setPower(SPECIMEN_ARM_PARAMS.POWER);
        currentState = SpecimenArmStates.ZERO;
    }

    public void init() {
        //TODO do we need to split into initAuto() and initTeleop()?
        targetState = SpecimenArmStates.HOME;
        setTargetTicks(currentState.ticks);  // Use setTargetTicks to initialize
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
        if (SPECIMEN_ARM_PARAMS.POWER != currentPower) {
            lift.setPower(SPECIMEN_ARM_PARAMS.POWER);
        }
        updateLiftHeightTicks(SpecimenArmStates.HOME, SPECIMEN_ARM_PARAMS.HOME_HEIGHT_TICKS);
        updateLiftHeightTicks(SpecimenArmStates.SPECIMEN_PICKUP, SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP);
        updateLiftHeightTicks(SpecimenArmStates.SPECIMEN_DELIVERY, SPECIMEN_ARM_PARAMS.SPECIMEN_DELIVERY);
        updateLiftVelocityPIDFCoefficients(SPECIMEN_ARM_PARAMS.VEL_P, SPECIMEN_ARM_PARAMS.VEL_I, SPECIMEN_ARM_PARAMS.VEL_D, SPECIMEN_ARM_PARAMS.VEL_F);
        updateLiftPositionPIDFCoefficients(SPECIMEN_ARM_PARAMS.POS_P);
    }

    public void setTargetTicks(int ticks) {
        // Clip the ticks to ensure they're within valid limits
        targetTicks = Range.clip(ticks, SPECIMEN_ARM_PARAMS.MIN_TARGET_TICKS, SPECIMEN_ARM_PARAMS.MAX_TARGET_TICKS);

        // Set the motor's target position
        lift.setTargetPosition(targetTicks);
    }

    public boolean isLiftAtTarget() {
        return Math.abs(currentTicks - targetTicks) < SPECIMEN_ARM_PARAMS.THRESHOLD;
    }

    public void updateLiftState() {
        if (isLiftAtTarget()) {
            setCurrentState(targetState);
        }
    }

    public void setTargetState(SpecimenArmStates state) {
        targetState = state;
        setTargetTicks(state.getLiftHeightTicks());
    }

    public SpecimenArmStates getCurrentState() {
        return currentState;
    }

    public void setCurrentState(SpecimenArmStates state) {
        currentState = state;
    }

    public int getTargetTicks() {
        return targetTicks;
    }

    public SpecimenArmStates getTargetState() {
        return targetState;
    }

    public int getCurrentTicks() {
        return currentTicks;
    }

    // Add a method to handle manual input for the lift
    public void setManualTargetState(double liftInput) {
        targetState = SpecimenArmStates.MANUAL;
        // Calculate the new target ticks based on input
        int deltaTicks = (int) Math.round(liftInput * SPECIMEN_ARM_PARAMS.SCALE_FACTOR);

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

    private void updateLiftHeightTicks(SpecimenArmStates liftState, int newHeightTicks) {
        if (liftState.getLiftHeightTicks() != newHeightTicks) {
            liftState.setLiftHeightTicks(newHeightTicks);
        }
    }

    // Basic telemetry display in a single line with a descriptive label
    public void displayBasicTelemetry(Telemetry telemetry) {
        @SuppressLint("DefaultLocale")
        String telemetryData = String.format("State: %s | Position: %d", currentState, currentTicks);

        if (currentState != targetState) {
            telemetryData += String.format(" | Target State: %s", targetState);
        }

        telemetry.addData("Sample Lift Status", telemetryData);
    }

    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addData("sampleLift/Current State", currentState);
        telemetry.addData("sampleLift/Target State", targetState);
        telemetry.addData("sampleLift/Current Position Ticks", currentTicks);
        telemetry.addData("sampleLift/Target Position Ticks", targetTicks);
        telemetry.addData("sampleLift/Motor Power", lift.getPower());
    }

    public void updateDashboardTelemetry() {
        MatchConfig.telemetryPacket.put("sampleLift/Current State", currentState.toString());
        MatchConfig.telemetryPacket.put("sampleLift/Target State", targetState.toString());
        MatchConfig.telemetryPacket.put("sampleLift/Current Position Ticks", currentTicks);
        MatchConfig.telemetryPacket.put("sampleLift/Target Position Ticks", targetTicks);
        @SuppressLint("DefaultLocale") String statusOverview = String.format("State: %s, Target: %s, Position: %d, Target: %d",
                currentState.toString(), targetState.toString(), currentTicks, targetTicks);
        MatchConfig.telemetryPacket.put("sampleLift/Status Overview", statusOverview);
    }
}
