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
        public double SCALE_FACTOR = 1;
        public double DEAD_ZONE = 0.05;
        public double POWER = .8;
        public double VEL_P = 0.0, VEL_I = 0.0, VEL_D = 0.0, VEL_F = 45.0;
        public double POS_P = 22.0;
        public final int MAX_TARGET_TICKS = 130;
        public final int MIN_TARGET_TICKS = 10;
        public double TIMEOUT_TIME_SECONDS = 3;
        public int HOME_HEIGHT_TICKS = 100;
        public int SPECIMEN_PICKUP_TICKS = 10;
        public int SPECIMEN_DELIVERY_TICKS = 50;
        public int SPECIMEN_STAGING_TICKS = 130;
        public int THRESHOLD = 5;

        // Angle offsets
        public double STARTING_ANGLE_OFFSET_DEGREES = 21.25; // Angle at STARTING_ENCODER_TICKS

        // Calculated ticks per degree based on motor specs
        public double TICKS_PER_DEGREE = 0.8; // 288 ticks/rev for 360 degrees

    }
    public enum SpecimenArmStates {
        ZERO, SPECIMEN_PICKUP, SPECIMEN_DELIVERY, MANUAL, SPECIMEN_STAGING;
        public int ticks;
        static {
            ZERO.ticks = 0;
            SPECIMEN_PICKUP.ticks = SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_TICKS;
            SPECIMEN_STAGING.ticks = SPECIMEN_ARM_PARAMS.SPECIMEN_STAGING_TICKS;
            SPECIMEN_DELIVERY.ticks = SPECIMEN_ARM_PARAMS.SPECIMEN_DELIVERY_TICKS;
        }
        public void setArmTicks(int t) {
            this.ticks = t;
        }
        public int getArmTicks() {
            return this.ticks;
        }
    }

    public DcMotorEx arm;
    private SpecimenArmStates currentState;
    private SpecimenArmStates targetState;
    private int currentTicks;
    private int targetTicks;
    private double currentPower;

    public SpecimenArmSubsystem(final HardwareMap hMap, final String name) {
        arm = hMap.get(DcMotorEx.class, name);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setVelocityPIDFCoefficients(SPECIMEN_ARM_PARAMS.VEL_P, SPECIMEN_ARM_PARAMS.VEL_I, SPECIMEN_ARM_PARAMS.VEL_D, SPECIMEN_ARM_PARAMS.VEL_F);
        arm.setPositionPIDFCoefficients(SPECIMEN_ARM_PARAMS.POS_P);
        arm.setPower(SPECIMEN_ARM_PARAMS.POWER);
        currentState = SpecimenArmStates.ZERO;
    }

    public void init() {
        //TODO do we need to split into initAuto() and initTeleop()?
        targetState = SpecimenArmStates.ZERO;
        setTargetTicks(currentState.ticks);  // Use setTargetTicks to initialize
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void periodic() {
        currentTicks = arm.getCurrentPosition();
        currentPower = arm.getPower();
        updateArmState();
        updateParameters();
        updateDashboardTelemetry();
        double angleRadians = Math.toRadians(getArmAngle());
    }

    // **updateLiftPIDs() Method**
    public void updateParameters() {
        if (SPECIMEN_ARM_PARAMS.POWER != currentPower) {
            arm.setPower(SPECIMEN_ARM_PARAMS.POWER);
        }
        updateLiftHeightTicks(SpecimenArmStates.SPECIMEN_PICKUP, SPECIMEN_ARM_PARAMS.SPECIMEN_PICKUP_TICKS);
        updateLiftHeightTicks(SpecimenArmStates.SPECIMEN_STAGING, SPECIMEN_ARM_PARAMS.SPECIMEN_STAGING_TICKS);
        updateLiftHeightTicks(SpecimenArmStates.SPECIMEN_DELIVERY, SPECIMEN_ARM_PARAMS.SPECIMEN_DELIVERY_TICKS);
        updateLiftVelocityPIDFCoefficients(SPECIMEN_ARM_PARAMS.VEL_P, SPECIMEN_ARM_PARAMS.VEL_I, SPECIMEN_ARM_PARAMS.VEL_D, SPECIMEN_ARM_PARAMS.VEL_F);
        updateLiftPositionPIDFCoefficients(SPECIMEN_ARM_PARAMS.POS_P);
    }

    public void setTargetTicks(int ticks) {
        // Clip the ticks to ensure they're within valid limits
        targetTicks = Range.clip(ticks, SPECIMEN_ARM_PARAMS.MIN_TARGET_TICKS, SPECIMEN_ARM_PARAMS.MAX_TARGET_TICKS);

        // Set the motor's target position
        arm.setTargetPosition(targetTicks);
    }

    public boolean isArmAtTarget() {
        return Math.abs(currentTicks - targetTicks) < SPECIMEN_ARM_PARAMS.THRESHOLD;
    }

    public void updateArmState() {
        if (isArmAtTarget()) {
            setCurrentState(targetState);
        }
    }

    public void setTargetState(SpecimenArmStates state) {
        targetState = state;
        setTargetTicks(state.getArmTicks());
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
        PIDFCoefficients currentVelocityCoefficients = arm.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (currentVelocityCoefficients.p != p || currentVelocityCoefficients.i != i ||
                currentVelocityCoefficients.d != d || currentVelocityCoefficients.f != f) {
            arm.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    private void updateLiftPositionPIDFCoefficients(double p) {
        double currentPositionCoefficient = arm.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).p;
        if (currentPositionCoefficient != p) {
            arm.setPositionPIDFCoefficients(p);
        }
    }

    private void updateLiftHeightTicks(SpecimenArmStates liftState, int newHeightTicks) {
        if (liftState.getArmTicks() != newHeightTicks) {
            liftState.setArmTicks(newHeightTicks);
        }
    }

    // Basic telemetry display in a single line with a descriptive label
    public void displayBasicTelemetry(Telemetry telemetry) {
        @SuppressLint("DefaultLocale")
        String telemetryData = String.format("State: %s | Position: %d", currentState, currentTicks);

        if (currentState != targetState) {
            telemetryData += String.format(" | Target State: %s", targetState);
        }

        telemetry.addData("Specimen Arm Status", telemetryData);
    }

    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addData("specimenArm/Current State", currentState);
        telemetry.addData("specimenArm/Target State", targetState);
        telemetry.addData("specimenArm/Current Position Ticks", currentTicks);
        telemetry.addData("specimenArm/Target Position Ticks", targetTicks);
        telemetry.addData("specimenArm/Motor Power", arm.getPower());
    }

    public void updateDashboardTelemetry() {
        MatchConfig.telemetryPacket.put("specimenArm/Current Arm Angle", getArmAngle());
        MatchConfig.telemetryPacket.put("specimenArm/Current State", currentState.toString());
        MatchConfig.telemetryPacket.put("specimenArm/Target State", targetState.toString());
        MatchConfig.telemetryPacket.put("specimenArm/Current Position Ticks", currentTicks);
        MatchConfig.telemetryPacket.put("specimenArm/Target Position Ticks", targetTicks);
        @SuppressLint("DefaultLocale") String statusOverview = String.format("State: %s, Target: %s, Position: %d, Target: %d",
                currentState.toString(), targetState.toString(), currentTicks, targetTicks);
        MatchConfig.telemetryPacket.put("specimenArm/Status Overview", statusOverview);
    }

    private double getArmAngle() {
        // Calculate the angle directly from current ticks using ticks per degree
        double angle = currentTicks / SPECIMEN_ARM_PARAMS.TICKS_PER_DEGREE;

        // Adjust by the starting angle offset, if necessary
        angle += SPECIMEN_ARM_PARAMS.STARTING_ANGLE_OFFSET_DEGREES;

        // Ensure angle is within the physical limits of 0 to 210 degrees
        return Range.clip(angle, 0, 210);
    }

}
