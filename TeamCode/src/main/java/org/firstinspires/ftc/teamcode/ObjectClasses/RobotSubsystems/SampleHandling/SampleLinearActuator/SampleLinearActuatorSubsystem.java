package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;

@Config
public class SampleLinearActuatorSubsystem extends SubsystemBase {

    public static class ActuatorParams {
        public double SCALE_FACTOR_FOR_MANUAL_ACTUATION = 150;
        public double DEAD_ZONE_FOR_MANUAL_ACTUATION = 0.05;
        public double MAX_DELTA_TICKS = 50;
        public int TICK_THRESHOLD = 50;
        public double TIMEOUT_TIME_SECONDS = 3; // Time after which a move action/command will give up

        public double POWER = .4;  // Unified power for both directions
        public int DEPLOY_FULL_POSITION_TICKS = 1000;
        public int DEPLOY_MID_POSITION_TICKS = 500;
        public int RETRACT_POSITION_TICKS = 0;
        public double VEL_P = .5;
        public double VEL_I = 0;
        public double VEL_D = 0;
        public double VEL_F = 0;
    }

    public static ActuatorParams ACTUATOR_PARAMS = new ActuatorParams();

    public enum SampleActuatorStates {
        DEPLOY_FULL(ACTUATOR_PARAMS.DEPLOY_FULL_POSITION_TICKS),
        DEPLOY_MID(ACTUATOR_PARAMS.DEPLOY_MID_POSITION_TICKS),
        RETRACT(ACTUATOR_PARAMS.RETRACT_POSITION_TICKS),
        MANUAL(0);  // Power will be set dynamically

        public int targetPositionTicks;

        SampleActuatorStates(int positionTicks) {
            this.targetPositionTicks = positionTicks;
        }

        public int getTargetPositionTicks() {
            return targetPositionTicks;
        }

        // Add this method to set the target position ticks dynamically
        public void setTargetPositionTicks(int positionTicks) {
            this.targetPositionTicks = positionTicks;
        }
    }

    private final DcMotorEx sampleActuator;
    private SampleActuatorStates currentState;
    private SampleActuatorStates targetState;
    private int currentTicks;  // Cached current position
    private int targetTicks;   // Cached target position
    private double currentPower;
    FieldConstants.AllianceColor colorSensor = null;
    private DigitalChannel retractedLimitSwitch;

    // Constructor with limit switch
    public SampleLinearActuatorSubsystem(HardwareMap hardwareMap, String actuatorMotorName, String limitSwitchName) {
        sampleActuator = hardwareMap.get(DcMotorEx.class, actuatorMotorName);

        // Initialize the limit switch if the name is provided
        if (limitSwitchName != null && !limitSwitchName.isEmpty()) {
            retractedLimitSwitch = hardwareMap.get(DigitalChannel.class, limitSwitchName);
            retractedLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        }
    }

    // Constructor without limit switch
    public SampleLinearActuatorSubsystem(HardwareMap hardwareMap, String actuatorMotorName) {
        this(hardwareMap, actuatorMotorName, null);  // Calls the main constructor with no limit switch name
    }

    // Initialize actuator motor with encoders and PID configuration
    public void init() {
        // Register the subsystem
        sampleActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  // Reset encoders on init
        sampleActuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sampleActuator.setPower(ACTUATOR_PARAMS.POWER);
        sampleActuator.setDirection(DcMotorEx.Direction.FORWARD);
        sampleActuator.setVelocityPIDFCoefficients(ACTUATOR_PARAMS.VEL_P, ACTUATOR_PARAMS.VEL_I, ACTUATOR_PARAMS.VEL_D, ACTUATOR_PARAMS.VEL_F);
        sampleActuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize the current and target states to retracted
        currentState = SampleActuatorStates.RETRACT;
        targetState = SampleActuatorStates.RETRACT;
        setTargetTicks(currentState.getTargetPositionTicks());

        // Set the mode to RUN_TO_POSITION for precise control
        sampleActuator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void periodic() {
        // Cache the current actuator position and target ticks
        currentTicks = sampleActuator.getCurrentPosition();
        targetTicks = targetState.getTargetPositionTicks();
        currentPower = sampleActuator.getPower();
        updateActuatorState();
        updateParameters();
        updateDashboardTelemetry();  // Update telemetry each loop
        if (colorSensor!=null) {
            if (colorSensor != MatchConfig.finalOpponentColor) {
                setTargetState(SampleActuatorStates.RETRACT);
            } else if (colorSensor == MatchConfig.finalOpponentColor) {
                Robot.getInstance().getSampleIntakeSubsystem().setCurrentState(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_REVERSE);
            }
        }
    }

    // Set the target state of the actuator
    public void setTargetState(SampleActuatorStates state) {
        targetState = state;
        setTargetTicks(state.getTargetPositionTicks());
    }

    // Set the target ticks, applying limits and setting the target position on the motor
    public void setTargetTicks(int ticks) {
        targetTicks = Range.clip(ticks, ACTUATOR_PARAMS.RETRACT_POSITION_TICKS, ACTUATOR_PARAMS.DEPLOY_FULL_POSITION_TICKS);
        sampleActuator.setTargetPosition(targetTicks);
    }

    // Check if the actuator has reached its target
    public boolean isActuatorAtTarget() {
        return Math.abs(currentTicks - targetTicks) < ACTUATOR_PARAMS.TICK_THRESHOLD;
    }

    // Method to check if the actuator is fully retracted
    public boolean isFullyRetracted() {
        if (retractedLimitSwitch == null) {
            System.out.println("Limit switch not configured; assuming fully retracted.");
            return currentState == SampleActuatorStates.RETRACT && isActuatorAtTarget();
        }
        return !retractedLimitSwitch.getState();  // Assuming switch triggers when low
    }

    // Update the actuator state based on whether it has reached its target
    public void updateActuatorState() {
        if (isActuatorAtTarget()) {
            currentState = targetState;
        }
    }
    public void updateParameters() {
        // Update power
        if (ACTUATOR_PARAMS.POWER != currentPower) {
            sampleActuator.setPower(ACTUATOR_PARAMS.POWER);
        }

        // Update target positions
        updateActuatorPositionTicks(SampleActuatorStates.DEPLOY_FULL, ACTUATOR_PARAMS.DEPLOY_FULL_POSITION_TICKS);
        updateActuatorPositionTicks(SampleActuatorStates.RETRACT, ACTUATOR_PARAMS.RETRACT_POSITION_TICKS);

        // Update velocity PID coefficients
        updateActuatorPID(ACTUATOR_PARAMS.VEL_P, ACTUATOR_PARAMS.VEL_I, ACTUATOR_PARAMS.VEL_D, ACTUATOR_PARAMS.VEL_F);
    }

    // Method to update actuator target ticks if they've been changed in ACTUATOR_PARAMS
    private void updateActuatorPositionTicks(SampleActuatorStates actuatorState, int newTicks) {
        if (actuatorState.getTargetPositionTicks() != newTicks) {
            actuatorState.setTargetPositionTicks(newTicks);
        }
    }

    // Method to update PIDF coefficients if they've changed
    private void updateActuatorPID(double p, double i, double d, double f) {
        PIDFCoefficients currentVelocityCoefficients = sampleActuator.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (currentVelocityCoefficients.p != p || currentVelocityCoefficients.i != i ||
                currentVelocityCoefficients.d != d || currentVelocityCoefficients.f != f) {
            sampleActuator.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    public void setManualTargetState(double actuatorInput) {
        // Set the actuator state to MANUAL
        setTargetState(SampleActuatorStates.MANUAL);

        // Calculate the new target ticks based on input
        int deltaTicks = (int) Math.round(actuatorInput * ACTUATOR_PARAMS.SCALE_FACTOR_FOR_MANUAL_ACTUATION);

        // Clip the delta to avoid large movements
        deltaTicks = (int) Range.clip(deltaTicks, -ACTUATOR_PARAMS.MAX_DELTA_TICKS, ACTUATOR_PARAMS.MAX_DELTA_TICKS);

        // Calculate the new target ticks based on the current target position
        int newTargetTicks = targetTicks + deltaTicks;

        setTargetTicks(newTargetTicks);  // Use setTargetTicks to ensure valid bounds
    }

    // Update dashboard telemetry with actuator state
    public void updateDashboardTelemetry() {
        MatchConfig.telemetryPacket.put("Sample Actuator/Current State", currentState.toString());
        MatchConfig.telemetryPacket.put("Sample Actuator/Target State", targetState.toString());
        MatchConfig.telemetryPacket.put("Sample Actuator/Current Position Ticks", currentTicks);
        MatchConfig.telemetryPacket.put("Sample Actuator/Target Position Ticks", targetTicks);
    }

    // Basic telemetry display in a single line with a descriptive label
    public void displayBasicTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        @SuppressLint("DefaultLocale")
        String telemetryData = String.format("State: %s | Position: %d", currentState, currentTicks);

        if (currentState != targetState) {
            telemetryData += String.format(" | Target State: %s", targetState);
        }

        telemetry.addData("Linear Actuator Status", telemetryData);
    }

    // Verbose telemetry display
    public void displayVerboseTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Sample Actuator Current State", currentState);
        telemetry.addData("Sample Actuator Target State", targetState);
        telemetry.addData("Sample Actuator Current Position Ticks", currentTicks);
        telemetry.addData("Sample Actuator Target Position Ticks", targetTicks);
        telemetry.addData("Sample Actuator Motor Power", sampleActuator.getPower());
    }

    public int getCurrentTicks() {
        return currentTicks;
    }

    public int getTargetTicks() {
        return targetTicks;
    }

    public SampleActuatorStates getCurrentState() {
        return currentState;
    }

}
