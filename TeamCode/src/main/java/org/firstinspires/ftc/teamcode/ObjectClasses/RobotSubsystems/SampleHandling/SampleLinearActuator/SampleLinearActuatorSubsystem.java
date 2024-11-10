package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public class SampleLinearActuatorSubsystem extends SubsystemBase {

    public static class ActuatorParams {

        public double WITHOUT_ENCODER_POWER = 0.7;  // Default power for both directions
        public double SCALE_FACTOR_FOR_MANUAL_ACTUATION = 33;
        public double DEAD_ZONE_FOR_MANUAL_ACTUATION = 0.10;
        public double TIMEOUT_TIME_SECONDS = 3; // Time after which a move action/command will give up
        public double POWER = .7;  // Unified power for both directions

        public double DEPLOYING_TO_FULL_TIME_MS = 600;
        public double DEPLOYING_TO_MID_TIME_MS = 300;
        public double RETRACTION_TIME_MS=700;

    }

    public static ActuatorParams ACTUATOR_PARAMS = new ActuatorParams();

    public enum SampleActuatorStates {
        FULLY_RETRACTED,
        DEPLOYING_TO_MID,
        DEPLOYED_MID,
        DEPLOYING_TO_FULL,
        DEPLOYED_FULLY,
        RETRACTING,
        MANUAL;
    }

    private final DcMotorEx sampleActuator;
    private SampleActuatorStates currentState;
    private double currentPower;
    private DigitalChannel retractedLimitSwitch;
    int currentTicks;

    ElapsedTime actuatorTimer = new ElapsedTime();

    // Constructor with limit switch
    public SampleLinearActuatorSubsystem(HardwareMap hardwareMap, String actuatorMotorName, String limitSwitchName) {
        sampleActuator = hardwareMap.get(DcMotorEx.class, actuatorMotorName);
        sampleActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sampleActuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sampleActuator.setDirection(DcMotorEx.Direction.FORWARD);
        sampleActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentState = SampleActuatorStates.FULLY_RETRACTED;

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
        // Initialize the current and target states to retracted
        currentState = SampleActuatorStates.FULLY_RETRACTED;
        sampleActuator.setPower(0);
    }

    @Override
    public void periodic() {
        // Cache the current actuator position and target ticks
        currentTicks = sampleActuator.getCurrentPosition();
        currentPower = sampleActuator.getPower();

        switch (currentState)
        {
            case DEPLOYING_TO_MID:
                if (actuatorTimer.milliseconds() >= ACTUATOR_PARAMS.DEPLOYING_TO_MID_TIME_MS) {
                    stopActuator();
                    setCurrentState(SampleActuatorStates.DEPLOYED_MID);
                }
                break;
            case DEPLOYING_TO_FULL:
                if (actuatorTimer.milliseconds() >= ACTUATOR_PARAMS.DEPLOYING_TO_FULL_TIME_MS) {
                    stopActuator();
                    setCurrentState(SampleActuatorStates.DEPLOYED_FULLY);
                }
                break;
            case RETRACTING:
                if (actuatorTimer.milliseconds() >= ACTUATOR_PARAMS.RETRACTION_TIME_MS) {
                    stopActuator();
                    setCurrentState(SampleActuatorStates.FULLY_RETRACTED);
                }
                break;
            case DEPLOYED_MID:
            case DEPLOYED_FULLY:
            case MANUAL:
            case FULLY_RETRACTED:
                //do nothing
                break;
        }
        updateDashboardTelemetry();  // Update telemetry each loop
    }

    public void fullyRetract() {
        currentState=SampleActuatorStates.RETRACTING;
        runWithoutEncodersReverse();
        actuatorTimer.reset();
    }

    public void deployMid() {
        currentState=SampleActuatorStates.DEPLOYING_TO_MID;
        runWithoutEncodersForward();
        actuatorTimer.reset();
    }

    public void deployFull() {
        currentState=SampleActuatorStates.DEPLOYING_TO_FULL;
        runWithoutEncodersForward();
        actuatorTimer.reset();
    }


    // Method to power the motor on in one direction without encoders
    public void runWithoutEncodersForward() {
        moveActuator(ACTUATOR_PARAMS.WITHOUT_ENCODER_POWER);
    }

    // Method to power the motor on in reverse without encoders
    public void runWithoutEncodersReverse() {
        moveActuator(-ACTUATOR_PARAMS.WITHOUT_ENCODER_POWER);
    }

    // Add a method to handle manual input for the lift
    public void manualMove(double actuatorInput) {
        // Set the actuator state to MANUAL
        currentState = SampleLinearActuatorSubsystem.SampleActuatorStates.MANUAL;
        double actuatorManualPower = Range.clip(actuatorInput, -.5, .5);
        sampleActuator.setPower(actuatorInput);
    }

    public void setCurrentState(SampleActuatorStates state) {
        currentState = state;
    }

    // Method to check if the actuator is fully retracted
    public boolean isFullyRetracted() {
        if (retractedLimitSwitch == null) {
            System.out.println("Limit switch not configured; assuming fully retracted.");
            return true;
        }   return !retractedLimitSwitch.getState();  // Assuming switch triggers when low
    }
    public int getCurrentTicks() {
        return currentTicks;
    }

    public SampleActuatorStates getCurrentState() {
        return currentState;
    }


    // Method to stop the motor
    public void stopActuator() {
        currentPower = 0;
        sampleActuator.setPower(0);
    }

    // Apply power to move the actuator in (positive power) or out (negative power)
    private void moveActuator(double power) {
        currentPower = Range.clip(power, -1.0, 1.0);  // Ensure power is within valid range
        sampleActuator.setPower(currentPower);
    }

    // Switch to RUN_TO_POSITION mode for precise movement
    public void enableRunToPositionMode() {
        if (sampleActuator.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
            sampleActuator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sampleActuator.setPower(ACTUATOR_PARAMS.POWER);
        }
    }

    // Update dashboard telemetry with actuator state
    public void updateDashboardTelemetry() {
        MatchConfig.telemetryPacket.put("Sample Actuator/Current State", currentState.toString());
        MatchConfig.telemetryPacket.put("Sample Actuator/Current Position Ticks", currentTicks);
    }

    // Compact telemetry display for the driver station
    public void displayBasicTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        @SuppressLint("DefaultLocale")
        String telemetryData = String.format("%s | Pos: %d", currentState != null ? currentState : "Unknown", currentTicks);
        telemetry.addLine(telemetryData);
    }

    // Verbose telemetry display
    public void displayVerboseTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Sample Actuator Current State", currentState);
        telemetry.addData("Sample Actuator Current Position Ticks", currentTicks);
        telemetry.addData("Sample Actuator Motor Power", currentPower);
    }
}
