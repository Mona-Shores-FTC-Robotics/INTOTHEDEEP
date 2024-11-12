package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import android.annotation.SuppressLint;

import com.example.sharedconstants.FieldConstants;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.GamePieceDetector.DetectionState;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleDetector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleProcessingStateMachine;

@Config
public class SampleIntakeSubsystem extends SubsystemBase {

    public static class IntakeParams {
        public double INTAKE_ON_POWER = 0.8;
        public double INTAKE_REVERSE_POWER = -0.5;
        public double INTAKE_OFF_POWER = 0.0;
        public double MAX_POWER = 1.0;  // Max allowable power for intake servo
        // Set a minimum proximity threshold to consider an object as "near"
        public double PROXIMITY_THRESHOLD = 40;
        public int COLOR_HISTORY_SIZE = 5;
        public double TRANSFER_TIME_MS= 400;
        public double EJECT_TIME_MS= 400;
    }

    public static IntakeParams INTAKE_PARAMS = new IntakeParams();

    public enum SampleIntakeStates {
        REVERSING_INTAKE_TO_EJECT(INTAKE_PARAMS.INTAKE_REVERSE_POWER),
        REVERSING_INTAKE_TO_TRANSFER(INTAKE_PARAMS.INTAKE_REVERSE_POWER),
        INTAKE_ON(INTAKE_PARAMS.INTAKE_ON_POWER),
        INTAKE_REVERSE(INTAKE_PARAMS.INTAKE_REVERSE_POWER),
        INTAKE_OFF(INTAKE_PARAMS.INTAKE_OFF_POWER);
        public double power;
        SampleIntakeStates(double power) {
            this.power = power;
        }
        // Dynamically update the intake power if parameters change
        public void updateIntakePower(double newPower) {
            this.power = newPower;
        }
    }

    private final CRServo sampleIntakeLeft;  // Continuous rotation servo
    private final CRServo sampleIntakeRight;  // Continuous rotation servo

    private final RevColorSensorV3 colorSensor;  // Nullable color sensor
    private SampleIntakeStates currentSampleIntakeState;
    private double currentPower;

    private final SampleDetector sampleDetector;
    private SampleProcessingStateMachine sampleProcessingStateMachine;

    ElapsedTime sampleIntakeTimer = new ElapsedTime();

    // Constructor with color sensor
    public SampleIntakeSubsystem(final HardwareMap hMap, final String intakeServoL, final String intakeServoR, final String colorSensorName) {
        sampleIntakeLeft = hMap.get(CRServo.class, intakeServoL);
        sampleIntakeRight = hMap.get(CRServo.class, intakeServoR);

        colorSensor = colorSensorName != null ? hMap.get(RevColorSensorV3.class, colorSensorName) : null;

        if (colorSensor != null) {
            sampleDetector = new SampleDetector(colorSensor, INTAKE_PARAMS.PROXIMITY_THRESHOLD, INTAKE_PARAMS.COLOR_HISTORY_SIZE);
        } else {
            sampleDetector = null; // no detector if sensor is unavailable;
        }
    }

    // Overloaded constructor without color sensor
    public SampleIntakeSubsystem(final HardwareMap hMap, final String intakeServoL, final String intakeServoR) {
        this(hMap, intakeServoL, intakeServoR, null);  // Calls the main constructor with no color sensor
    }

    // Initialize intake servo
    public void init() {
        setCurrentState(SampleIntakeStates.INTAKE_OFF);  // Set default state to off
        currentPower = INTAKE_PARAMS.INTAKE_OFF_POWER;  // Cache initial power
        sampleProcessingStateMachine = Robot.getInstance().getSampleProcessingStateMachine();
    }

    @Override
    public void periodic() {
        if (sampleDetector != null) {
            DetectionState detectionState = sampleDetector.updateDetection();

            switch(detectionState) {
                case JUST_DETECTED:
                    handleSamplePickup();
                    sampleProcessingStateMachine.updateSampleProcessingState();
                    break;

                case STILL_DETECTED:
                    sampleProcessingStateMachine.updateSampleProcessingState();
                    handleEjectionAndTransferStates();
                    break;

                case NOT_DETECTED:
                    // Do nothing
                    break;
            }
        }
        updateParameters();
        updateDashboardTelemetry();
    }


    public void handleSamplePickup() {
        if (sampleDetector.isGoodSample()) {
            //If we detect 1) yellow; or 2) blue and we are blue; or 3) red and we are red, then its a good sample
            sampleProcessingStateMachine.setOnGoodSampleDetectionState();
        } else if (sampleDetector.isBadSample()) {
            //Otherwise its a bad sample
            sampleProcessingStateMachine.setOnBadSampleDetectionState();
        }
    }

    private void handleEjectionAndTransferStates() {
        switch (currentSampleIntakeState) {
            case REVERSING_INTAKE_TO_TRANSFER:
                if (sampleIntakeTimer.milliseconds() >= INTAKE_PARAMS.TRANSFER_TIME_MS) {
                    setCurrentState(SampleIntakeStates.INTAKE_OFF);
                }
                break;
            case REVERSING_INTAKE_TO_EJECT:
                if (sampleIntakeTimer.milliseconds() >= INTAKE_PARAMS.EJECT_TIME_MS) {
                    setCurrentState(SampleIntakeStates.INTAKE_OFF);
                }
                break;
            case INTAKE_ON:
            case INTAKE_OFF:
            case INTAKE_REVERSE:
                //do nothing
                break;
        }
    }

    public void transferSampleToBucket() {
        sampleIntakeTimer.reset();
        setCurrentState(SampleIntakeStates.REVERSING_INTAKE_TO_TRANSFER);
    }

    public void ejectBadSample() {
        sampleIntakeTimer.reset();
        setCurrentState(SampleIntakeStates.REVERSING_INTAKE_TO_EJECT);
    }

    // Set servo power, ensuring it's within limits
    private void setPower(double power) {
        currentPower = Range.clip(power, -INTAKE_PARAMS.MAX_POWER, INTAKE_PARAMS.MAX_POWER);  // Clip power to safe range
        sampleIntakeLeft.setPower(currentPower);  // Apply the clipped power
        sampleIntakeRight.setPower(-currentPower);  // Apply the clipped power
    }

    // Set the current intake state and update power
    public void setCurrentState(SampleIntakeStates state) {
        currentSampleIntakeState = state;
        setPower(state.power);
    }

    // Getters for telemetry use or other purposes
    public SampleIntakeStates getCurrentState() {
        return currentSampleIntakeState;
    }

    // Update intake parameters dynamically (called in periodic)
    private void updateParameters() {
        // Update the power for each state dynamically from dashboard changes
        SampleIntakeStates.INTAKE_ON.updateIntakePower(INTAKE_PARAMS.INTAKE_ON_POWER);
        SampleIntakeStates.INTAKE_REVERSE.updateIntakePower(INTAKE_PARAMS.INTAKE_REVERSE_POWER);
        SampleIntakeStates.INTAKE_OFF.updateIntakePower(INTAKE_PARAMS.INTAKE_OFF_POWER);
    }

    public SampleDetector getSampleDetector() {
        return sampleDetector;
    }

    // Telemetry display for the dashboard
    @SuppressLint("DefaultLocale")
    public void updateDashboardTelemetry() {
        String intakeSummary = String.format(
                "State: %s | Power: %.2f",
                currentSampleIntakeState,
                currentPower
        );
        MatchConfig.telemetryPacket.put("Sample Intake", intakeSummary);

        if (sampleDetector != null) {
            String telemetrySummary = String.format(
                    "State: %s | Color: %s | Proximity: %.2f mm",
                    sampleDetector.getDetectionState(),
                    sampleDetector.getConsensusColor(),
                    sampleDetector.getConsensusProximity()
            );
            MatchConfig.telemetryPacket.put("SampleDetector", telemetrySummary);
        } else {
            MatchConfig.telemetryPacket.put("SampleDetector", "No Sample Color Sensor");
        }
    }

    // Improved basic telemetry display for the driver station
    public void displayBasicTelemetry(Telemetry telemetry) {
        String intakeState = (currentSampleIntakeState != null) ? currentSampleIntakeState.toString() : "Unknown";
        String colorStatus = (colorSensor != null) ? sampleDetector.getConsensusColor().toString() : "No Color Sensor";
        String detectionState = (sampleDetector != null) ? sampleDetector.getDetectionState().toString() : "N/A";
        double proximity = (sampleDetector != null) ? sampleDetector.getConsensusProximity() : -1;

        telemetry.addLine(String.format(
                "Sample: %s | %s | %s | %.2f mm",
                intakeState,
                colorStatus,
                detectionState,
                proximity
        ));
    }

    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addData("Sample Intake State", currentSampleIntakeState.toString());
        telemetry.addData("Sample Intake Power", currentPower);
        telemetry.addData("Time in Current State (ms)", sampleIntakeTimer.milliseconds());
        telemetry.addLine();
        telemetry.addData("Sample Detection State", sampleDetector.getDetectionState().toString());
        telemetry.addData("Consensus Color", sampleDetector.getConsensusColor().toString());
        telemetry.addData("Proximity (mm)", sampleDetector.getConsensusProximity());
        telemetry.addData("Proximity History", sampleDetector.getProximityHistory().toString());
        telemetry.addData("Raw Detected Color", sampleDetector.getRawDetectedColor().toString());
    }

}
