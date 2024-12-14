package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.GamePieceDetector.DetectionState;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ConfigurableParameters;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleDetector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleProcessingStateMachine;

@Config
public class SampleIntakeSubsystem extends SubsystemBase {

    public static class SampleIntakeParams extends ConfigurableParameters {

        public double INTAKE_ON_POWER;
        public double INTAKE_OFF_POWER;
        public double MAX_POWER;  // Max allowable power for intake servo

        // Set a minimum proximity threshold to consider an object as "near"
        public double PROXIMITY_THRESHOLD;
        public int COLOR_HISTORY_SIZE;
        public int PROXIMITY_HISTORY_SIZE;
        public double TRANSFER_TIME_MS;
        public double EJECT_TIME_MS;

        public double EJECT_POWER_REVERSE;
        public double TRANSFER_POWER_REVERSE;

        @Override
        public void loadDefaultsForRobotType(Robot.RobotType robotType) {
            if (haveRobotSpecificParametersBeenLoaded()) return; // Skip reloading if already customized

            switch (robotType) {
                case INTO_THE_DEEP_19429:
                    INTAKE_ON_POWER = -0.8;
                    INTAKE_OFF_POWER = 0.0;
                    MAX_POWER = 1.0;
                    PROXIMITY_THRESHOLD = 40.0;
                    COLOR_HISTORY_SIZE = 2;
                    PROXIMITY_HISTORY_SIZE = 3;
                    TRANSFER_TIME_MS = 1200;
                    EJECT_TIME_MS = 800.0;
                    EJECT_POWER_REVERSE = 1.0;
                    TRANSFER_POWER_REVERSE = .38;
                    break;

                case INTO_THE_DEEP_20245:
                    INTAKE_ON_POWER = -0.8;
                    INTAKE_OFF_POWER = 0;
                    MAX_POWER = 1.0;
                    PROXIMITY_THRESHOLD = 40;
                    COLOR_HISTORY_SIZE = 2;
                    PROXIMITY_HISTORY_SIZE = 3;
                    TRANSFER_TIME_MS = 1200;
                    EJECT_TIME_MS = 801.0;
                    EJECT_POWER_REVERSE = 1.0;
                    TRANSFER_POWER_REVERSE = .38;
                    break;

                default:
                    throw new IllegalArgumentException("Unknown robot type: " + robotType);
            }
            markRobotSpecificParametersLoaded(); // Flag parameters as customized
        }
    }

    public static SampleIntakeParams SAMPLE_INTAKE_PARAMS = new SampleIntakeParams();

    public enum SampleIntakeStates {
        REVERSING_INTAKE_TO_EJECT,
        REVERSING_INTAKE_TO_TRANSFER,
        INTAKE_ON,
        INTAKE_REVERSE,
        INTAKE_OFF;

        public double getIntakePower() {
            switch (this) {
                case INTAKE_REVERSE:
                    return SAMPLE_INTAKE_PARAMS.EJECT_POWER_REVERSE;
                case INTAKE_ON:
                    return SAMPLE_INTAKE_PARAMS.INTAKE_ON_POWER;
                case INTAKE_OFF:
                    return SAMPLE_INTAKE_PARAMS.INTAKE_OFF_POWER;
                default:
                    throw new IllegalStateException("Power not defined for state: " + this);
            }
        }
    }
    private CRServo sampleIntakeSolo;  // Continuous rotation servo

    private CRServo sampleIntakeLeft;  // Continuous rotation servo
    private CRServo sampleIntakeRight;  // Continuous rotation servo

    private final RevColorSensorV3 colorSensor;  // Nullable color sensor
    private SampleIntakeStates currentSampleIntakeState;
    private double currentPower;

    private final SampleDetector sampleDetector;
    private SampleProcessingStateMachine sampleProcessingStateMachine;

    ElapsedTime sampleIntakeTimer = new ElapsedTime();

    private boolean automaticPickup=true;

    public enum SampleIntakeDetectState {
        DETECTING,
        DETECTED_GOOD_SAMPLE,
        DETECTED_BAD_SAMPLE,
    }

    private SampleIntakeDetectState currentIntakeDetectionState;

    // Constructor with color sensor
    public SampleIntakeSubsystem(final HardwareMap hMap, Robot.RobotType robotType,  final String intakeServoL, final String intakeServoR, final String colorSensorName) {
        SAMPLE_INTAKE_PARAMS.loadDefaultsForRobotType(robotType);

        if (intakeServoR!=null) {
            sampleIntakeLeft = hMap.get(CRServo.class, intakeServoL);
            sampleIntakeRight = hMap.get(CRServo.class, intakeServoR);
        } else
        {
            sampleIntakeSolo = hMap.get(CRServo.class, intakeServoL);
        }

        colorSensor = colorSensorName != null ? hMap.get(RevColorSensorV3.class, colorSensorName) : null;

        if (colorSensor != null) {
            sampleDetector = new SampleDetector(colorSensor, SAMPLE_INTAKE_PARAMS.PROXIMITY_THRESHOLD, SAMPLE_INTAKE_PARAMS.PROXIMITY_HISTORY_SIZE, SAMPLE_INTAKE_PARAMS.COLOR_HISTORY_SIZE);
        } else {
            sampleDetector = null; // no detector if sensor is unavailable;
        }
    }


    // Overloaded constructor without color sensor
    public SampleIntakeSubsystem(final HardwareMap hMap, Robot.RobotType robotType, final String intakeServo, final String colorSensorName) {
        this(hMap, robotType, intakeServo, null, colorSensorName);  // Calls the main constructor with no color sensor
    }

    // Initialize intake servo
    public void init() {
        setCurrentState(SampleIntakeStates.INTAKE_OFF);  // Set default state to off
        currentPower = SAMPLE_INTAKE_PARAMS.INTAKE_OFF_POWER;  // Cache initial power
        sampleProcessingStateMachine = Robot.getInstance().getSampleProcessingStateMachine();
        currentIntakeDetectionState= SampleIntakeDetectState.DETECTING;
    }

    @Override
    public void periodic() {
        DetectionState detectionState = sampleDetector.updateDetection();
        switch (currentIntakeDetectionState) {
            case DETECTING:
                if (!isColorSensorConnected()) {
                    //record that the specimen detector is disconnected in the log
//                    FlightRecorder.write("SAMPLE_DETECTOR" , new GamePieceDetectorMessage(SampleDetector.DetectionState.SENSOR_DISCONNECTED,-1 , FieldConstants.SampleColor.UNKNOWN));
                    break;
                }
                if (detectionState == DetectionState.JUST_DETECTED && automaticPickup){
                    if (sampleDetector.isGoodSample()) currentIntakeDetectionState = SampleIntakeDetectState.DETECTED_GOOD_SAMPLE;
                    else if (sampleDetector.isBadSample()) currentIntakeDetectionState = SampleIntakeDetectState.DETECTED_BAD_SAMPLE;
                }
//                FlightRecorder.write("SAMPLE_DETECTOR" , new GamePieceDetectorMessage(sampleDetector.getDetectionState() , sampleDetector.getConsensusProximity() , sampleDetector.getConsensusColor()));
                break;
            case DETECTED_BAD_SAMPLE:
            case DETECTED_GOOD_SAMPLE:
                if (sampleProcessingStateMachine.getCurrentSampleDetectionState() == SampleProcessingStateMachine.SampleDetectionStates.WAITING_FOR_SAMPLE_DETECTION) {
                    sampleDetector.clearDetectionState();
                    currentIntakeDetectionState = SampleIntakeDetectState.DETECTING;
                }
                break;
        }
        updateIntakeStateMachine();
        sampleProcessingStateMachine.updateSampleProcessingState();
        updateDashboardTelemetry();
    }

    private void updateIntakeStateMachine() {
        switch (currentSampleIntakeState) {
            case REVERSING_INTAKE_TO_TRANSFER:
                if (sampleIntakeTimer.milliseconds() >= SAMPLE_INTAKE_PARAMS.TRANSFER_TIME_MS) {
                    setCurrentState(SampleIntakeStates.INTAKE_OFF);
                }
                break;
            case REVERSING_INTAKE_TO_EJECT:
                if (sampleIntakeTimer.milliseconds() >= SAMPLE_INTAKE_PARAMS.EJECT_TIME_MS) {
                    setCurrentState(SampleIntakeStates.INTAKE_OFF);
                }
                break;
            case INTAKE_ON:
            case INTAKE_OFF:
            case INTAKE_REVERSE:
                //do nothing
                break;
        }
//        FlightRecorder.write("SAMPLE_INTAKE_STATE" , new SampleIntakeMessage(currentSampleIntakeState , currentPower));
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
        currentPower = Range.clip(power, - SAMPLE_INTAKE_PARAMS.MAX_POWER, SAMPLE_INTAKE_PARAMS.MAX_POWER);  // Clip power to safe range
        sampleIntakeLeft.setPower(currentPower);  // Apply the clipped power
        sampleIntakeRight.setPower(-currentPower);  // Apply the clipped power
    }

    private void setSoloPower(double power) {
        currentPower = Range.clip(power, - SAMPLE_INTAKE_PARAMS.MAX_POWER, SAMPLE_INTAKE_PARAMS.MAX_POWER);  // Clip power to safe range
        sampleIntakeSolo.setPower(currentPower);  // Apply the clipped power
    }

    // Set servo power, ensuring it's within limits
    private void setPower(double powerLeft, double powerRight) {
        sampleIntakeLeft.setPower(powerLeft);  // Apply the clipped power
        sampleIntakeRight.setPower(-powerRight);  // Apply the clipped power
    }

    // Set the current intake state and update power
    public void setCurrentState(SampleIntakeStates state) {
        currentSampleIntakeState = state;
        if (state == SampleIntakeStates.REVERSING_INTAKE_TO_TRANSFER) {
            if (sampleIntakeRight == null) {
                setSoloPower(SAMPLE_INTAKE_PARAMS.TRANSFER_POWER_REVERSE);
            } else {
                setPower(SAMPLE_INTAKE_PARAMS.TRANSFER_POWER_REVERSE, SAMPLE_INTAKE_PARAMS.TRANSFER_POWER_REVERSE);
            }
        } else if (state == SampleIntakeStates.REVERSING_INTAKE_TO_EJECT) {
            if (sampleIntakeRight == null) {
                setSoloPower(SAMPLE_INTAKE_PARAMS.EJECT_POWER_REVERSE);
            } else {
                setPower(SAMPLE_INTAKE_PARAMS.EJECT_POWER_REVERSE, SAMPLE_INTAKE_PARAMS.EJECT_POWER_REVERSE);
            }
        } else {

            if (sampleIntakeRight == null) {
                setSoloPower(state.getIntakePower());
            } else {
                setPower(state.getIntakePower());
            }
        }
    }


    // Set the current intake state and update power
    public void turnOffIntake() {
        SampleIntakeStates intakeOffState = SampleIntakeStates.INTAKE_OFF;
        if (sampleIntakeRight == null) {
            setSoloPower(intakeOffState.getIntakePower());
        } else {
            setPower(intakeOffState.getIntakePower());
        }
    }

    public void turnOnIntake() {
        Robot.getInstance().getLightingSubsystem().setLightBlack();
        SampleIntakeStates intakeOnState = SampleIntakeStates.INTAKE_ON;
        if (sampleIntakeRight == null) {
            setSoloPower(intakeOnState.getIntakePower());
        } else {
            setPower(intakeOnState.getIntakePower());
        }
    }

    public void reverseIntake() {
        SampleIntakeStates intakeReverseState = SampleIntakeStates.INTAKE_REVERSE;
        if (sampleIntakeRight == null) {
            setSoloPower(intakeReverseState.getIntakePower());
        } else {
            setPower(intakeReverseState.getIntakePower(), intakeReverseState.getIntakePower());
        }
    }

    // Getters for telemetry use or other purposes
    public SampleIntakeStates getCurrentState() {
        return currentSampleIntakeState;
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
    @SuppressLint("DefaultLocale")
    public void displayBasicTelemetry(Telemetry telemetry) {
        String intakeState = (currentSampleIntakeState != null) ? currentSampleIntakeState.toString() : "Unknown";
        String colorStatus = (colorSensor != null) ? sampleDetector.getConsensusColor().toString() : "No Color Sensor";
        telemetry.addLine(String.format(
                "%s | %s ",
                intakeState,
                colorStatus
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

    private boolean isColorSensorConnected() {
        try {
            if (colorSensor != null && sampleDetector != null) {
                String connectionInfo = colorSensor.getConnectionInfo();
                MatchConfig.telemetryPacket.put("Color Sensor Connection Info", connectionInfo);

                // Validate connection info
                return connectionInfo != null && connectionInfo.toLowerCase().contains("bus");
            }
        } catch (Exception e) {
            // If an exception occurs, the sensor might be disconnected or unresponsive
            MatchConfig.telemetryPacket.put("Color Sensor Error", "Disconnected or Inaccessible");
        }
        return false; // Return false if null or an exception is caught
    }

    public SampleIntakeDetectState getCurrentIntakeDetectState() {
        return currentIntakeDetectionState;
    }

    public void setAutomaticPickupTrue() {
        this.automaticPickup = true;
    }

    public void setAutomaticPickupFalse() {
        this.automaticPickup = false;
    }

    public boolean haveSample() {
        return sampleDetector.haveSample();
    }

}
