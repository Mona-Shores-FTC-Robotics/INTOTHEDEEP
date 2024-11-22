package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.ConfigurableParameters;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenDetector;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.GamePieceDetector.DetectionState;

@Config
public class SpecimenIntakeSubsystem extends SubsystemBase {

    public static class SpecimenIntakeParams extends ConfigurableParameters {
        public double INTAKE_ON_POWER = Double.NaN;
        public double INTAKE_REVERSE_POWER = Double.NaN;
        public double INTAKE_OFF_POWER = Double.NaN;
        public double MAX_POWER = Double.NaN;  // Max allowable power for intake servo
        public double PROXIMITY_THRESHOLD_IN_MM = Double.NaN;
        public int HISTORY_SIZE = -1;

        @Override
        public void loadDefaultsForRobotType(Robot.RobotType robotType) {
            if (haveRobotSpecificParametersBeenLoaded()) return;
            switch (robotType) {
                case INTO_THE_DEEP_19429:
                    SPECIMEN_INTAKE_PARAMS.INTAKE_ON_POWER = -1.0;
                    SPECIMEN_INTAKE_PARAMS.INTAKE_REVERSE_POWER = 1.0;
                    SPECIMEN_INTAKE_PARAMS.INTAKE_OFF_POWER = 0.0;
                    SPECIMEN_INTAKE_PARAMS.MAX_POWER = 1.0;
                    SPECIMEN_INTAKE_PARAMS.PROXIMITY_THRESHOLD_IN_MM = 30;
                    SPECIMEN_INTAKE_PARAMS.HISTORY_SIZE = 5;
                    break;

                case INTO_THE_DEEP_20245:
                    SPECIMEN_INTAKE_PARAMS.INTAKE_ON_POWER = -0.8;
                    SPECIMEN_INTAKE_PARAMS.INTAKE_REVERSE_POWER = 0.8;
                    SPECIMEN_INTAKE_PARAMS.INTAKE_OFF_POWER = 0.0;
                    SPECIMEN_INTAKE_PARAMS.MAX_POWER = 0.8;
                    SPECIMEN_INTAKE_PARAMS.PROXIMITY_THRESHOLD_IN_MM = 25;
                    SPECIMEN_INTAKE_PARAMS.HISTORY_SIZE = 4;
                    break;

                default:
                    throw new IllegalArgumentException("Unknown robot type: " + robotType);
            }
            markRobotSpecificParametersLoaded();
        }
    }

    public static SpecimenIntakeParams SPECIMEN_INTAKE_PARAMS = new SpecimenIntakeParams();

    public enum SpecimenIntakeStates {
        INTAKE_ON,
        INTAKE_REVERSE,
        INTAKE_OFF;

        public double getIntakePower() {
            switch (this) {
                case INTAKE_ON:
                    return SPECIMEN_INTAKE_PARAMS.INTAKE_ON_POWER;
                case INTAKE_REVERSE:
                    return SPECIMEN_INTAKE_PARAMS.INTAKE_REVERSE_POWER;
                case INTAKE_OFF:
                    return SPECIMEN_INTAKE_PARAMS.INTAKE_OFF_POWER;
                default:
                    throw new IllegalStateException("Angle not defined for state: " + this);
            }
        }

    }

    private final CRServo specimenIntake;  // Continuous rotation servo
    private SpecimenIntakeStates currentState;
    private double currentPower;
    private Boolean preloadModeActive;

    private final SpecimenDetector specimenDetector;
    // Constructor with color sensor
    public SpecimenIntakeSubsystem(final HardwareMap hMap,final Robot.RobotType robotType, final String intakeServo, final String colorSensorName) {
        SPECIMEN_INTAKE_PARAMS.loadDefaultsForRobotType(robotType);

        specimenIntake = hMap.get(CRServo.class, intakeServo);
        // Nullable color sensor
        RevColorSensorV3 colorSensor = colorSensorName != null ? hMap.get(RevColorSensorV3.class , colorSensorName) : null;
        if (colorSensor != null) {
            specimenDetector = new SpecimenDetector(colorSensor , SPECIMEN_INTAKE_PARAMS.PROXIMITY_THRESHOLD_IN_MM, SPECIMEN_INTAKE_PARAMS.HISTORY_SIZE);
        } else {
            specimenDetector = null; // no detector if sensor is unavailable;
        }
    }

    // Overloaded constructor without color sensor
    public SpecimenIntakeSubsystem(final HardwareMap hMap, Robot.RobotType robotType, final String intakeServo ) {
        this(hMap, robotType, intakeServo, null);  // Calls the main constructor with no color sensor
    }

    // Initialize intake servo
    public void init() {
        preloadModeActive = Robot.getInstance().getOpModeType() == Robot.OpModeType.AUTO;
        setCurrentState(SpecimenIntakeStates.INTAKE_OFF);  // Set default state to off
        currentPower = SPECIMEN_INTAKE_PARAMS.INTAKE_OFF_POWER;  // Cache initial power
    }

    @Override
    public void periodic() {
        if (specimenDetector != null)
        {
            // Update the detection state via the detector
            DetectionState specimenDetectionState = specimenDetector.updateDetection();
            switch (specimenDetectionState) {
                case JUST_DETECTED:
                    handleSpecimenPickup();  // Trigger pickup behavior if specimen was just detected
                    break;

                case STILL_DETECTED:
                    // Optionally, handle logic if specimen is still detected, if needed
                    //Todo could we check if there is a high current on the arm and release the specimen if there is?
                    break;

                case NOT_DETECTED:
                    break;
            }
        }
        updateDashboardTelemetry();
    }

    public void handleSpecimenPickup() {
        //Only handle specimen detection autonomously if its not the preload and we are in not in Auto
        if (!preloadModeActive) {
            Robot.getInstance().getSpecimenDetectionStateMachine().onSpecimenDetection();
        }
    }

    public void disablePreloadMode() {
        preloadModeActive = false;
    }

    // Set the current intake state and update power
    public void setCurrentState(SpecimenIntakeStates state) {
        currentState = state;
        setPower(state.getIntakePower());
    }

    // Set servo power, ensuring it's within limits
    private void setPower(double power) {
        currentPower = Range.clip(power, -SPECIMEN_INTAKE_PARAMS.MAX_POWER, SPECIMEN_INTAKE_PARAMS.MAX_POWER);  // Clip power to safe range
        specimenIntake.setPower(currentPower);  // Apply the clipped power
    }

    public void turnOnIntake() {
        currentState = SpecimenIntakeStates.INTAKE_ON;
        setPower(currentState.getIntakePower());
    }

    public void turnOffIntake() {
        currentState = SpecimenIntakeStates.INTAKE_OFF;
        setPower(currentState.getIntakePower());
    }

    public void reverseIntake() {
        currentState = SpecimenIntakeStates.INTAKE_REVERSE;
        setPower(currentState.getIntakePower());
    }

    public SpecimenDetector getSpecimenDetector() {
        return specimenDetector;
    }

    // Basic telemetry display with context for the driver station
    public void displayBasicTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        String intakeState = (currentState != null) ? currentState.toString() : "Unknown";
        String detectionState = (specimenDetector != null) ? specimenDetector.getDetectionState().toString() : "N/A";

        telemetry.addLine(String.format(
                "%s | %s",
                intakeState, detectionState
        ));
    }

    // Telemetry display for the dashboard
    @SuppressLint("DefaultLocale")
    public void updateDashboardTelemetry() {
        String intakeStatus = String.format(
                "State: %s | Power: %.2f | %s | %s mm",
                currentState != null ? currentState.toString() : "Unknown",
                currentPower,
                specimenDetector != null ? specimenDetector.getDetectionState().toString() : "No Detector",
                specimenDetector != null ? String.format("%.2f mm", specimenDetector.getConsensusProximity()) : "No Detector"
        );

        MatchConfig.telemetryPacket.put("Specimen Intake Status", intakeStatus);
    }


    public boolean checkForPreload() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE))
        {
            if (Robot.getInstance().getSpecimenIntakeSubsystem().specimenDetector!=null) {
                Robot.getInstance().getSpecimenIntakeSubsystem().specimenDetector.updateDetection();
            }
        }
        return specimenDetector != null && specimenDetector.haveSpecimen();
    }

    public FieldConstants.AllianceColor updateAllianceColorBasedOnPreload() {
        // Determine alliance color based on specimen detector consensus
        if (specimenDetector != null && specimenDetector.getConsensusColor() == FieldConstants.SampleColor.RED) {
            MatchConfig.finalAllianceColor = FieldConstants.AllianceColor.RED;
        } else { // (specimenDetector != null && specimenDetector.getConsensusColor() == FieldConstants.SampleColor.BLUE)
            MatchConfig.finalAllianceColor = FieldConstants.AllianceColor.BLUE;
       }
        return MatchConfig.finalAllianceColor;
    }

    public void monitorSpecimenPreload(RealRobotAdapter adapter, Boolean lockedSettingsFlag, Boolean manualOverrideFlag) {
        // Skip updates if settings are locked or manually overridden
        if (!lockedSettingsFlag && !manualOverrideFlag) {
            boolean havePreload = checkForPreload();
            if (havePreload) {
                FieldConstants.AllianceColor detectedColor = updateAllianceColorBasedOnPreload();
                adapter.setAllianceColor(detectedColor);
            }
        }
    }

    public boolean isNotReversing() {
        return currentState != SpecimenIntakeStates.INTAKE_REVERSE;
    }

}
