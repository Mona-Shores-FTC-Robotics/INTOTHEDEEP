package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.GamePieceDetector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleDetector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenDetector;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.GamePieceDetector.DetectionState;

@Config
public class SpecimenIntakeSubsystem extends SubsystemBase {


    public static class SpecimenIntakeParams {
        public double INTAKE_ON_POWER = -0.8;
        public double INTAKE_REVERSE_POWER = 0.8;
        public double INTAKE_OFF_POWER = 0.0;
        public double MAX_POWER = 1.0;  // Max allowable power for intake servo
        public double PROXIMITY_THRESHOLD_IN_MM = 30;
        public int HISTORY_SIZE = 5;
    }

    public static SpecimenIntakeParams SPECIMEN_INTAKE_PARAMS = new SpecimenIntakeParams();

    public enum SpecimenIntakeStates {
        INTAKE_ON(SPECIMEN_INTAKE_PARAMS.INTAKE_ON_POWER),
        INTAKE_REVERSE(SPECIMEN_INTAKE_PARAMS.INTAKE_REVERSE_POWER),
        INTAKE_OFF(SPECIMEN_INTAKE_PARAMS.INTAKE_OFF_POWER);
        public double power;
        SpecimenIntakeStates(double power) {
            this.power = power;
        }
        // Dynamically update the intake power if parameters change
        public void updateIntakePower(double newPower) {
            this.power = newPower;
        }
    }

    private final CRServo specimenIntake;  // Continuous rotation servo
    private final RevColorSensorV3 colorSensor;  // Nullable color sensor
    private SpecimenIntakeStates currentState;
    private double currentPower;
    private Boolean preloadModeActive;

    private final SpecimenDetector specimenDetector;

    // Constructor with color sensor
    public SpecimenIntakeSubsystem(final HardwareMap hMap, final String intakeServo, final String colorSensorName) {
        specimenIntake = hMap.get(CRServo.class, intakeServo);
        colorSensor = colorSensorName != null ? hMap.get(RevColorSensorV3.class, colorSensorName) : null;
        if (colorSensor != null) {
            specimenDetector = new SpecimenDetector(colorSensor, SPECIMEN_INTAKE_PARAMS.PROXIMITY_THRESHOLD_IN_MM, SPECIMEN_INTAKE_PARAMS.HISTORY_SIZE);
        } else {
            specimenDetector = null; // no detector if sensor is unavailable;
        }
    }

    // Overloaded constructor without color sensor
    public SpecimenIntakeSubsystem(final HardwareMap hMap, final String intakeServo ) {
        this(hMap, intakeServo, null);  // Calls the main constructor with no color sensor
    }

    // Initialize intake servo
    public void init() {
        preloadModeActive = Robot.getInstance().getOpModeType() == Robot.OpModeType.AUTO;
        setCurrentState(SpecimenIntakeStates.INTAKE_OFF);  // Set default state to off
        currentPower = SPECIMEN_INTAKE_PARAMS.INTAKE_OFF_POWER;  // Cache initial power
    }

    @Override
    public void periodic() {
        if (specimenDetector != null) {
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
        updateParameters();
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
        setPower(state.power);
    }

    // Set servo power, ensuring it's within limits
    private void setPower(double power) {
        currentPower = Range.clip(power, -SPECIMEN_INTAKE_PARAMS.MAX_POWER, SPECIMEN_INTAKE_PARAMS.MAX_POWER);  // Clip power to safe range
        specimenIntake.setPower(currentPower);  // Apply the clipped power
    }

    public void turnOnIntake() {
        currentState = SpecimenIntakeStates.INTAKE_ON;
        setPower(currentState.power);
    }

    public void turnOffIntake() {
        currentState = SpecimenIntakeStates.INTAKE_OFF;
        setPower(currentState.power);
    }

    public void reverseIntake() {
        currentState = SpecimenIntakeStates.INTAKE_REVERSE;
        setPower(currentState.power);
    }


    // Update intake parameters dynamically (called in periodic)
    private void updateParameters() {
        // Update the power for each state dynamically from dashboard changes
        SpecimenIntakeStates.INTAKE_ON.updateIntakePower(SPECIMEN_INTAKE_PARAMS.INTAKE_ON_POWER);
        SpecimenIntakeStates.INTAKE_REVERSE.updateIntakePower(SPECIMEN_INTAKE_PARAMS.INTAKE_REVERSE_POWER);
        SpecimenIntakeStates.INTAKE_OFF.updateIntakePower(SPECIMEN_INTAKE_PARAMS.INTAKE_OFF_POWER);
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
}
