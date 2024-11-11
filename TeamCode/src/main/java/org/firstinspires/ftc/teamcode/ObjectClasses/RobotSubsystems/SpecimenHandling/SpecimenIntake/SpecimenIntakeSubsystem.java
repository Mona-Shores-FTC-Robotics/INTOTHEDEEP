package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake;

import static com.example.sharedconstants.FieldConstants.AllianceColor.RED;
import static com.example.sharedconstants.FieldConstants.SampleColor;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Lighting.LightingSubsystem;

import java.util.LinkedList;
import java.util.Queue;

@Config
public class SpecimenIntakeSubsystem extends SubsystemBase {

    public static class SpecimenIntakeParams {
        public double INTAKE_ON_POWER = -0.8;
        public double INTAKE_REVERSE_POWER = 0.8;
        public double INTAKE_OFF_POWER = 0.0;
        public double MAX_POWER = 1.0;  // Max allowable power for intake servo
        public double PROXIMITY_THRESHOLD = 61;
    }

    public static SpecimenIntakeParams INTAKE_PARAMS = new SpecimenIntakeParams();

    public enum SpecimenIntakeStates {
        INTAKE_ON(INTAKE_PARAMS.INTAKE_ON_POWER),
        INTAKE_REVERSE(INTAKE_PARAMS.INTAKE_REVERSE_POWER),
        INTAKE_OFF(INTAKE_PARAMS.INTAKE_OFF_POWER);
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
    private double proximity;
    private LightingSubsystem lightingSubsystem;
    private Boolean hasSpecimen;
    private Boolean preloadStaged;

    private static final int HISTORY_SIZE = 5;
    private Queue<Double> proximityHistory = new LinkedList<>();

    // Constructor with color sensor
    public SpecimenIntakeSubsystem(final HardwareMap hMap, final String intakeServo, final String colorSensorName) {
        specimenIntake = hMap.get(CRServo.class, intakeServo);
        if (colorSensorName != null && !colorSensorName.isEmpty()) {
            colorSensor = hMap.get(RevColorSensorV3.class, colorSensorName);
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight)colorSensor).enableLight(false);
            }
        } else {
            colorSensor = null;  // No color sensor configured
        }
    }

    // Overloaded constructor without color sensor
    public SpecimenIntakeSubsystem(final HardwareMap hMap, final String intakeServo ) {
        this(hMap, intakeServo, null);  // Calls the main constructor with no color sensor
    }

    // Initialize intake servo
    public void init() {
        hasSpecimen=detectSpecimen();
        preloadStaged=true;
        if (Robot.getInstance().getOpModeType() == Robot.OpModeType.AUTO) {
            preloadStaged = false;
        }
        setCurrentState(SpecimenIntakeStates.INTAKE_OFF);  // Set default state to off
        currentPower = INTAKE_PARAMS.INTAKE_OFF_POWER;  // Cache initial power
        lightingSubsystem = Robot.getInstance().getLightingSubsystem();
    }

    @Override
    public void periodic() {
        // Detect the color of the game piece in every loop
        if (colorSensor!=null) {
            hasSpecimen = detectSpecimen();
            handleSpecimenPickup();
        }
        updateParameters();
        updateDashboardTelemetry();
    }

    public void preloadHasBeenStaged()
    {
        preloadStaged = true;
    }

    // Telemetry display for the dashboard
    @SuppressLint("DefaultLocale")
    public void updateDashboardTelemetry() {
        MatchConfig.telemetryPacket.put("Specimen Intake/State", currentState.toString());
        MatchConfig.telemetryPacket.put("Specimen Intake/Power", currentPower);

        // Add color sensor reading to the telemetry, if sensor is present
        if (colorSensor != null) {
            MatchConfig.telemetryPacket.put("Specimen Intake/Proximity", String.format("%.2f", proximity));
        } else {
            MatchConfig.telemetryPacket.put("Specimen Intake/", "No Sensor");
        }
    }

    // Basic telemetry display with context for the driver station
    public void displayBasicTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        // Display the current state and detected color on the same line
        String intakeState = (currentState != null) ? currentState.toString() : "Unknown";
        String colorStatus = (colorSensor != null) ? detectSpecimen ().toString() : "No Color Sensor";
        telemetry.addLine(String.format("%s | Color: %s", intakeState, colorStatus));
    }

    public void handleSpecimenPickup() {
        //only do something about detection if the preload has been staged (don't want to move the specimen arm before we move away from wall)
        if (hasSpecimen && preloadStaged) {
            if (MatchConfig.finalAllianceColor == RED) {
                lightingSubsystem.setBothLightsRed ();
            } else lightingSubsystem.setBothLightsBlue ();
            setCurrentState(SpecimenIntakeStates.INTAKE_OFF);
//            Robot.getInstance().getSpecimenDetectionStateMachine().onSpecimenDetection();
        }
    }

    // Set the current intake state and update power
    public void setCurrentState(SpecimenIntakeStates state) {
        currentState = state;
        setPower(state.power);
    }

    // Set servo power, ensuring it's within limits
    private void setPower(double power) {
        currentPower = Range.clip(power, -INTAKE_PARAMS.MAX_POWER, INTAKE_PARAMS.MAX_POWER);  // Clip power to safe range
        specimenIntake.setPower(currentPower);  // Apply the clipped power
    }

    public Boolean detectSpecimen() {
        // Get the current proximity reading
        proximity = colorSensor.getDistance(DistanceUnit.MM);

        // Add the current reading to the history
        if (proximityHistory.size() >= HISTORY_SIZE) {
            proximityHistory.poll();  // Remove the oldest reading if at limit
        }
        proximityHistory.add(proximity);

        // Check if all values in the history are below the threshold
        for (double p : proximityHistory) {
            if (p >= INTAKE_PARAMS.PROXIMITY_THRESHOLD) {
                return false;  // Return false if any reading is above the threshold
            }
        }

        return proximityHistory.size() == HISTORY_SIZE;  // Return true if all values are below threshold and history is full
    }

    // Update intake parameters dynamically (called in periodic)
    private void updateParameters() {
        // Update the power for each state dynamically from dashboard changes
        SpecimenIntakeStates.INTAKE_ON.updateIntakePower(INTAKE_PARAMS.INTAKE_ON_POWER);
        SpecimenIntakeStates.INTAKE_REVERSE.updateIntakePower(INTAKE_PARAMS.INTAKE_REVERSE_POWER);
        SpecimenIntakeStates.INTAKE_OFF.updateIntakePower(INTAKE_PARAMS.INTAKE_OFF_POWER);
    }
}
