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

@Config
public class SpecimenIntakeSubsystem extends SubsystemBase {

    public static class SpecimenIntakeParams {
        public double INTAKE_ON_POWER = -0.8;
        public double INTAKE_REVERSE_POWER = 0.8;
        public double INTAKE_OFF_POWER = 0.0;
        public double MAX_POWER = 1.0;  // Max allowable power for intake servo
        public int COLOR_HISTORY_SIZE = 5;
        public double PROXIMITY_THRESHOLD = .5;
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
           // handleSpecimenPickup();
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
            MatchConfig.telemetryPacket.put("Specimen Intake/Detected Color", "No Sensor");
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
            Robot.getInstance().getSpecimenDetectionStateMachine().onSpecimenDetection();
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

    // Method to read color from the sensor

    // Integrated student sample data using chatGPT
    public Boolean detectSpecimen() {
        // Use the global proximity variable and update telemetry
        proximity = colorSensor.getDistance(DistanceUnit.INCH);

        // Object detected within the range
        return proximity < INTAKE_PARAMS.PROXIMITY_THRESHOLD;
    }

    // Update intake parameters dynamically (called in periodic)
    private void updateParameters() {
        // Update the power for each state dynamically from dashboard changes
        SpecimenIntakeStates.INTAKE_ON.updateIntakePower(INTAKE_PARAMS.INTAKE_ON_POWER);
        SpecimenIntakeStates.INTAKE_REVERSE.updateIntakePower(INTAKE_PARAMS.INTAKE_REVERSE_POWER);
        SpecimenIntakeStates.INTAKE_OFF.updateIntakePower(INTAKE_PARAMS.INTAKE_OFF_POWER);
    }

    private SampleColor getRawDetectedColor() {
        // Get HSV values and raw RGB values
        float[] hsvValues = new float[3];
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        Color.RGBToHSV(red, green, blue, hsvValues);

        int hue = Math.round(hsvValues[0]);
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        // Update telemetry for debugging
        MatchConfig.telemetryPacket.put("Sample Intake/HSV/Hue", hue);
        MatchConfig.telemetryPacket.put("Sample Intake/HSV/Saturation", saturation);
        MatchConfig.telemetryPacket.put("Sample Intake/HSV/Value", value);
        MatchConfig.telemetryPacket.put("Sample Intake/color/Red", red);
        MatchConfig.telemetryPacket.put("Sample Intake/color/Green", green);
        MatchConfig.telemetryPacket.put("Sample Intake/color/Blue", blue);

        // Red detection (using updated sample data)
        if (    (hue >= 10 && hue <= 35) ||
                (red > 500 && green < 700 && blue < 600)) {
            return SampleColor.RED;
        }
        // Blue detection
        else if (   (hue >= 190 && hue <= 250) ||
                (red < 400 && green > 580 && blue > 1200)) {
            return SampleColor.BLUE;
        }
        // Yellow detection
        else if (   (hue >= 70 && hue <= 120) ||
                (red > 1000 && green > 1600 && blue < 750)) {
            return SampleColor.YELLOW;
        }
        // "Unknown" detection if no match
        else {
            return SampleColor.UNKNOWN;
        }
    }
}
