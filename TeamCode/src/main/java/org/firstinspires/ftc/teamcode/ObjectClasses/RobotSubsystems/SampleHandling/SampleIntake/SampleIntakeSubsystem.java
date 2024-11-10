package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import static com.example.sharedconstants.FieldConstants.SampleColor;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.example.sharedconstants.FieldConstants;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Lighting.LightingSubsystem;

import java.util.ArrayDeque;
import java.util.Deque;

@Config
public class SampleIntakeSubsystem extends SubsystemBase {

    public static class IntakeParams {
        public double INTAKE_ON_POWER = 0.8;
        public double INTAKE_REVERSE_POWER = -0.8;
        public double INTAKE_OFF_POWER = 0.0;
        public double MAX_POWER = 1.0;  // Max allowable power for intake servo
        // Set a minimum proximity threshold to consider an object as "near"
        public double PROXIMITY_THRESHOLD = .8;
        public int COLOR_HISTORY_SIZE = 5;
        public double TRANSFER_TIME_MS= 400;
    }

    public static IntakeParams INTAKE_PARAMS = new IntakeParams();

    public enum SampleIntakeStates {
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
    private SampleIntakeStates currentState;
    private LightingSubsystem lightingSubsystem;
    private double currentPower;
    private double proximity;

    private final Deque<SampleColor> colorHistory = new ArrayDeque<>(INTAKE_PARAMS.COLOR_HISTORY_SIZE);
    // Variables to store the latest colors
    private SampleColor lastRawColor = SampleColor.UNKNOWN;
    private SampleColor lastFilteredColor = SampleColor.UNKNOWN;
    ElapsedTime sampleIntakeTimer = new ElapsedTime();
    private boolean haveSample = false;

    // Constructor with color sensor
    public SampleIntakeSubsystem(final HardwareMap hMap, final String intakeServoL, final String intakeServoR, final String colorSensorName) {
        sampleIntakeLeft = hMap.get(CRServo.class, intakeServoL);
        sampleIntakeRight = hMap.get(CRServo.class, intakeServoR);

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
    public SampleIntakeSubsystem(final HardwareMap hMap, final String intakeServoL, final String intakeServoR) {
        this(hMap, intakeServoL, intakeServoR, null);  // Calls the main constructor with no color sensor
    }

    // Initialize intake servo
    public void init() {
        setCurrentState(SampleIntakeStates.INTAKE_OFF);  // Set default state to off
        currentPower = INTAKE_PARAMS.INTAKE_OFF_POWER;  // Cache initial power
        lightingSubsystem = Robot.getInstance().getLightingSubsystem();
        haveSample= false;
    }

    @Override
    public void periodic() {
        // Detect the color of the game piece in every loop
        if (haveSample){
            Robot.getInstance().getSampleDetectionStateMachine().updateSameDetectionState();
            if (currentState==SampleIntakeStates.REVERSING_INTAKE_TO_TRANSFER && sampleIntakeTimer.milliseconds()>=INTAKE_PARAMS.TRANSFER_TIME_MS)
            {
                setCurrentState(SampleIntakeStates.INTAKE_OFF);
                haveSample=false;
                Robot.getInstance().getSampleDetectionStateMachine().updateSameDetectionState();
            }
        } else if (colorSensor!=null) {
            lastFilteredColor = detectSampleColor();
            handleSamplePickup(lastFilteredColor);
        }
        updateParameters();
        updateDashboardTelemetry();
    }
    // Set the current intake state and update power
    public void setCurrentState(SampleIntakeStates state) {
        currentState = state;
        setPower(state.power);
    }
    // Set servo power, ensuring it's within limits
    private void setPower(double power) {
        currentPower = Range.clip(power, -INTAKE_PARAMS.MAX_POWER, INTAKE_PARAMS.MAX_POWER);  // Clip power to safe range
        sampleIntakeLeft.setPower(currentPower);  // Apply the clipped power
        sampleIntakeRight.setPower(-currentPower);  // Apply the clipped power
    }

    public void transferSampleToBucket() {
        sampleIntakeTimer.reset();
        setCurrentState(SampleIntakeStates.REVERSING_INTAKE_TO_TRANSFER);
    }

    // Integrated student sample data using chatGPT
    public SampleColor detectSampleColor() {
        // Use the global proximity variable and update telemetry
        proximity = colorSensor.getDistance(DistanceUnit.INCH);

        if (proximity < INTAKE_PARAMS.PROXIMITY_THRESHOLD) {  // Object detected within the range
            SampleColor rawColor = getRawDetectedColor();
            lastRawColor = rawColor; // Store raw color
            addColorToHistory(rawColor);
            lastFilteredColor = getConsensusColor(); // Store filtered color
            return lastFilteredColor;
        } else {
            colorHistory.clear();
            lastRawColor = SampleColor.NO_SAMPLE;
            lastFilteredColor = SampleColor.NO_SAMPLE;
            return SampleColor.NO_SAMPLE;
        }
    }

    private SampleColor getRawDetectedColor() {
        // Get HSV values and raw RGB values
        float[] hsvValues = new float[3];
        colorSensor.argb();
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);

        int hue = Math.round(hsvValues[0]);
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

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

    private void addColorToHistory(SampleColor color) {
        if (colorHistory.size() == INTAKE_PARAMS.COLOR_HISTORY_SIZE) {
            colorHistory.removeFirst();
        }
        colorHistory.addLast(color);
    }

    private SampleColor getConsensusColor() {
        if (colorHistory.size() < INTAKE_PARAMS.COLOR_HISTORY_SIZE) {
            return SampleColor.UNKNOWN;
        }
        SampleColor firstColor = colorHistory.peekFirst();
        for (SampleColor color : colorHistory) {
            if (color != firstColor) {
                return SampleColor.UNKNOWN;
            }
        }
        return firstColor;
    }

    // Update intake parameters dynamically (called in periodic)
    private void updateParameters() {
        // Update the power for each state dynamically from dashboard changes
        SampleIntakeStates.INTAKE_ON.updateIntakePower(INTAKE_PARAMS.INTAKE_ON_POWER);
        SampleIntakeStates.INTAKE_REVERSE.updateIntakePower(INTAKE_PARAMS.INTAKE_REVERSE_POWER);
        SampleIntakeStates.INTAKE_OFF.updateIntakePower(INTAKE_PARAMS.INTAKE_OFF_POWER);
    }

    // Telemetry display for the dashboard
    @SuppressLint("DefaultLocale")
    public void updateDashboardTelemetry() {
        MatchConfig.telemetryPacket.put("Sample Intake/State", currentState.toString());
        MatchConfig.telemetryPacket.put("Sample Intake/Power", currentPower);

        if (colorSensor != null) {
            MatchConfig.telemetryPacket.put("Sample Intake/Raw Color", lastRawColor.toString());
            MatchConfig.telemetryPacket.put("Sample Intake/Filtered Color", lastFilteredColor.toString());
            MatchConfig.telemetryPacket.put("Sample Intake/Proximity", String.format("%.2f", proximity));
        } else {
            MatchConfig.telemetryPacket.put("Sample Intake/Detected Color", "No Sensor");
        }
    }

    // Improved basic telemetry display for the driver station
    public void displayBasicTelemetry(Telemetry telemetry) {
        // Display the current state and detected color on the same line
        String intakeState = (currentState != null) ? currentState.toString() : "Unknown";
        String colorStatus = (colorSensor != null) ? detectSampleColor().toString() : "No Color Sensor";
        telemetry.addLine(String.format("%s | Color: %s", intakeState, colorStatus));
    }

    public void displayVerboseTelemetry(Telemetry telemetry) {
        telemetry.addData("Sample Intake Status", String.format("State: %s", currentState));
        if (colorSensor != null) {
            telemetry.addData("Detected Color", detectSampleColor().toString());
            telemetry.addData("Sample Intake/Proximity", proximity);
        } else {
            telemetry.addData("Detected Color", "No Sensor");
        }
    }


    // Getters for telemetry use or other purposes
    public SampleIntakeStates getCurrentState() {
        return currentState;
    }

    public void handleSamplePickup(SampleColor sampleColor) {
        if (    (sampleColor == SampleColor.RED && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED) ||
                (sampleColor == SampleColor.BLUE && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) ||
                sampleColor == SampleColor.YELLOW) {

            // Good piece: Retract actuator and process the sample
            if (sampleColor == SampleColor.YELLOW) {
                lightingSubsystem.setBothLightsYellow();
            } else if (sampleColor == SampleColor.RED) {
                lightingSubsystem.setBothLightsRed();
            }else  {
                lightingSubsystem.setBothLightsBlue();
            }

            if (Robot.getInstance().getSampleDetectionStateMachine()!=null)
            {
                //If we are in Teleop then use the onGoodSampleDetectedCommand
//                if (Robot.getInstance().getOpModeType() == Robot.OpModeType.TELEOP) {
//                    Robot.getInstance().getSampleHandlingStateMachine().onGoodSampleDetectedCommand();
//                } else {
                    haveSample = true;
                    Robot.getInstance().getSampleDetectionStateMachine().setGoodSampleDetectedState();
                    Robot.getInstance().getSampleDetectionStateMachine().updateSameDetectionState();
//                }
            }

        } else if ((sampleColor == SampleColor.RED && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) ||
                (sampleColor == SampleColor.BLUE && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED)) {

            // Bad piece: Expel the sample and reset the actuator to Mid
            Robot.getInstance().getSampleHandlingStateMachine().onBadSampleDetectedCommand();

        } else if (sampleColor == SampleColor.UNKNOWN) {
            System.out.println("Unknown color detected, no action taken.");
        }
    }
}
