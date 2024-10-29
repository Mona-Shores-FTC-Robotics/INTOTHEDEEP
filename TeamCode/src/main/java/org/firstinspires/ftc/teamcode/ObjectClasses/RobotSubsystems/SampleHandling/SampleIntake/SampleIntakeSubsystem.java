package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import static com.example.sharedconstants.FieldConstants.SampleColor;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Config
public class SampleIntakeSubsystem extends SubsystemBase {

    public static class IntakeParams {
        public double INTAKE_ON_POWER = -0.8;
        public double INTAKE_REVERSE_POWER = 0.8;
        public double INTAKE_OFF_POWER = 0.0;
        public double MAX_POWER = 1.0;  // Max allowable power for intake servo
    }

    public static IntakeParams INTAKE_PARAMS = new IntakeParams();

    public enum SampleIntakeStates {
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

    private final ColorSensor colorSensor;  // Nullable color sensor
    private SampleIntakeStates currentState;
    private double currentPower;

    // Constructor with color sensor
    public SampleIntakeSubsystem(final HardwareMap hMap, final String intakeServoL, final String intakeServoR, final String colorSensorName) {
        sampleIntakeLeft = hMap.get(CRServo.class, intakeServoL);
        sampleIntakeRight = hMap.get(CRServo.class, intakeServoR);

        if (colorSensorName != null && !colorSensorName.isEmpty()) {
            colorSensor = hMap.get(ColorSensor.class, colorSensorName);
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

    // Method to read color from the sensor
    public SampleColor detectSampleColor() {
        if (colorSensor != null) {
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Use the enum to return the color
            if (red > blue && red > green) {
                return SampleColor.RED;
            } else if (blue > red && blue > green) {
                return SampleColor.BLUE;
            } else if (green > red && green > blue) {
                return SampleColor.YELLOW;
            } else {
                return SampleColor.UNKNOWN;
            }
        } else {
            // If no color sensor, return UNKNOWN
            return SampleColor.UNKNOWN;
        }
    }

    // Update intake parameters dynamically (called in periodic)
    private void updateParameters() {
        // Update the power for each state dynamically from dashboard changes
        SampleIntakeStates.INTAKE_ON.updateIntakePower(INTAKE_PARAMS.INTAKE_ON_POWER);
        SampleIntakeStates.INTAKE_REVERSE.updateIntakePower(INTAKE_PARAMS.INTAKE_REVERSE_POWER);
        SampleIntakeStates.INTAKE_OFF.updateIntakePower(INTAKE_PARAMS.INTAKE_OFF_POWER);
    }

    @Override
    public void periodic() {
        // Detect the color of the game piece in every loop
        if (colorSensor!=null) {
            SampleColor detectedSampleColor = detectSampleColor();
            handleSamplePickup(detectedSampleColor);
        }
        updateParameters();
        updateDashboardTelemetry();
    }

    // Telemetry display for the dashboard
    public void updateDashboardTelemetry() {
        MatchConfig.telemetryPacket.put("Sample Intake/State", currentState.toString());
        MatchConfig.telemetryPacket.put("Sample Intake/Power", currentPower);

        // Add color sensor reading to the telemetry, if sensor is present
        if (colorSensor != null) {
            FieldConstants.SampleColor detectedColor = detectSampleColor();
            MatchConfig.telemetryPacket.put("Sample Intake/Detected Color", detectedColor.toString());
        } else {
            MatchConfig.telemetryPacket.put("Sample Intake/Detected Color", "No Sensor");
        }
    }

    // Basic telemetry display with context for the driver station
    public void displayBasicTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Sample Intake Status", String.format("State: %s", currentState));
        if (colorSensor != null) {
            telemetry.addData("Detected Color", detectSampleColor().toString());
        } else {
            telemetry.addData("Detected Color", "No Sensor");
        }
    }

    // Getters for telemetry use or other purposes
    public SampleIntakeStates getCurrentState() {
        return currentState;
    }

    public void handleSamplePickup(SampleColor sampleColor) {
        if ((sampleColor == SampleColor.RED && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED) ||
                (sampleColor == SampleColor.BLUE && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) ||
                sampleColor == SampleColor.YELLOW) {

            // Good piece: Retract actuator and process the sample
            Robot.getInstance().getSampleHandlingStateMachine().onGoodSampleDetected();

        } else if ((sampleColor == SampleColor.RED && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) ||
                (sampleColor == SampleColor.BLUE && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED)) {

            // Bad piece: Expel the sample and reset the actuator to Mid
            Robot.getInstance().getSampleHandlingStateMachine().onBadSampleDetected();

        } else if (sampleColor == SampleColor.UNKNOWN) {
            System.out.println("Unknown color detected, no action taken.");
        }
    }
}
