package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake;

import static com.example.sharedconstants.FieldConstants.SampleColor;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public class SpecimenIntakeSubsystem extends SubsystemBase {

    public static class SpecimenIntakeParams {
        public double INTAKE_ON_POWER = -0.8;
        public double INTAKE_REVERSE_POWER = 0.8;
        public double INTAKE_OFF_POWER = 0.0;
        public double MAX_POWER = 1.0;  // Max allowable power for intake servo
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

    private final ColorSensor colorSensor;  // Nullable color sensor
    private SpecimenIntakeStates currentState;
    private double currentPower;

    // Constructor with color sensor
    public SpecimenIntakeSubsystem(final HardwareMap hMap, final String intakeServo, final String colorSensorName) {
        specimenIntake = hMap.get(CRServo.class, intakeServo);

        if (colorSensorName != null && !colorSensorName.isEmpty()) {
            colorSensor = hMap.get(ColorSensor.class, colorSensorName);
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
        setCurrentState(SpecimenIntakeStates.INTAKE_OFF);  // Set default state to off
        currentPower = INTAKE_PARAMS.INTAKE_OFF_POWER;  // Cache initial power
    }


    @Override
    public void periodic() {
        // Detect the color of the game piece in every loop
        if (colorSensor!=null) {
            SampleColor detectedSpecimenColor = detectSpecimenColor();
            handleSpecimenPickup(detectedSpecimenColor);
        }
        updateParameters();
        updateDashboardTelemetry();
    }

    // Telemetry display for the dashboard
    public void updateDashboardTelemetry() {
        MatchConfig.telemetryPacket.put("Specimen Intake/State", currentState.toString());
        MatchConfig.telemetryPacket.put("Specimen Intake/Power", currentPower);

        // Add color sensor reading to the telemetry, if sensor is present
        if (colorSensor != null) {
            SampleColor detectedColor = detectSpecimenColor();
            MatchConfig.telemetryPacket.put("Specimen Intake/Detected Color", detectedColor.toString());
        } else {
            MatchConfig.telemetryPacket.put("Specimen Intake/Detected Color", "No Sensor");
        }
    }

    // Basic telemetry display with context for the driver station
    public void displayBasicTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        // Display the current state and detected color on the same line
        String intakeState = (currentState != null) ? currentState.toString() : "Unknown";
        String colorStatus = (colorSensor != null) ? detectSpecimenColor().toString() : "No Color Sensor";
        telemetry.addLine(String.format("%s | Color: %s", intakeState, colorStatus));

    }

    // Getters for telemetry use or other purposes
    public SpecimenIntakeStates getCurrentState() {
        return currentState;
    }

    public void handleSpecimenPickup(SampleColor specimenColor) {
        if ((specimenColor == SampleColor.RED && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED) ||
                (specimenColor == SampleColor.BLUE && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE)){

            // On good specimen change color of lights
            // stop turning specimen intake
            // flip the specimen arm
            // Good piece: Retract actuator and process the specimen
//            Robot.getInstance().getSpecimenHandlingStateMachine().onGoodSpecimenDetected();

        } else if ((specimenColor == SampleColor.RED && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) ||
                (specimenColor == SampleColor.BLUE && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED) ||
                (specimenColor== SampleColor.YELLOW)) {

            // Bad piece: Expel the specimen
//            Robot.getInstance().getSpecimenStateMachine().onBadSpecimenDetected();

        } else if (specimenColor == SampleColor.UNKNOWN) {
            System.out.println("Unknown color detected, no action taken.");
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
    public SampleColor detectSpecimenColor() {
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

    public boolean goodSpecimenDetected(){
        SampleColor detectedSpecimenColor = detectSpecimenColor();
        return (detectedSpecimenColor == SampleColor.RED && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED) ||
                (detectedSpecimenColor == SampleColor.BLUE && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE);
    }

    // Update intake parameters dynamically (called in periodic)
    private void updateParameters() {
        // Update the power for each state dynamically from dashboard changes
        SpecimenIntakeStates.INTAKE_ON.updateIntakePower(INTAKE_PARAMS.INTAKE_ON_POWER);
        SpecimenIntakeStates.INTAKE_REVERSE.updateIntakePower(INTAKE_PARAMS.INTAKE_REVERSE_POWER);
        SpecimenIntakeStates.INTAKE_OFF.updateIntakePower(INTAKE_PARAMS.INTAKE_OFF_POWER);
    }
}
