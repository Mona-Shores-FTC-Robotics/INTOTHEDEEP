package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@Config
public class SampleIntakeSubsystem extends SubsystemBase {

    public static class IntakeParams {
        public double INTAKE_ON_POWER = .8;
        public double INTAKE_REVERSE_POWER = -.8;
        public double INTAKE_OFF_POWER = 0.0;
        public double MAX_POWER = 1.0;  // Max allowable power for intake motor
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

    private final DcMotorEx sampleIntake;
    private SampleIntakeStates currentState;
    private double currentPower;

    // Constructor
    public SampleIntakeSubsystem(final HardwareMap hMap, final String intakeMotorName) {
        sampleIntake = hMap.get(DcMotorEx.class, intakeMotorName);
    }

    // Initialize intake motor
    public void init() {
        Robot.getInstance().registerSubsystem(Robot.SubsystemType.SAMPLE_INTAKE);  // Register subsystem
        sampleIntake.setDirection(DcMotorEx.Direction.FORWARD);
        sampleIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sampleIntake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        setCurrentState(SampleIntakeStates.INTAKE_OFF);  // Set default state to off
        currentPower = INTAKE_PARAMS.INTAKE_OFF_POWER;  // Cache initial power
    }

    // Set the current intake state and update power
    public void setCurrentState(SampleIntakeStates state) {
        currentState = state;
        setPower(state.power);
    }

    // Set motor power, ensuring it's within limits
    private void setPower(double power) {
        currentPower = Range.clip(power, -INTAKE_PARAMS.MAX_POWER, INTAKE_PARAMS.MAX_POWER);  // Clip power to safe range
        sampleIntake.setPower(currentPower);  // Apply the clipped power
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
        // Update parameters in real time, if adjusted via dashboard
        updateParameters();
        updateDashboardTelemetry();
    }

    // Telemetry display for the dashboard
    public void updateDashboardTelemetry() {
        MatchConfig.telemetryPacket.put("Sample Intake/State", currentState.toString());
        MatchConfig.telemetryPacket.put("Sample Intake/Power", currentPower);
    }

    // Basic telemetry display with context for the driver station
    public void displayBasicTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Sample Intake Status", String.format("State: %s", currentState));
    }

    // Getters for telemetry use or other purposes
    public SampleIntakeStates getCurrentState() {
        return currentState;
    }

    public double getCurrentPower() {
        return currentPower;
    }
}
