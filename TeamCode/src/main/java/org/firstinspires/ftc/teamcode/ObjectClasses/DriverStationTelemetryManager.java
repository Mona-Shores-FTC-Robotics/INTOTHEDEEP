package org.firstinspires.ftc.teamcode.ObjectClasses;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.TimeUnit;
public class DriverStationTelemetryManager {

    // Enum to define telemetry modes
    private enum TelemetryMode {
        BASIC,
        VERBOSE_DRIVE,
        VERBOSE_ENCODERS,
        VERBOSE_HEADING,
        VERBOSE_SAMPLE_LIFT,
        VERBOSE_SAMPLE_LINEAR_ACTUATOR // Updated telemetry mode name for sample lift
        // You can add more modes later, like VERBOSE_OTHER_SUBSYSTEM, etc.
    }

    private final Telemetry telemetry;
    private TelemetryMode currentMode = TelemetryMode.BASIC;  // Start with BASIC mode

    // Constructor to initialize telemetry
    public DriverStationTelemetryManager(Telemetry telemetryObj) {
        telemetry = telemetryObj;
    }

    // Method to cycle between telemetry modes
    public void cycleTelemetryMode() {
        currentMode = TelemetryMode.values()[(currentMode.ordinal() + 1) % TelemetryMode.values().length];
    }

    // Method to display telemetry based on the current mode
    public void displayTelemetry() {
        // Always display the current telemetry mode at the top
        telemetry.addLine("Current Telemetry Mode: " + currentMode);

        switch (currentMode) {
            case BASIC:
                displayBasicTelemetry();
                break;
            case VERBOSE_DRIVE:
                displayVerboseDriveTelemetry();
                break;
            case VERBOSE_ENCODERS:
                displayVerboseEncodersTelemetry();
                break;
            case VERBOSE_HEADING:
                displayVerboseHeadingTelemetry();
                break;
            case VERBOSE_SAMPLE_LIFT:
                displayVerboseSampleLiftTelemetry();
                break;
            case VERBOSE_SAMPLE_LINEAR_ACTUATOR:
                displayVerboseSampleLiftTelemetry();
                break;
        }
        telemetry.update();  // Make sure to update telemetry after displaying data
    }

    // Method to display an error message, immediately in auto or deferred in teleop
    public void displayError(String message) {
        if (Robot.getInstance().isAutoMode()) {
            telemetry.addLine("Error: " + message);
            telemetry.update();  // Immediate update in Auto mode because there is no loop/telemetry manager
        } else {
            // In TeleOp, display the error as part of the regular telemetry loop
            telemetry.addLine("Error: " + message);  // The regular telemetry loop will handle the update
        }
    }

    // Basic telemetry method
    private void displayBasicTelemetry() {
        displayTimeTelemetry(telemetry);
        telemetry.addLine();
        // Check and display DriveSubsystem telemetry if available
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Robot.getInstance().getDriveSubsystem().displayBasicTelemetry(telemetry);
        }
        telemetry.addLine();
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
            Robot.getInstance().getSampleIntakeSubsystem().displayBasicTelemetry(telemetry);
        }
        telemetry.addLine();
        // Check and display SampleLiftSubsystem telemetry if available
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT)) {
            Robot.getInstance().getSampleLiftSubsystem().displayBasicTelemetry(telemetry);
        }
        telemetry.addLine();
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
            Robot.getInstance().getSampleLinearActuatorSubsystem().displayBasicTelemetry(telemetry);
        }
    }

    // Verbose telemetry method for DriveSubsystem heading
    private void displayVerboseHeadingTelemetry() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Robot.getInstance().getDriveSubsystem().displayYawTelemetry(telemetry);
        }
    }

    // Verbose telemetry method for DriveSubsystem
    private void displayVerboseDriveTelemetry() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Robot.getInstance().getDriveSubsystem().displayVerboseTelemetry(telemetry);
        }
    }

    // Verbose telemetry method for DriveSubsystem encoders
    private void displayVerboseEncodersTelemetry() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Robot.getInstance().getDriveSubsystem().displayVerboseEncodersTelemetry(telemetry);
        }
    }

    // Verbose telemetry method for SampleLiftSubsystem
    private void displayVerboseSampleLiftTelemetry() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT)) {
            Robot.getInstance().getSampleLiftSubsystem().displayVerboseTelemetry(telemetry);
        }
    }

    // Verbose telemetry method for SampleLiftSubsystem
    private void displayVerboseSampleLinearActuatorTelemetry() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
            Robot.getInstance().getSampleLinearActuatorSubsystem().displayVerboseTelemetry(telemetry);
        }
    }

    private void displayTimeTelemetry(Telemetry telemetry) {
        // Display average loop time in telemetry
        telemetry.addData("Average Loop Time (ms)", "%.1f", MatchConfig.getAverageLoopTime());
        telemetry.addData("Teleop Time (s)", "%d", MatchConfig.teleOpTimer.time(TimeUnit.SECONDS));
    }
}
