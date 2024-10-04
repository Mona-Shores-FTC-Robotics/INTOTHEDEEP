package org.firstinspires.ftc.teamcode.ObjectClasses;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

public class DriverStationTelemetryManager {

    // Enum to define telemetry modes
    private enum TelemetryMode {
        BASIC,
        VERBOSE_DRIVE,
        VERBOSE_ENCODERS  // New telemetry mode for encoders
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
        telemetry.addLine("Basic Telemetry:");
        // Check and display DriveSubsystem telemetry if available
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Robot.getInstance().getDriveSubsystem().displayBasicTelemetry(telemetry);
        }
        // Add more basic telemetry for other subsystems if needed
    }

    // Verbose telemetry method for DriveSubsystem
    private void displayVerboseDriveTelemetry() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Robot.getInstance().getDriveSubsystem().displayVerboseTelemetry(telemetry);
        }
    }

    private void displayVerboseEncodersTelemetry() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.DRIVE) && Robot.getInstance().robotType==Robot.RobotType.ROBOT_CHASSIS_TWO_DEAD_WHEEL_INTERNAL_IMU) {
            Robot.getInstance().getDriveSubsystem().displayVerboseEncodersTelemetry(telemetry);
        }
    }

}
