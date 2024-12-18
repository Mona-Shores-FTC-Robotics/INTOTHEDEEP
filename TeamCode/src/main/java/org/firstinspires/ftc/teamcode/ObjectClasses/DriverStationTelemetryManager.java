package org.firstinspires.ftc.teamcode.ObjectClasses;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.AnalogBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.ButtonBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamePadBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamePadBindingManager;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamepadType;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class DriverStationTelemetryManager {

    // Enum to define telemetry modes
    private enum TelemetryMode {
        PIT_MODE,
        BASIC,
        BUTTON_BINDINGS,
        VERBOSE_DRIVE,
        VERBOSE_SAMPLE_INTAKE,
        VERBOSE_SAMPLE_LINEAR_ACTUATOR,
        VERBOSE_SAMPLE_LIFT,
        VERBOSE_SPECIMEN_ARM,
    }

    private final Telemetry telemetry;
    private TelemetryMode currentMode = TelemetryMode.BASIC;  // Start with BASIC mode

    // Constructor to initialize telemetry
    public DriverStationTelemetryManager(Telemetry telemetryObj) {
        telemetry = telemetryObj;
    }

    // Method to cycle between telemetry modes
    public void cycleTelemetryMode() {
        TelemetryMode[] modes = TelemetryMode.values();
        currentMode = modes[(currentMode.ordinal() + 1) % modes.length];
    }

    public void displayTelemetry(){
        displayTelemetry(null);
    }
    // Method to display telemetry based on the current mode
    public void displayTelemetry(GamePadBindingManager bindingManager) {
        // Always display the current telemetry mode at the top
        telemetry.addLine("Current Telemetry Mode: " + currentMode);

        if (Robot.getInstance().opModeType == Robot.OpModeType.TELEOP ||
                Robot.getInstance().opModeType == Robot.OpModeType.AUTO ) {
            switch (currentMode) {
                case BASIC:
                    displayBasicTelemetry();
                    break;
                case BUTTON_BINDINGS:
                    if (bindingManager!=null) {
                        displayButtonBindings(bindingManager); // Call the button bindings display
                    }
                    break;
                case VERBOSE_DRIVE:
                    displayVerboseDriveTelemetry();
                    break;
                case VERBOSE_SAMPLE_LIFT:
                    displayVerboseSampleLiftTelemetry();
                    break;
                case VERBOSE_SAMPLE_LINEAR_ACTUATOR:
                    displayVerboseSampleLinearActuatorTelemetry();
                    break;
                case VERBOSE_SPECIMEN_ARM:
                    displayVerboseSpecimenArmTelemetry();
                    break;
                default:
                    // If it's not listed, skip it
                    cycleTelemetryMode();
                    break;
            }
        } else if (Robot.getInstance().opModeType == Robot.OpModeType.PIT_MODE) {
            switch (currentMode) {
                case PIT_MODE:
                    displayPitModeTelemetry();
                    break;
                case BUTTON_BINDINGS:
                    displayButtonBindings(bindingManager); // Call the button bindings display
                    break;
                case BASIC:
                    displayBasicTelemetry();
                    break;
                case VERBOSE_DRIVE:
                    displayVerboseDriveTelemetry();
                    break;
                case VERBOSE_SAMPLE_LIFT:
                    displayVerboseSampleLiftTelemetry();
                    break;
                case VERBOSE_SAMPLE_LINEAR_ACTUATOR:
                    displayVerboseSampleLinearActuatorTelemetry();
                    break;
                case VERBOSE_SPECIMEN_ARM:
                    displayVerboseSpecimenArmTelemetry();
                    break;
                case VERBOSE_SAMPLE_INTAKE:
                    displayVerboseSampleIntakeTelemetry();
                    break;
            }
        }
        telemetry.update(); // Update telemetry after displaying data
    }

    @SuppressLint("DefaultLocale")
    private void displayPitModeTelemetry() {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;

        // Display formatted instructions for Pit Mode
        telemetry.addLine();

        // Button guide with formatted spacing for easy reading
        telemetry.addLine("<font face=\"monospace\">Button        - Motor</font>");
        telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left Motor</font>");
        telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right Motor</font>");
        telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Back Right Motor</font>");
        telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Back Left Motor</font>");

        // Display encoder counts and velocities for each motor
        Robot robot = Robot.getInstance();
        DcMotorEx leftFront = robot.getDriveSubsystem().getMecanumDrive().leftFront;
        DcMotorEx leftBack = robot.getDriveSubsystem().getMecanumDrive().leftBack;
        DcMotorEx rightBack = robot.getDriveSubsystem().getMecanumDrive().rightBack;
        DcMotorEx rightFront = robot.getDriveSubsystem().getMecanumDrive().rightFront;

        // Build the telemetry display using <pre> to preserve formatting
        String telemetryDisplay = "<font face=\"monospace\">=== Motor Status ===\n" +
                String.format("%s Enc:%06d | Vel:%+05d\n", "Front Left&nbsp;&nbsp;|", leftFront.getCurrentPosition(), (int) leftFront.getVelocity()) +
                String.format("%s Enc:%06d | Vel:%+05d\n", "Front Right&nbsp;|", rightFront.getCurrentPosition(), (int) rightFront.getVelocity()) +
                String.format("%s Enc:%06d | Vel:%+05d\n", "Back Left&nbsp;&nbsp;&nbsp;|", leftBack.getCurrentPosition(), (int) leftBack.getVelocity()) +
                String.format("%s Enc:%06d | Vel:%+05d\n", "Back Right&nbsp;&nbsp;|", rightBack.getCurrentPosition(), (int) rightBack.getVelocity()) +
                "</font>";

        // Add the formatted string to telemetry
        telemetry.addLine(telemetryDisplay);
    }

    // Method to display an error message, immediately in auto or deferred in teleop
    public void displayError(String message) {
        if (Robot.getInstance().isAutoMode()) {
            telemetry.addLine("Error: " + message);
            telemetry.update(); // Immediate update in Auto mode because there is no loop/telemetry manager
        } else {
            // In TeleOp, display the error as part of the regular telemetry loop
            telemetry.addLine("Error: " + message); // The regular telemetry loop will handle the update
        }
    }

    // Basic telemetry method
    private void displayBasicTelemetry() {
        telemetry.addLine();
        displayBaseTelemetry(telemetry);
        telemetry.addLine();
        if (Robot.getInstance().hasSubsystemWithErrorTelemetry(Robot.SubsystemType.DRIVE)) {
            DriveSubsystem.TelemetryHelper.displayBasicTelemetry(telemetry);
        }
        telemetry.addLine();
        // Add a header line to distinguish sample system telemetry
        telemetry.addLine("=== Sample Telemetry ===");
        if (Robot.getInstance().hasSubsystemWithErrorTelemetry(Robot.SubsystemType.SAMPLE_INTAKE)) {
            Robot.getInstance().getSampleIntakeSubsystem().displayBasicTelemetry(telemetry);
        }
        if ( Robot.getInstance().hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
                Robot.getInstance().getSampleLinearActuatorSubsystem().displayBasicTelemetry(telemetry);
        }
        if (Robot.getInstance().hasSubsystemWithErrorTelemetry(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            Robot.getInstance().getSampleLiftBucketSubsystem().displayBasicTelemetry(telemetry);
        }

        telemetry.addLine();
        // Add a header line to distinguish specimen system telemetry
        telemetry.addLine("=== Specimen Telemetry ===");
        if (Robot.getInstance().hasSubsystemWithErrorTelemetry(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            Robot.getInstance().getSpecimenIntakeSubsystem().displayBasicTelemetry(telemetry);
        }
        if (Robot.getInstance().hasSubsystemWithErrorTelemetry(Robot.SubsystemType.SPECIMEN_ARM)) {
            Robot.getInstance().getSpecimenArmSubsystem().displayBasicTelemetry(telemetry);
        }
    }

    // Verbose telemetry method for DriveSubsystem
    private void displayVerboseDriveTelemetry() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.DRIVE)) {
            DriveSubsystem.TelemetryHelper.displayVerboseTelemetry(telemetry);
        } else cycleTelemetryMode();
    }

    // Verbose telemetry method for SampleLiftSubsystem
    private void displayVerboseSampleLiftTelemetry() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            Robot.getInstance().getSampleLiftBucketSubsystem().displayVerboseTelemetry(telemetry);
        } else cycleTelemetryMode();
    }

    // Verbose telemetry method for SampleLinearActuatorSubsystem
    private void displayVerboseSampleLinearActuatorTelemetry() {
        if ( Robot.getInstance().hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
            Robot.getInstance().getSampleLinearActuatorSubsystem().displayVerboseTelemetry(telemetry);
        } else cycleTelemetryMode();
    }

    // Verbose telemetry method for SpecimenArmSubsystem
    private void displayVerboseSpecimenArmTelemetry() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            Robot.getInstance().getSpecimenArmSubsystem().displayVerboseTelemetry(telemetry);
        } else cycleTelemetryMode();
    }

    // Verbose telemetry method for SpecimenArmSubsystem
    private void displayVerboseSampleIntakeTelemetry() {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
            Robot.getInstance().getSampleIntakeSubsystem().displayVerboseTelemetry(telemetry);
        } else cycleTelemetryMode();
    }


    @SuppressLint("DefaultLocale")
    private void displayBaseTelemetry(Telemetry telemetry) {
        telemetry.addLine(String.format(
                "Loop Time (ms): %.0f | Teleop Time (s): %d",
                MatchConfig.getAverageLoopTime(),
                Robot.getInstance().getOpModeType() == Robot.OpModeType.TELEOP
                        ? MatchConfig.teleOpTimer.time(TimeUnit.SECONDS)
                        : 0 // Default value if not in TELEOP mode
        ));
        telemetry.addLine()
                .addData("Robot Type", MatchConfig.finalRobotType)
                .addData("Alliance Color", MatchConfig.finalAllianceColor)
                .addData("Side of Field", MatchConfig.finalSideOfField);
    }

    public void setPitModeTelemetry() {
        currentMode = TelemetryMode.PIT_MODE;
    }

    public void displayButtonBindings(GamePadBindingManager gamePadBindingManager) {

        // Determine the active OpMode type
        Robot.OpModeType currentOpMode = Robot.getInstance().getOpModeType();

        // Define which GamepadTypes are relevant for each OpMode
        List<GamepadType> relevantGamepadTypes = new ArrayList<>();
        switch (currentOpMode) {
            case TELEOP:
                relevantGamepadTypes.add(GamepadType.DRIVER);
                relevantGamepadTypes.add(GamepadType.OPERATOR);
                break;
            case PIT_MODE:
                relevantGamepadTypes.add(GamepadType.PIT);
                break;
            case AUTO:
            default:
                //no bindings for auto
                break;
        }
        for (GamepadType gamepadType : relevantGamepadTypes) {
            telemetry.addLine(String.format("<b>%s Gamepad</b>", gamepadType));

            for (GamePadBinding binding : gamePadBindingManager.getBindings()) {
                if (binding instanceof ButtonBinding && ((ButtonBinding) binding).getGamepadType() == gamepadType) {
                    ButtonBinding buttonBinding = (ButtonBinding) binding;
                    String buttonName = buttonBinding.getButtonName();
                    String description = buttonBinding.getDescription();
                    telemetry.addLine(String.format("<font face=\"monospace\">%s: %s</font>", buttonName, description));
                } else if (binding instanceof AnalogBinding && ((AnalogBinding) binding).getGamepadType() == gamepadType) {
                    AnalogBinding analogBinding = (AnalogBinding) binding;
                    String combinedAnalogInputNames = String.join(", ", analogBinding.getAnalogInputNames());
                    String description = analogBinding.getDescription();

                    // Display the combined analog input names with the single description
                    telemetry.addLine(String.format("<font face=\"monospace\">%s: %s</font>", combinedAnalogInputNames, description));
                }
            }
            telemetry.addLine(""); // Add a blank line for spacing
        }

        telemetry.addLine("========================");
        // Removed telemetry.update(); to prevent premature updates
    }

}
