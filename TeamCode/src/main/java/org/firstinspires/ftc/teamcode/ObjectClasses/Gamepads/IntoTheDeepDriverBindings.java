package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.AnalogBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.ButtonBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamePadBindingManager;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamepadType;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveToObservationZone;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.SlowModeCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenButtonHandling;

import java.util.Collections;
import java.util.function.DoubleSupplier;

public class IntoTheDeepDriverBindings {
    GamepadEx driverGamePad;
    Robot robot;
    GamePadBindingManager bindingManager;
    DriveToObservationZone driveToObservationZoneAction;

    public IntoTheDeepDriverBindings(GamepadEx gamePad, GamePadBindingManager gamePadBindingManager) {
        robot = Robot.getInstance();
        driverGamePad = gamePad;
        bindingManager = gamePadBindingManager;

        //////////////////////////////////////////////////////////
        // LEFT STICK / RIGHT STICK - Default Driving           //
        //////////////////////////////////////////////////////////
        bindDefaultDriving(driverGamePad::getLeftY, driverGamePad::getLeftX, (driverGamePad::getRightX));

        //////////////////////////////////////////////////////////
        // LEFT BUMPER - Cycle Telemetry                        //
        //////////////////////////////////////////////////////////
        cycleTelemetry(GamepadKeys.Button.LEFT_BUMPER);

        //////////////////////////////////////////////////////////
        // RIGHT BUMPER - Slow Mode                             //
        //////////////////////////////////////////////////////////
        bindSlowMode(GamepadKeys.Button.RIGHT_BUMPER);

        //////////////////////////////////////////////////////////
        // DPAD_UP - Cycle Drive Modes                          //
        //////////////////////////////////////////////////////////
        cycleDriveMode(GamepadKeys.Button.DPAD_UP);

        //////////////////////////////////////////////////////////
        // DPAD_DOWN - Reset Yaw                               //
        //////////////////////////////////////////////////////////
        resetGyro(GamepadKeys.Button.DPAD_DOWN);

        //////////////////////////////////////////////////////////
        // START BUTTON  - FIELD ORIENTED CONTROL              //
        //////////////////////////////////////////////////////////
        toggleFieldOrientedControl(GamepadKeys.Button.START);

        //////////////////////////////////////////////////////////
        // X BUTTON                                             //
        //////////////////////////////////////////////////////////
        driveToObservationZone(GamepadKeys.Button.X);

        //////////////////////////////////////////////////////////
        // A BUTTON - Specimen Handling (Intake and Scoring)    //
        //////////////////////////////////////////////////////////
        bindSpecimenIntakeAndScore(GamepadKeys.Button.A);


    }

    private void driveToObservationZone(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command driveToObservationZoneCommand = new InstantCommand(() -> {
                driveToObservationZoneAction = new DriveToObservationZone();
                ActionCommand actionCommand = new ActionCommand(driveToObservationZoneAction, Collections.singleton(robot.getDriveSubsystem()));
                actionCommand.schedule();
            });

            Command stopDriveToObservationZoneCommand = new InstantCommand(() -> {
                if (driveToObservationZoneAction != null) {
                    driveToObservationZoneAction.cancelAbruptly();
                    driveToObservationZoneAction = null;
                }
            });

            driverGamePad.getGamepadButton(button)
                    .whenPressed(driveToObservationZoneCommand)
                    .whenReleased(stopDriveToObservationZoneCommand);

            // Register the drive forward binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    button,
                    "Drive to Observation Zone"
            ));
        }
    }

    private void toggleFieldOrientedControl(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command enableFieldOriented = new InstantCommand(() -> robot.getDriveSubsystem().fieldOrientedControl = true);
            Command disableFieldOriented = new InstantCommand(() -> robot.getDriveSubsystem().fieldOrientedControl = false);

            driverGamePad.getGamepadButton(button)
                    .toggleWhenPressed(enableFieldOriented, disableFieldOriented);

            // Register the field-oriented toggle
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    button,
                    "Toggle field-oriented control."
            ));
        }
    }

    private void resetGyro(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command resetYawCommand = new InstantCommand(() -> {
                robot.getDriveSubsystem().getMecanumDrive().lazyImu.get().resetYaw();
                robot.getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);
            });

            driverGamePad.getGamepadButton(button)
                    .whenPressed(resetYawCommand);

            // Register the yaw reset
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    button,
                    resetYawCommand,
                    "Reset Localization to Start Pose"
            ));
        }
    }

    private void cycleDriveMode(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command cycleDriveModeCommand = new InstantCommand(() -> robot.getDriveSubsystem().cycleDriveMode());

            driverGamePad.getGamepadButton(button)
                    .whenPressed(cycleDriveModeCommand);

            // Register the drive mode cycling
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    button,
                    cycleDriveModeCommand,
                    "Cycle Drive Mode"
            ));
        }
    }

    private void cycleTelemetry(GamepadKeys.Button button) {
        // Command to cycle telemetry modes using DriverStationTelemetryManager
        Command cycleTelemetryModeCommand = new InstantCommand(robot.getDriverStationTelemetryManager()::cycleTelemetryMode);

        driverGamePad.getGamepadButton(button)
                .whenPressed(cycleTelemetryModeCommand);

        bindingManager.registerBinding(new ButtonBinding(
                GamepadType.DRIVER,
                button,
                cycleTelemetryModeCommand,
                "Cycle telemetry"
        ));
    }
    private void bindSlowMode(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command slowModeCommand = new SlowModeCommand(robot.getDriveSubsystem(),
                    driverGamePad::getLeftY,
                    driverGamePad::getLeftX,
                    driverGamePad::getRightX
            );

            driverGamePad.getGamepadButton(button)
                    .whenHeld(slowModeCommand);

            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    button,
                    slowModeCommand,
                    "Slow Mode"
            ));
        }
    }

    private void bindDefaultDriving(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command defaultDriveCommand = new DefaultDriveCommand(robot.getDriveSubsystem(),
                    leftY,
                    leftX,
                    rightX);

            CommandScheduler.getInstance().setDefaultCommand(robot.getDriveSubsystem(), defaultDriveCommand);

            bindingManager.registerBinding(new AnalogBinding(GamepadType.DRIVER, "Left Y", "Drive"));
            bindingManager.registerBinding(new AnalogBinding(GamepadType.DRIVER, "Left X", "Strafe"));
            bindingManager.registerBinding(new AnalogBinding(GamepadType.DRIVER, "Right X", "Rotate"));
        }
    }

    private void bindSpecimenIntakeAndScore(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM) &&
                robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            SpecimenButtonHandling specimenHandlingStateMachine = robot.getSpecimenButtonHandling();
            Command specimenHandlePressCommand = new InstantCommand(specimenHandlingStateMachine::onSpecimenHandleButtonPress);

            driverGamePad.getGamepadButton(button)
                    .whenPressed(specimenHandlePressCommand);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    button,
                    specimenHandlePressCommand,
                    "Pickup/Score Specimen"
            ));
        }
    }
}
