package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveForwardAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.SlowModeCommand;

import java.util.Collections;

public class IntoTheDeepDriverBindings {
    public Command defaultDriveCommand;
    public Command slowModeCommand;
    public Command cycleTelemetryModeCommand;
    public Command cycleDriveModeCommand;
    public DriveForwardAction currentDriveForwardAction;

    public IntoTheDeepDriverBindings(GamepadEx gamepad) {
        Robot robot = Robot.getInstance();
        ButtonBindingManager bindingManager = ButtonBindingManager.getInstance();

        //////////////////////////////////////////////////////////
        // LEFT STICK / RIGHT STICK - Default Driving           //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            defaultDriveCommand = new DefaultDriveCommand(robot.getDriveSubsystem(),
                    gamepad::getLeftY,
                    gamepad::getLeftX,
                    gamepad::getRightX);

            CommandScheduler.getInstance().setDefaultCommand(robot.getDriveSubsystem(), defaultDriveCommand);

            // Register the default drive command (no specific button)
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER, // Assuming DRIVER gamepad
                    null, // No specific button
                    defaultDriveCommand,
                    "Default driving with left and right sticks."
            ));
        }

        //////////////////////////////////////////////////////////
        // RIGHT BUMPER - Slow Mode                             //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            slowModeCommand = new SlowModeCommand(robot.getDriveSubsystem(),
                    gamepad::getLeftY,
                    gamepad::getLeftX,
                    gamepad::getRightX
            );

            gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenHeld(slowModeCommand);

            // Register the slow mode binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    GamepadKeys.Button.RIGHT_BUMPER,
                    slowModeCommand,
                    "Hold to activate slow driving mode."
            ));
        }

        //////////////////////////////////////////////////////////
        // LEFT BUMPER - Cycle Telemetry Modes                   //
        //////////////////////////////////////////////////////////

        cycleTelemetryModeCommand = new InstantCommand(() -> {
            Robot.getInstance().getDriverStationTelemetryManager().cycleTelemetryMode();
        });

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(cycleTelemetryModeCommand);

        // Register the telemetry mode cycling
        bindingManager.registerBinding(new ButtonBinding(
                GamepadType.DRIVER,
                GamepadKeys.Button.LEFT_BUMPER,
                cycleTelemetryModeCommand,
                "Cycle through telemetry display modes."
        ));

        //////////////////////////////////////////////////////////
        // DPAD_UP - Cycle Drive Modes                         //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            cycleDriveModeCommand = new InstantCommand(() -> {
                robot.getDriveSubsystem().cycleDriveMode();
            });

            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenPressed(cycleDriveModeCommand);

            // Register the drive mode cycling
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    GamepadKeys.Button.DPAD_UP,
                    cycleDriveModeCommand,
                    "Cycle through drive modes."
            ));
        }

        //////////////////////////////////////////////////////////
        // DPAD_DOWN - Reset Yaw                               //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command resetYawCommand = new InstantCommand(() -> {
                robot.getDriveSubsystem().getMecanumDrive().lazyImu.get().resetYaw();
                robot.getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);
            });

            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(resetYawCommand);

            // Register the yaw reset
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    GamepadKeys.Button.DPAD_DOWN,
                    resetYawCommand,
                    "Reset the robot's yaw orientation."
            ));
        }

        //////////////////////////////////////////////////////////
        // START BUTTON  - FIELD ORIENTED CONTROL              //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command enableFieldOriented = new InstantCommand(() -> {
                robot.getDriveSubsystem().fieldOrientedControl = true;
            });

            Command disableFieldOriented = new InstantCommand(() -> {
                robot.getDriveSubsystem().fieldOrientedControl = false;
            });

            gamepad.getGamepadButton(GamepadKeys.Button.START)
                    .toggleWhenPressed(enableFieldOriented, disableFieldOriented);

            // Register the field-oriented toggle
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    GamepadKeys.Button.START,
                    null, // It's a toggle, not a single command
                    "Toggle field-oriented control."
            ));
        }

        //////////////////////////////////////////////////////////
        // X BUTTON DRIVE FORWARD 6 INCHES                     //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command driveForwardCommand = new InstantCommand(() -> {
                DriveForwardAction driveForwardAction = new DriveForwardAction();
                ActionCommand driveActionCommand = new ActionCommand(driveForwardAction, Collections.singleton(robot.getDriveSubsystem()));
                driveActionCommand.schedule();
                currentDriveForwardAction = driveForwardAction;
            });

            Command stopDriveForwardCommand = new InstantCommand(() -> {
                if (currentDriveForwardAction != null) {
                    currentDriveForwardAction.cancelAbruptly();
                    currentDriveForwardAction = null;
                }
            });

            gamepad.getGamepadButton(GamepadKeys.Button.X)
                    .whenPressed(driveForwardCommand)
                    .whenReleased(stopDriveForwardCommand);

            // Register the drive forward binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    GamepadKeys.Button.X,
                    driveForwardCommand, // Could also consider a composite if needed
                    "Drive forward 6 inches."
            ));
        }
    }
}
