package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

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

    public IntoTheDeepDriverBindings(GamepadEx gamepad) {
        Robot robot = Robot.getInstance();

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT STICK / RIGHT STICK - Default Driving           //
        //                                                      //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {

            defaultDriveCommand = new DefaultDriveCommand(Robot.getInstance().getDriveSubsystem(),
                    gamepad::getLeftY,
                    gamepad::getLeftX,
                    gamepad::getRightX);

            CommandScheduler.getInstance().setDefaultCommand(Robot.getInstance().getDriveSubsystem(), defaultDriveCommand);
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT BUMPER - Slow Mode                             //
        //                                                      //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            slowModeCommand = new SlowModeCommand(Robot.getInstance().getDriveSubsystem(),
                    gamepad::getLeftY,
                    gamepad::getLeftX,
                    gamepad::getRightX
            );

            gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenHeld(slowModeCommand);
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        //LEFT BUMPER - Cycle Telemetry Modes                   //
        //                                                      //
        //////////////////////////////////////////////////////////

        // Command to cycle telemetry modes using DriverStationTelemetryManager
        cycleTelemetryModeCommand = new InstantCommand(() -> {
            Robot.getInstance().getDriverStationTelemetryManager().cycleTelemetryMode();
        });

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(cycleTelemetryModeCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD_UP - Cycle Drive Modes                         //
        //                                                      //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            // Command to cycle through drive modes
            cycleDriveModeCommand = new InstantCommand(() -> {
                Robot.getInstance().getDriveSubsystem().cycleDriveMode();
            });

            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenPressed(cycleDriveModeCommand);
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        //  DPAD_DOWN - Reset Yaw                               //
        //                                                      //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(new InstantCommand(() -> {
                        Robot.getInstance().getDriveSubsystem().getMecanumDrive().lazyImu.get().resetYaw();
                        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);
                    }));
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        //  START BUTTON  - FIELD ORIENTED CONTROL              //
        //                                                      //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {

            gamepad.getGamepadButton(GamepadKeys.Button.START)
                    .toggleWhenPressed(new InstantCommand(() -> {
                        Robot.getInstance().getDriveSubsystem().fieldOrientedControl = true;
                    }), new InstantCommand(() -> {
                        Robot.getInstance().getDriveSubsystem().fieldOrientedControl = false;
                    }));
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON DRIVE FORWARD 6 INCHES                     //
        //                                                      //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            DriveForwardAction driveForwardAction = new DriveForwardAction();
            gamepad.getGamepadButton(GamepadKeys.Button.X)
                    .whenPressed(new ActionCommand(driveForwardAction, Collections.singleton(robot.getDriveSubsystem())))
                    .whenReleased(driveForwardAction::cancelAbruptly);
        }

    }
}

