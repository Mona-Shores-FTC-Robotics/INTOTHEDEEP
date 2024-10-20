package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.SlowModeCommand;

public class IntoTheDeepDriverBindings {
    public Command defaultDriveCommand;
    public Command slowModeCommand;
    public Command cycleTelemetryModeCommand;
    public Command cycleDriveModeCommand;

    public IntoTheDeepDriverBindings(GamepadEx gamepad) {

        //Make the commands to use for the bindings
        MakeCommands(gamepad);

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT STICK / RIGHT STICK - Default Driving           //
        //                                                      //
        //////////////////////////////////////////////////////////
        CommandScheduler.getInstance().setDefaultCommand(Robot.getInstance().getDriveSubsystem(), defaultDriveCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT BUMPER - Slow Mode                             //
        //                                                      //
        //////////////////////////////////////////////////////////

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(slowModeCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        // BACK BUTTON - Cycle Telemetry Mode                  //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(cycleTelemetryModeCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        // BACK BUTTON - Cycle Drive Mode                       //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(cycleDriveModeCommand);


        //////////////////////////////////////////////////////////
        //                                                      //
        // BACK BUTTON - Cycle Drive Mode                       //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    Robot.getInstance().getDriveSubsystem().getMecanumDrive().lazyImu.get().resetYaw();
                    Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField);
                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  START BUTTON  - FIELD ORIENTED CONTROL              //
        //                                                      //
        //////////////////////////////////////////////////////////
        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .toggleWhenPressed(new InstantCommand(() -> {
                    Robot.getInstance().getDriveSubsystem().fieldOrientedControl = true;
                }), new InstantCommand(() -> {
                    Robot.getInstance().getDriveSubsystem().fieldOrientedControl = false;
                }));
    }

    private void MakeCommands(GamepadEx gamepad) {
        defaultDriveCommand = new DefaultDriveCommand(Robot.getInstance().getDriveSubsystem(),
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX
        );

        slowModeCommand = new SlowModeCommand(Robot.getInstance().getDriveSubsystem(),
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX
        );

        // Command to cycle telemetry modes using DriverStationTelemetryManager
        cycleTelemetryModeCommand = new InstantCommand(() -> {
            Robot.getInstance().getDriverStationTelemetryManager().cycleTelemetryMode();
        });

        // Command to cycle telemetry modes using DriverStationTelemetryManager
        cycleDriveModeCommand = new InstantCommand(() -> {
            Robot.getInstance().getDriveSubsystem().cycleDriveMode();
        });

    }
}

