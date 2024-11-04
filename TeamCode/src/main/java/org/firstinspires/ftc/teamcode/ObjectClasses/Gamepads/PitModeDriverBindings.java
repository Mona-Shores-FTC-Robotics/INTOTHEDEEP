package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class PitModeDriverBindings {

    public PitModeDriverBindings(GamepadEx gamepad) {
        // Reference to the robot's drive subsystem
        Robot robot = Robot.getInstance();
        DcMotorEx leftFront = robot.getDriveSubsystem().getMecanumDrive().leftFront;
        DcMotorEx leftBack = robot.getDriveSubsystem().getMecanumDrive().leftBack;
        DcMotorEx rightBack = robot.getDriveSubsystem().getMecanumDrive().rightBack;
        DcMotorEx rightFront = robot.getDriveSubsystem().getMecanumDrive().rightFront;

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT BUMPER - Cycle Telemetry Modes                  //
        //                                                      //
        //////////////////////////////////////////////////////////

        // Command to cycle telemetry modes using DriverStationTelemetryManager
        Command cycleTelemetryModeCommand = new InstantCommand(() -> {
            robot.getDriverStationTelemetryManager().cycleTelemetryMode();
        });

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(cycleTelemetryModeCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        // Button Bindings to Move Each Motor                   //
        //                                                      //
        //////////////////////////////////////////////////////////

        // Command for Front Left Motor (X / Square)
        Command runFrontLeftMotorCommand = new InstantCommand(() -> {
            leftFront.setPower(0.5); // Set motor power to 50%
        });

        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(runFrontLeftMotorCommand)
                .whenReleased(new InstantCommand(() -> {
                    leftFront.setPower(0);
                }));

        // Command for Front Right Motor (Y / Triangle)
        Command runFrontRightMotorCommand = new InstantCommand(() -> {
            rightFront.setPower(0.5);
        });

        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(runFrontRightMotorCommand)
                .whenReleased(new InstantCommand(() -> {
                    rightFront.setPower(0);
                }));

        // Command for Back Right Motor (B / O)
        Command runBackRightMotorCommand = new InstantCommand(() -> {
            rightBack.setPower(0.5);
        });

        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(runBackRightMotorCommand)
                .whenReleased(new InstantCommand(() -> {
                    rightBack.setPower(0);
                }));

        // Command for Back Left Motor (A / X)
        Command runBackLeftMotorCommand = new InstantCommand(() -> {
            leftBack.setPower(0.5);
        });

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(runBackLeftMotorCommand)
                .whenReleased(new InstantCommand(() -> {
                    leftBack.setPower(0);
                }));
    }
}
