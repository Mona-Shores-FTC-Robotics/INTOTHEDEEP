package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.ButtonBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamePadBindingManager;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamepadType;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleButtonHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.ChangeSampleBucketPositionAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleTwister.SampleTwisterSubsystem;

import java.util.Collections;
import java.util.Set;

public class PitModeDriverBindings {
    Robot robot;
    GamePadBindingManager bindingManager;
    GamepadEx driverGamePad;

    public PitModeDriverBindings(GamepadEx gamepad, GamePadBindingManager gamePadBindingManager) {
        // Reference to the robot's drive subsystem
        robot = Robot.getInstance();
        bindingManager = gamePadBindingManager;
        driverGamePad = gamepad;

        DcMotorEx leftFront = robot.getDriveSubsystem().getMecanumDrive().leftFront;
        DcMotorEx leftBack = robot.getDriveSubsystem().getMecanumDrive().leftBack;
        DcMotorEx rightBack = robot.getDriveSubsystem().getMecanumDrive().rightBack;
        DcMotorEx rightFront = robot.getDriveSubsystem().getMecanumDrive().rightFront;

        //////////////////////////////////////////////////////////
        // LEFT BUMPER - Cycle Telemetry                        //
        //////////////////////////////////////////////////////////
        cycleTelemetry(GamepadKeys.Button.LEFT_BUMPER);
        MoveFlipper(GamepadKeys.Button.X);
        SampleTwister(GamepadKeys.Button.Y);
        SampleIntake(GamepadKeys.Button.A);
        SampleOuttake(GamepadKeys.Button.B);
        //////////////////////////////////////////////////////////
        // Button Bindings to Move Each Motor                   //
        //////////////////////////////////////////////////////////

        // Command for Front Left Motor (X / Square)
//        Command runFrontLeftMotorCommand = new InstantCommand(() -> {
//            leftFront.setPower(0.5); // Set motor power to 50%
//        });
//
//        gamepad.getGamepadButton(GamepadKeys.Button.X)
//                .whileHeld(runFrontLeftMotorCommand)
//                .whenReleased(new InstantCommand(() -> {
//                    leftFront.setPower(0);
//                }));
//
//        // Register Front Left Motor binding
//        bindingManager.registerBinding(new ButtonBinding(
//                GamepadType.PIT,
//                GamepadKeys.Button.X,
//                runFrontLeftMotorCommand,
//                "Run Front Left Motor at 50% power (Hold X)."
//        ));
//
//        // Command for Front Right Motor (Y / Triangle)
//        Command runFrontRightMotorCommand = new InstantCommand(() -> {
//            rightFront.setPower(0.5);
//        });
//
//        gamepad.getGamepadButton(GamepadKeys.Button.Y)
//                .whileHeld(runFrontRightMotorCommand)
//                .whenReleased(new InstantCommand(() -> {
//                    rightFront.setPower(0);
//                }));
//
//        // Register Front Right Motor binding
//        bindingManager.registerBinding(new ButtonBinding(
//                GamepadType.PIT,
//                GamepadKeys.Button.Y,
//                runFrontRightMotorCommand,
//                "Run Front Right Motor at 50% power (Hold Y)."
//        ));
//
//        // Command for Back Right Motor (B / O)
//        Command runBackRightMotorCommand = new InstantCommand(() -> {
//            rightBack.setPower(0.5);
//        });
//
//        gamepad.getGamepadButton(GamepadKeys.Button.B)
//                .whileHeld(runBackRightMotorCommand)
//                .whenReleased(new InstantCommand(() -> {
//                    rightBack.setPower(0);
//                }));
//
//        // Register Back Right Motor binding
//        bindingManager.registerBinding(new ButtonBinding(
//                GamepadType.PIT,
//                GamepadKeys.Button.B,
//                runBackRightMotorCommand,
//                "Run Back Right Motor at 50% power (Hold B)."
//        ));
//
//        // Command for Back Left Motor (A / X)
//        Command runBackLeftMotorCommand = new InstantCommand(() -> {
//            leftBack.setPower(0.5);
//        });
//
//        gamepad.getGamepadButton(GamepadKeys.Button.A)
//                .whileHeld(runBackLeftMotorCommand)
//                .whenReleased(new InstantCommand(() -> {
//                    leftBack.setPower(0);
//                }));
//
//        // Register Back Left Motor binding
//        bindingManager.registerBinding(new ButtonBinding(
//                GamepadType.PIT,
//                GamepadKeys.Button.A,
//                runBackLeftMotorCommand,
//                "Run Back Left Motor at 50% power (Hold A)."
//        ));


    }

    private void SampleOuttake(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
            SampleIntakeSubsystem intakeSubsystem = Robot.getInstance().getSampleIntakeSubsystem();
            Command turnIntakeOn = new InstantCommand(intakeSubsystem::reverseIntake);
            Command turnIntakeOff = new InstantCommand(intakeSubsystem::turnOffIntake);

            driverGamePad.getGamepadButton(button)
                    .toggleWhenPressed(turnIntakeOn, turnIntakeOff);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    "Reverse Sample Intake"
            ));
        }
    }

    private void SampleIntake(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
            SampleIntakeSubsystem intakeSubsystem = Robot.getInstance().getSampleIntakeSubsystem();
            Command turnIntakeOn = new InstantCommand(intakeSubsystem::turnOnIntake);
            Command turnIntakeOff = new InstantCommand(intakeSubsystem::turnOffIntake);

            driverGamePad.getGamepadButton(button)
                    .toggleWhenPressed(turnIntakeOn, turnIntakeOff);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    "Sample Intake"
            ));
        }
    }

    private void cycleTelemetry(GamepadKeys.Button button) {
        // Command to cycle telemetry modes using DriverStationTelemetryManager
        Command cycleTelemetryModeCommand = new InstantCommand(() -> robot.getDriverStationTelemetryManager().cycleTelemetryMode());

        driverGamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(cycleTelemetryModeCommand);

        // Register LEFT BUMPER binding
        bindingManager.registerBinding(new ButtonBinding(
                GamepadType.PIT,
                button,
                cycleTelemetryModeCommand,
                "Cycle through telemetry display modes."
        ));
    }

    private void SampleTwister(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_TWISTER)) {
            SampleTwisterSubsystem sampleTwisterSubsystem = Robot.getInstance().getSampleTiwsterSubsystem();
            Command faceInwards = new InstantCommand(sampleTwisterSubsystem::setTwisterServoFaceInward);
            Command faceOutwards = new InstantCommand(sampleTwisterSubsystem::setTwisterServoFaceOutwards);

            driverGamePad.getGamepadButton(button)
                    .toggleWhenPressed(faceInwards, faceOutwards);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    "Twist"
            ));
        }
    }

    private void MoveFlipper(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR))
        {
            SampleLinearActuatorSubsystem linearActuatorSubsystem = Robot.getInstance().getSampleLinearActuatorSubsystem();
            Command flipIntakeDown = new InstantCommand(linearActuatorSubsystem::flipSampleIntakeDown);
            Command flipIntakeUp = new InstantCommand(linearActuatorSubsystem::setFlipperUp);

            driverGamePad.getGamepadButton(button)
                    .toggleWhenPressed(flipIntakeDown, flipIntakeUp);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    flipIntakeDown,
                    "Move Flipper"
            ));
        }
    }

}
