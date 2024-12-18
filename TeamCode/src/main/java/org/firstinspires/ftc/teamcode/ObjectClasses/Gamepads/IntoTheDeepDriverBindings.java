package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import static com.example.sharedconstants.FieldConstants.AllianceColor.BLUE;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.example.sharedconstants.FieldConstants;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.AnalogBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.ButtonBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamePadBindingManager;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamepadType;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveForwardAndBack;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveToChamber;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveToNetZone;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveToObservationZone;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DriveAtFixedDegreeHeadingCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenButtonHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.function.DoubleSupplier;

public class IntoTheDeepDriverBindings {
    GamepadEx driverGamePad;
    Robot robot;
    GamePadBindingManager bindingManager;
    DriveToObservationZone driveToObservationZoneAction;
    DriveToNetZone driveToNetZoneAction;
    DriveToChamber driveToChamberAction;
    DriveForwardAndBack driveForwardAndBack;
    public Command driveTo90OrMinus90;
    public Command driveTo45or225;

    private TriggerReader fixedBucketScoreAngleTrigger;
    private TriggerReader fixedSpecimenScoreAngleTrigger;
    private boolean isBucketAngleCommandActive = false; // Track if 45/225 mode is active
    private boolean isSpecimenAngleCommandActive = false; // Track if -90/90 mode is active


    public IntoTheDeepDriverBindings(GamepadEx gamePad , GamePadBindingManager gamePadBindingManager) {
        robot = Robot.getInstance();
        driverGamePad = gamePad;
        bindingManager = gamePadBindingManager;

        bindDefaultDriving(driverGamePad::getLeftY , driverGamePad::getLeftX , (driverGamePad::getRightX));
        cycleTelemetry(GamepadKeys.Button.LEFT_BUMPER);
        bindSlowMode(GamepadKeys.Button.RIGHT_BUMPER);
        cycleDriveMode(GamepadKeys.Button.DPAD_UP);
        resetGyro(GamepadKeys.Button.DPAD_DOWN);
        toggleFieldOrientedControl(GamepadKeys.Button.START);
        driveToNetZone(GamepadKeys.Button.X);
        driveToObservationZone(GamepadKeys.Button.B);
        bindSpecimenArmIntakeAndScore(GamepadKeys.Button.A);

        bindSpecimenIntakeToggle(GamepadKeys.Button.Y);


        //////////////////////////////////////////////////////////
        // A BUTTON - Specimen Handling (Intake and Scoring)    //
        //////////////////////////////////////////////////////////
//        bindBucketAngle(GamepadKeys.Trigger.LEFT_TRIGGER);

        //////////////////////////////////////////////////////////
        // y BUTTON - Reverse Specimen Intake                   //
        //////////////////////////////////////////////////////////
//        bindSpecimenAngleDriving(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    private void bindSpecimenIntakeToggle(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            SpecimenIntakeSubsystem intakeSubsystem = Robot.getInstance().getSpecimenIntakeSubsystem();
            Command turnIntakeOn = new InstantCommand(intakeSubsystem::reverseIntake);
            Command turnIntakeOff = new InstantCommand(intakeSubsystem::turnOffIntake);

            driverGamePad.getGamepadButton(button)
                    .toggleWhenPressed(turnIntakeOn , turnIntakeOff);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR ,
                    button ,
                    "Toggle Specimen Intake"
            ));
        }
    }

    private void retrySpecimenScore(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command retrySpecimenScoreCommand =
                    new InstantCommand(() -> {
                        driveForwardAndBack = new DriveForwardAndBack(10);
                        ActionCommand actionCommand = new ActionCommand(driveForwardAndBack , Collections.singleton(robot.getDriveSubsystem()));
                        actionCommand.schedule();
                    });
            driverGamePad.getGamepadButton(button)
                    .whenPressed(retrySpecimenScoreCommand);
            // Register the drive forward binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER ,
                    button ,
                    "Retry Specimen Scoring"
            ));
        }
    }

    private void driveToNetZone(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command driveToNetZoneCommand =
                    new InstantCommand(() -> {
                        driveToNetZoneAction = new DriveToNetZone();
                        ActionCommand actionCommand = new ActionCommand(driveToNetZoneAction , Collections.singleton(robot.getDriveSubsystem()));
                        actionCommand.schedule();
                    });

            Command stopDriveToNetZone = new InstantCommand(() -> {
                if (driveToNetZoneAction != null) {
                    driveToNetZoneAction.cancelAbruptly();
                    driveToNetZoneAction = null;
                }
            });

            driverGamePad.getGamepadButton(button)
                    .whenPressed(driveToNetZoneCommand)
                    .whenReleased(stopDriveToNetZone);

            // Register the drive forward binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER ,
                    button ,
                    "Drive to Net Zone"
            ));
        }
    }

    private void driveToObservationZone(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {


            Command driveToObservationZoneCommand = new InstantCommand(() -> {
                driveToObservationZoneAction = new DriveToObservationZone();
                ActionCommand actionCommand = new ActionCommand(driveToObservationZoneAction ,
                        Collections.unmodifiableSet(new HashSet<>(Arrays.asList(
                                        robot.getDriveSubsystem() ,
                                        robot.getSpecimenArmSubsystem() ,
                                        robot.getSpecimenIntakeSubsystem()
                                )
                                )
                        )
                );
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
                    GamepadType.DRIVER ,
                    button ,
                    "Drive to Observation Zone"
            ));
        }
    }

    private void toggleFieldOrientedControl(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command enableFieldOriented = new InstantCommand(() -> robot.getDriveSubsystem().fieldOrientedControl = true);
            Command disableFieldOriented = new InstantCommand(() -> robot.getDriveSubsystem().fieldOrientedControl = false);

            driverGamePad.getGamepadButton(button)
                    .toggleWhenPressed(enableFieldOriented , disableFieldOriented);

            // Register the field-oriented toggle
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER ,
                    button ,
                    "Toggle field-oriented control."
            ));
        }
    }

    private void resetGyro(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command resetYawCommand = new InstantCommand(() -> {
                robot.getDriveSubsystem().getMecanumDrive().lazyImu.get().resetYaw();
                robot.getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField , MatchConfig.finalAllianceColor);
            });

            driverGamePad.getGamepadButton(button)
                    .whenPressed(resetYawCommand);

            // Register the yaw reset
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER ,
                    button ,
                    resetYawCommand ,
                    "Reset to Start Pose"
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
                    GamepadType.DRIVER ,
                    button ,
                    cycleDriveModeCommand ,
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
                GamepadType.DRIVER ,
                button ,
                cycleTelemetryModeCommand ,
                "Cycle telemetry"
        ));
    }

    private void bindSlowMode(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            driverGamePad.getGamepadButton(button)
                    .whenPressed(new InstantCommand(robot.getDriveSubsystem()::enableSlowMode))
                    .whenReleased(new InstantCommand(robot.getDriveSubsystem()::disableSlowMode)
                    );

            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    button,
                    "Toggle Slow Mode"
            ));
        }
    }

    private void bindDefaultDriving(DoubleSupplier leftY , DoubleSupplier leftX , DoubleSupplier rightX) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Command defaultDriveCommand = new DefaultDriveCommand(robot.getDriveSubsystem() ,
                    leftY ,
                    leftX ,
                    rightX);

            CommandScheduler.getInstance().setDefaultCommand(robot.getDriveSubsystem() , defaultDriveCommand);

            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.DRIVER ,
                    List.of("Ly" , "Lx" , "Rx") ,
                    "Drive/Strafe/Rotate"
            ));
        }
    }

    private void bindSpecimenArmIntakeAndScore(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM) &&
                robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            SpecimenButtonHandling specimenHandlingStateMachine = robot.getSpecimenButtonHandling();
            Command specimenHandlePressCommand = new InstantCommand(specimenHandlingStateMachine::onSpecimenHandleButtonPress);

            driverGamePad.getGamepadButton(button)
                    .whenPressed(specimenHandlePressCommand);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER ,
                    button ,
                    specimenHandlePressCommand ,
                    "Pickup/Score Specimen"
            ));
        }
    }
    private void bindBucketAngle(GamepadKeys.Trigger trigger) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            driveTo45or225 = new DriveAtFixedDegreeHeadingCommand(
                    robot.getDriveSubsystem(),
                    driverGamePad::getLeftY,
                    driverGamePad::getLeftX,
                    (MatchConfig.finalAllianceColor == BLUE) ? 225 : 45
            );
            fixedBucketScoreAngleTrigger = new TriggerReader(driverGamePad, trigger, 0.5);

            // Register for debugging/telemetry
            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.DRIVER,
                    Collections.singletonList(trigger.name()),
                    "Toggle 45/225 Mode"
            ));
        }
    }

    private void bindSpecimenAngleDriving(GamepadKeys.Trigger trigger) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            driveTo90OrMinus90 = new DriveAtFixedDegreeHeadingCommand(
                    robot.getDriveSubsystem(),
                    driverGamePad::getLeftY,
                    driverGamePad::getLeftX,
                    (MatchConfig.finalAllianceColor == BLUE) ? -90 : 90
            );
            fixedSpecimenScoreAngleTrigger = new TriggerReader(driverGamePad, trigger, 0.5);

            // Register for debugging/telemetry
            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.DRIVER,
                    Collections.singletonList(trigger.name()),
                    "Toggle -90/90 Mode"
            ));
        }
    }
    public void updateTriggerBindings() {
        if (fixedBucketScoreAngleTrigger!=null && fixedSpecimenScoreAngleTrigger!=null) {
            // Toggle 45/225 mode (left trigger)
            toggleTriggerCommand(fixedBucketScoreAngleTrigger, driveTo45or225, isBucketAngleCommandActive);
            isBucketAngleCommandActive = CommandScheduler.getInstance().isScheduled(driveTo45or225);

            // Toggle -90/90 mode (right trigger)
            toggleTriggerCommand(fixedSpecimenScoreAngleTrigger, driveTo90OrMinus90, isSpecimenAngleCommandActive);
            isSpecimenAngleCommandActive = CommandScheduler.getInstance().isScheduled(driveTo90OrMinus90);
        }
    }

    private void toggleTriggerCommand(TriggerReader triggerReader, Command command, boolean isActive) {
        triggerReader.readValue();
        if (triggerReader.wasJustPressed()) {
            if (isActive) {
                CommandScheduler.getInstance().cancel(command);
            } else {
                CommandScheduler.getInstance().schedule(command);
            }
        }
    }
}

