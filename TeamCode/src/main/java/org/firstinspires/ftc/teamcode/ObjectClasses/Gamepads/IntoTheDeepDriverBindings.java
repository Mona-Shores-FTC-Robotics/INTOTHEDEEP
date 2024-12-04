package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import static com.example.sharedconstants.FieldConstants.AllianceColor.BLUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.AnalogBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.ButtonBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamePadBindingManager;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamepadType;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveForwardAndBack;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveToNetZone;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveToObservationZone;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands.DriveAtFixedDegreeHeadingCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenButtonHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;
@Config
public class IntoTheDeepDriverBindings {
    private static final double RESET_POSE_DELAY_TIME_MILLISECONDS = 500;
    public double SLOW_TURN_SPEED = .5 ;
    GamepadEx driverGamePad;
    Robot robot;
    GamePadBindingManager bindingManager;
    DriveToObservationZone driveToObservationZoneAction;
    DriveToNetZone driveToNetZoneAction;
    DriveForwardAndBack driveForwardAndBack;

    public IntoTheDeepDriverBindings(GamepadEx gamePad , GamePadBindingManager gamePadBindingManager) {
        robot = Robot.getInstance();
        driverGamePad = gamePad;
        bindingManager = gamePadBindingManager;

        //Main Controls
        bindDefaultDriving(driverGamePad::getLeftY , driverGamePad::getLeftX , (driverGamePad::getRightX));
        bindSpecimenArmIntakeAndScore(GamepadKeys.Button.A);
        bindNitroMode(GamepadKeys.Button.LEFT_BUMPER);
        bindSlowMode(GamepadKeys.Button.RIGHT_BUMPER);
        driveToNetZone(GamepadKeys.Button.X);
        driveToObservationZone(GamepadKeys.Button.B);

        //Operator also has the ability to do this
        bindSpecimenIntakeToggle(GamepadKeys.Button.Y);

        //Buttons for if things go wrong
        resetGyro(GamepadKeys.Button.DPAD_DOWN);

        // Configuration Options
        cycleTelemetry(GamepadKeys.Button.BACK);
        toggleFieldOrientedControl(GamepadKeys.Button.START);

        //Drive Angle restriction
//        bindBucketAngle(GamepadKeys.Trigger.LEFT_TRIGGER);
//        bindSpecimenAngleDriving(GamepadKeys.Trigger.RIGHT_TRIGGER);

        turnSlowLeft(GamepadKeys.Trigger.LEFT_TRIGGER);
        turnSlowRight(GamepadKeys.Trigger.RIGHT_TRIGGER);

    }

    private void turnSlowRight(GamepadKeys.Trigger trigger) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Set<Subsystem> requirements = new HashSet<>();
            requirements.add(Robot.getInstance().getDriveSubsystem());

            DefaultDriveCommand slowRightTurn = new DefaultDriveCommand(
                    robot.getDriveSubsystem(),
                    driverGamePad::getLeftY,
                    driverGamePad::getLeftX,
                    ()->-driverGamePad.getTrigger(trigger)*SLOW_TURN_SPEED
            );

            // Trigger reader to detect when the trigger is pressed
            Trigger triggerDown = new Trigger(() -> driverGamePad.getTrigger(trigger) > 0.2);
            Trigger triggerUp = new Trigger(() -> driverGamePad.getTrigger(trigger) < 0.2);

            // Start the command once when the trigger is pressed
            triggerDown.whenActive(slowRightTurn);
            triggerUp.cancelWhenActive(slowRightTurn);

            // Register for debugging/telemetry
            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.DRIVER,
                    Collections.singletonList(trigger.name()),
                    "Slow Right Turn"
            ));
        }
    }


    private void turnSlowLeft(GamepadKeys.Trigger trigger) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            Set<Subsystem> requirements = new HashSet<>();
            requirements.add(Robot.getInstance().getDriveSubsystem());

            DefaultDriveCommand slowLeftTurn = new DefaultDriveCommand(
                    robot.getDriveSubsystem(),
                    driverGamePad::getLeftY,
                    driverGamePad::getLeftX,
                    ()-> driverGamePad.getTrigger(trigger)*SLOW_TURN_SPEED
            );

            // Trigger reader to detect when the trigger is pressed
            Trigger triggerDown = new Trigger(() -> driverGamePad.getTrigger(trigger) > 0.2);

            Trigger triggerUp = new Trigger(() -> driverGamePad.getTrigger(trigger) < 0.2);


            // Start the command once when the trigger is pressed
            triggerDown.whenActive(slowLeftTurn);
            triggerUp.cancelWhenActive(slowLeftTurn);
            // Register for debugging/telemetry
            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.DRIVER,
                    Collections.singletonList(trigger.name()),
                    "Slow Left Turn"
            ));
        }
    }

    private void bindNitroMode(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            driverGamePad.getGamepadButton(button)
                    .whenPressed(new InstantCommand(robot.getDriveSubsystem()::enableNitroMode))
                    .whenReleased(new InstantCommand(robot.getDriveSubsystem()::disableNitroMode)
                    );

            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    button,
                    "Hold for Nitro Mode"
            ));
        }
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
            ElapsedTime buttonPressTimer = new ElapsedTime(); // Timer to track button press duration

            // Command to reset the gyro and pose
            Command resetYawCommand = new InstantCommand(() -> {
                //this resets the onboard IMU, which we aren't using right now
                robot.getDriveSubsystem().getMecanumDrive().lazyImu.get().resetYaw();
                //this resets the pinpoint pose, which effectively resets the heading and pose to the corner
                robot.getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);
            });

            AtomicBoolean canTrigger = new AtomicBoolean(true); // Tracks if the button can trigger the command

            // Start the timer when the button is pressed
            driverGamePad.getGamepadButton(button)
                    .whenPressed(buttonPressTimer::reset)
                    .whileHeld(() -> {
                        if (canTrigger.get() && buttonPressTimer.milliseconds() >= RESET_POSE_DELAY_TIME_MILLISECONDS) {
                            resetYawCommand.schedule(); // Schedule the command
                            canTrigger.set(false);      // Block further triggers until button is released
                        }
                    })
                    .whenReleased(() -> canTrigger.set(true)); // Allow re-triggering on button release

            // Register the yaw reset with a description
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.DRIVER,
                    button,
                    resetYawCommand,
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
            // Command to drive the robot to a fixed heading
            DriveAtFixedDegreeHeadingCommand lockBasketAngle = new DriveAtFixedDegreeHeadingCommand(
                    robot.getDriveSubsystem(),
                    driverGamePad::getLeftY,
                    driverGamePad::getLeftX,
                    (MatchConfig.finalAllianceColor == BLUE) ? 225 : 45
            );

            // Trigger reader to detect when the trigger is pressed
            Trigger triggerDown = new Trigger(() -> driverGamePad.getTrigger(trigger) > 0.3);

            // While the trigger is held, execute the command to rotate to the fixed angle
            triggerDown.whileActiveOnce(lockBasketAngle);

            // Register for debugging/telemetry
            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.DRIVER,
                    Collections.singletonList(trigger.name()),
                    "Hold to Rotate to Fixed Angle"
            ));
        }
    }

    private void bindSpecimenAngleDriving(GamepadKeys.Trigger trigger) {
        if (robot.hasSubsystem(Robot.SubsystemType.DRIVE)) {
            // Command to drive the robot to -90 or 90 degrees based on alliance color
            DriveAtFixedDegreeHeadingCommand lockSpecimenPickupAngle = new DriveAtFixedDegreeHeadingCommand(
                    robot.getDriveSubsystem(),
                    driverGamePad::getLeftY,
                    driverGamePad::getLeftX,
                    (MatchConfig.finalAllianceColor == BLUE) ? 180 : 0
            );

            // Trigger reader to detect when the trigger is pressed
            Trigger triggerDown = new Trigger(() -> driverGamePad.getTrigger(trigger) > 0.5);

            // While the trigger is held, run the driveTo90OrMinus90 command
            triggerDown.whileActiveOnce(lockSpecimenPickupAngle);

            // Register for debugging/telemetry
            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.DRIVER,
                    Collections.singletonList(trigger.name()),
                    "Hold for 180/0 Mode"
            ));
        }
    }
}

