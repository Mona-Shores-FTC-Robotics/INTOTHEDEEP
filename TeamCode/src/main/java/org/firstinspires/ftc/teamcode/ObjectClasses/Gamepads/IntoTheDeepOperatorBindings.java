package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.AnalogBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.ButtonBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamePadBindingManager;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamepadType;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Climber.ChangeClimberMotorPowerCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Climber.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Climber.MoveClimberArmCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Climber.ClimberMotorHoldPositionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleButtonHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingActions.BetterPrepareAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingActions.PrepareToScoreInHighBasketAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingActions.ScoreSampleAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.DefaultSampleLiftCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.DefaultSampleLinearActuatorCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.ActionsAndCommands.DefaultSpecimenArmWithMotionProfileCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class IntoTheDeepOperatorBindings {
    GamepadEx operatorGamePad;
    Robot robot;
    GamePadBindingManager bindingManager;

    private TriggerReader increasePickupAngleTriggerReader;
    private TriggerReader decreasePickupAngleTriggerReader;

    public IntoTheDeepOperatorBindings(GamepadEx gamepad, GamePadBindingManager gamePadBindingManager) {
        robot = Robot.getInstance();
        operatorGamePad = gamepad;
        bindingManager = gamePadBindingManager;

        DepolySampleIntake(GamepadKeys.Button.A);
        BucketScoreSequence(GamepadKeys.Button.X);
        ManualLinearActuator(operatorGamePad::getRightY);
//        bindClimberMotorMovement(GamepadKeys.Button.RIGHT_BUMPER);
//        bindMoveClimberArm(GamepadKeys.Button.LEFT_BUMPER);

        //Manual controls for when things aren't working right
        ManualLift(operatorGamePad::getLeftX);
        MoveBucketServoArm(GamepadKeys.Button.DPAD_UP);
        ReverseSampleIntakeToggle(GamepadKeys.Button.DPAD_LEFT);
        ReverseSpecimenIntakeToggle(GamepadKeys.Button.DPAD_RIGHT);
        ManualDump(GamepadKeys.Button.DPAD_DOWN);
        bindDecreasePickupAngle(GamepadKeys.Trigger.LEFT_TRIGGER);
        bindIncreasePickupAngle(GamepadKeys.Trigger.RIGHT_TRIGGER);

        //Telemetry Cycling
        cycleTelemetry(GamepadKeys.Button.BACK);

        //Unbound
//        bindManualSpecimenArmMovement(operatorGamePad::getLeftY);
//        bindReadyForSampleScoring(GamepadKeys.Button.X);
//        bindScoreSample(GamepadKeys.Button.Y);

    }

    //Todo test this and see if its worth using...
    private void bindFlipArmToAngle(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM))
        {
            SpecimenArmSubsystem armSubsystem = Robot.getInstance().getSpecimenArmSubsystem();
            Command flipArmCommandNewWay = new InstantCommand(armSubsystem::rotateToCCWWithConstantPower);
            operatorGamePad.getGamepadButton(button)
                    .whenPressed(flipArmCommandNewWay);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    "Flip Arm To Angle"
            ));
        }
    }

    private void ReverseSpecimenIntakeToggle(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            SpecimenIntakeSubsystem intakeSubsystem = Robot.getInstance().getSpecimenIntakeSubsystem();
            Command turnIntakeOn = new InstantCommand(intakeSubsystem::reverseIntake);
            Command turnIntakeOff = new InstantCommand(intakeSubsystem::turnOffIntake);

            operatorGamePad.getGamepadButton(button)
                    .toggleWhenPressed(turnIntakeOn, turnIntakeOff);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    "Reverse Specimen Intake"
            ));
        }
    }

    private void ReverseSampleIntakeToggle(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
            SampleIntakeSubsystem intakeSubsystem = Robot.getInstance().getSampleIntakeSubsystem();
            Command turnIntakeOn = new InstantCommand(intakeSubsystem::reverseIntake);
            Command turnIntakeOff = new InstantCommand(intakeSubsystem::turnOffIntake);

            operatorGamePad.getGamepadButton(button)
                    .toggleWhenPressed(turnIntakeOn, turnIntakeOff);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    "Reverse Sample Intake"
            ));
        }
    }

    private void MoveBucketServoArm(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            SampleButtonHandling sampleHandlingStateMachine = robot.getSampleButtonHandling();
            Command moveSampleBucket = new InstantCommand(sampleHandlingStateMachine::onMoveSampleBucketButtonPress);
            operatorGamePad.getGamepadButton(button)
                    .whenPressed(moveSampleBucket);
            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    moveSampleBucket,
                    "Move Bucket"
            ));
        }
    }

    private void ManualDump(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            Command dumpSample = new InstantCommand(Robot.getInstance().getSampleLiftBucketSubsystem()::dumpSampleInBucket);

            operatorGamePad.getGamepadButton(button)
                    .whenPressed(dumpSample);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    dumpSample,
                    "Dump"
            ));
        }
    }

    private void cycleTelemetry(GamepadKeys.Button button) {
        // Command to cycle telemetry modes using DriverStationTelemetryManager
        Command cycleTelemetryModeCommand = new InstantCommand(robot.getDriverStationTelemetryManager()::cycleTelemetryMode);

        operatorGamePad.getGamepadButton(button)
                .whenPressed(cycleTelemetryModeCommand);

        bindingManager.registerBinding(new ButtonBinding(
                GamepadType.OPERATOR,
                button,
                cycleTelemetryModeCommand,
                "Cycle telemetry"
        ));
    }



    private void bindReadyForSampleScoring(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            Set<Subsystem> requirements = new HashSet<>();
            requirements.add(Robot.getInstance().getSampleLiftBucketSubsystem());

            Command prepareToScoreSample = new ActionCommand(new PrepareToScoreInHighBasketAction(),requirements);

            operatorGamePad.getGamepadButton(button)
                    .whenPressed( prepareToScoreSample);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    "Lift Sample to Bucket"
            ));
        }
    }

    private void bindScoreSample(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            Set<Subsystem> requirements = new HashSet<>();
            requirements.add(Robot.getInstance().getSampleLiftBucketSubsystem());
            requirements.add(Robot.getInstance().getDriveSubsystem());

            operatorGamePad.getGamepadButton(button)
                    .whenPressed(() -> {
                        // Define a new SequentialAction each time the button is pressed
                        ScoreSampleAction ScoreSampleAction = new ScoreSampleAction();
                        // Wrap the SequentialAction in an ActionCommand and schedule it
                        Command scoreSample = new ActionCommand(ScoreSampleAction, requirements);
                        scoreSample.schedule();
                    });

            // Register button binding for debugging or tracking purposes
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    "Score Sample->Drive->Lower Lift"
            ));
        }
    }


    private void BucketScoreSequence(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            Set<Subsystem> requirements = new HashSet<>();
            requirements.add(Robot.getInstance().getSampleLiftBucketSubsystem());
            requirements.add(Robot.getInstance().getDriveSubsystem());

            operatorGamePad.getGamepadButton(button)
                    .whenPressed(()->
                    {
                        BetterPrepareAction betterPrepareAction = new BetterPrepareAction(-10);
                        Command prepareToScoreSampleCommand = new ActionCommand(betterPrepareAction,requirements);
                        prepareToScoreSampleCommand.schedule();
                    })
                    .whenReleased(() -> {
                        // Define a new SequentialAction each time the button is pressed
                        ScoreSampleAction scoreSampleAction = new ScoreSampleAction();
                        // Wrap the SequentialAction in an ActionCommand and schedule it
                        Command scoreSample = new ActionCommand(scoreSampleAction, requirements);
                        scoreSample.schedule();
                    });

            // Register button binding for debugging or tracking purposes
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    "Press (raise and drive), Release (score and drive)"
            ));
        }
    }



    private void DepolySampleIntake(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR))
        {
            SampleButtonHandling sampleHandlingStateMachine = robot.getSampleButtonHandling();
            Command sampleIntakeButtonPressCommand = new InstantCommand(sampleHandlingStateMachine::onIntakeButtonPress);

            operatorGamePad.getGamepadButton(button)
                    .whenPressed(sampleIntakeButtonPressCommand);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    sampleIntakeButtonPressCommand,
                    "Move Sample Actuator"
            ));
        }
    }

    private void ManualLift(DoubleSupplier analogInput) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            Command defaultSampleLiftCommand = new DefaultSampleLiftCommand(
                    robot.getSampleLiftBucketSubsystem(),
                    analogInput
            );
            CommandScheduler.getInstance().setDefaultCommand(robot.getSampleLiftBucketSubsystem(), defaultSampleLiftCommand);

            // Register the default sample lift command (no specific button)
            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.OPERATOR,
                    "Right Y",
                    "Manual Sample Lift"
            ));
        }
    }
    private void ManualLinearActuator(DoubleSupplier analogInput){
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
            Command defaultSampleLinearActuatorCommand = new DefaultSampleLinearActuatorCommand(
                    robot.getSampleLinearActuatorSubsystem(),
                    robot.getSampleIntakeSubsystem(),
                    analogInput // Use the dynamically passed joystick input
            );

            CommandScheduler.getInstance().setDefaultCommand(robot.getSampleLinearActuatorSubsystem(), defaultSampleLinearActuatorCommand);

            // Register the default specimen arm command (no specific button)
            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.OPERATOR,
                    "Right X", // No specific button
                    "Manual Sample Linear Actuator"
            ));
        }
    }
    private void bindManualSpecimenArmMovement (DoubleSupplier analogInput){
        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            Command defaultSpecimenArmCommand = new DefaultSpecimenArmWithMotionProfileCommand(
                    robot.getSpecimenArmSubsystem(),
                    analogInput
            );

            CommandScheduler.getInstance().setDefaultCommand(robot.getSpecimenArmSubsystem(), defaultSpecimenArmCommand);

            // Register the default specimen arm command (no specific button)
            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.OPERATOR,
                    "Left Y",
                    "Manual Specimen Arm"
            ));
        }
    }

    private void bindClimberMotorMovement(GamepadKeys.Button button) {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.CLIMBER))
        {
            operatorGamePad.getGamepadButton(button)
                    .whenPressed(new ChangeClimberMotorPowerCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberMotorStates.ROBOT_UP))
                    .whenReleased(new ClimberMotorHoldPositionCommand((Robot.getInstance().getClimberSubsystem()))
                    );

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    "Climb Robot While Pressed"
            ));
        }
    }

    private void bindMoveClimberArm (GamepadKeys.Button button) {
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.CLIMBER))
        {
            SequentialCommandGroup resetArm = new SequentialCommandGroup(
                    new MoveClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.STOWED_STEP1),
                    new WaitCommand(350),
                    new MoveClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.STOWED_STEP2),
                    new WaitCommand(150),
                    new MoveClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.STOWED_STEP3));


            operatorGamePad.getGamepadButton(button)
                    .toggleWhenPressed(
                            new MoveClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.READY),
                            resetArm
                    );

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    "Move Climber Arm"
            ));
        }
    }


    private void bindIncreasePickupAngle(GamepadKeys.Trigger trigger) {
        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            increasePickupAngleTriggerReader = new TriggerReader(operatorGamePad, trigger, 0.5);

            // Register for debugging/telemetry
            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.OPERATOR,
                    Collections.singletonList(trigger.name()),
                    "Increase Pickup Angle"
            ));
        }
    }

    private void bindDecreasePickupAngle(GamepadKeys.Trigger trigger) {
        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            decreasePickupAngleTriggerReader = new TriggerReader(operatorGamePad, trigger, 0.5);

            // Register for debugging/telemetry
            bindingManager.registerBinding(new AnalogBinding(
                    GamepadType.OPERATOR,
                    Collections.singletonList(trigger.name()),
                    "Decrease Pickup Angle"
            ));
        }
    }

    public void updateTriggerBindings() {
        if (decreasePickupAngleTriggerReader!=null && increasePickupAngleTriggerReader!=null) {
            // Toggle 45/225 mode (left trigger)
            Command increasePickupAngle  = new InstantCommand(robot.getSpecimenArmSubsystem()::increasePickupAngle, robot.getSpecimenArmSubsystem());
            Command decreasePickupAngle  = new InstantCommand(robot.getSpecimenArmSubsystem()::decreasePickupAngle, robot.getSpecimenArmSubsystem());
            if (decreasePickupAngleTriggerReader.wasJustPressed())
            {
                decreasePickupAngle.schedule();
            }
            if (increasePickupAngleTriggerReader.wasJustPressed())
            {
                increasePickupAngle.schedule();
            }
        }
    }
}
