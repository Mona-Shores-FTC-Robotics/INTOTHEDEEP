package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingStateMachine;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.ChangeSampleIntakePowerAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.DefaultSampleLiftCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.MoveSampleLiftAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.DefaultSampleLinearActuatorCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm.DefaultSpecimenArmCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm.MoveSpecimenArmAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenHandlingStateMachine;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.ChangeSpecimenIntakePowerAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

import java.util.Collections;
import java.util.Set;

public class IntoTheDeepOperatorBindings {
    public Command cycleTelemetryModeCommand;

    public static TriggerReader rightTrigger;
    public static TriggerReader leftTrigger;

    public IntoTheDeepOperatorBindings(GamepadEx operatorGamepad) {
        Robot robot = Robot.getInstance();
        ButtonBindingManager bindingManager = ButtonBindingManager.getInstance();

        //////////////////////////////////////////////////////////
        // LEFT STICK                                           //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            Command defaultSampleLiftCommand = new DefaultSampleLiftCommand(
                    robot.getSampleLiftBucketSubsystem(),
                    operatorGamepad::getLeftY
            );

            CommandScheduler.getInstance().setDefaultCommand(robot.getSampleLiftBucketSubsystem(), defaultSampleLiftCommand);

            // Register the default sample lift command (no specific button)
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    null, // No specific button
                    defaultSampleLiftCommand,
                    "Default Sample Lift Control with left stick."
            ));
        } else if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            Command defaultSpecimenArmCommand = new DefaultSpecimenArmCommand(
                    robot.getSpecimenArmSubsystem(),
                    operatorGamepad::getLeftY
            );

            CommandScheduler.getInstance().setDefaultCommand(robot.getSpecimenArmSubsystem(), defaultSpecimenArmCommand);

            // Register the default specimen arm command (no specific button)
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    null, // No specific button
                    defaultSpecimenArmCommand,
                    "Default Specimen Arm Control with left stick."
            ));
        }

        //////////////////////////////////////////////////////////
        // RIGHT STICK                                          //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
            Command defaultSampleLinearActuatorCommand = new DefaultSampleLinearActuatorCommand(
                    robot.getSampleLinearActuatorSubsystem(),
                    operatorGamepad::getRightY
            );
            CommandScheduler.getInstance().setDefaultCommand(robot.getSampleLinearActuatorSubsystem(), defaultSampleLinearActuatorCommand);

            // Register the default sample linear actuator command (no specific button)
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    null, // No specific button
                    defaultSampleLinearActuatorCommand,
                    "Default Sample Linear Actuator Control with right stick."
            ));
        }

        //////////////////////////////////////////////////////////
        // DPAD-UP -                                            //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            SpecimenArmSubsystem specimenArmSubsystem = robot.getSpecimenArmSubsystem();
            Set<Subsystem> specimenArmRequirements = Collections.singleton(specimenArmSubsystem);

            Command moveSpecimenDeliveryCommand = new ActionCommand(
                    new MoveSpecimenArmAction(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_DELIVERY),
                    specimenArmRequirements
            );

            operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenPressed(moveSpecimenDeliveryCommand);

            // Register DPAD_UP binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.DPAD_UP,
                    moveSpecimenDeliveryCommand,
                    "Move Specimen Arm to Delivery Position."
            ));
        } else if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
            SampleLinearActuatorSubsystem sampleLinearActuatorSubsystem = robot.getSampleLinearActuatorSubsystem();
            Command runActuatorForwardCommand = new InstantCommand(sampleLinearActuatorSubsystem::runWithoutEncodersForward);
            Command stopActuatorCommand = new InstantCommand(sampleLinearActuatorSubsystem::stopActuator);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenPressed(runActuatorForwardCommand)
                    .whenReleased(stopActuatorCommand);

            // Register DPAD_UP binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.DPAD_UP,
                    runActuatorForwardCommand, // Primary command
                    "Run Sample Linear Actuator Forward (DPAD_UP)."
            ));
        }

        //////////////////////////////////////////////////////////
        // DPAD-RIGHT                                           //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
            SampleIntakeSubsystem sampleIntakeSubsystem = robot.getSampleIntakeSubsystem();
            Set<Subsystem> sampleIntakeRequirements = Collections.singleton(sampleIntakeSubsystem);

            ChangeSampleIntakePowerAction turnOffIntake = new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
            ChangeSampleIntakePowerAction turnOnIntake = new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);

            Command turnOnIntakeCommand = new ActionCommand(turnOnIntake, sampleIntakeRequirements);
            Command turnOffIntakeCommand = new ActionCommand(turnOffIntake, sampleIntakeRequirements);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                    .whenPressed(turnOnIntakeCommand)
                    .whenReleased(turnOffIntakeCommand);

            // Register DPAD_RIGHT (mapped to X button) binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.X,
                    turnOnIntakeCommand,
                    "Activate Sample Intake (Press X)."
            ));
        }

        //////////////////////////////////////////////////////////
        // DPAD-LEFT -                                          //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            SpecimenArmSubsystem specimenArmSubsystem = robot.getSpecimenArmSubsystem();
            Set<Subsystem> specimenArmRequirements = Collections.singleton(specimenArmSubsystem);

            Command moveSpecimenStagingCommand = new ActionCommand(
                    new MoveSpecimenArmAction(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_STAGING),
                    specimenArmRequirements
            );

            operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                    .whenPressed(moveSpecimenStagingCommand);

            // Register DPAD_LEFT binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.DPAD_LEFT,
                    moveSpecimenStagingCommand,
                    "Move Specimen Arm to Staging Position."
            ));
        }

        //////////////////////////////////////////////////////////
        // DPAD-DOWN -                                          //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            SpecimenArmSubsystem specimenArmSubsystem = robot.getSpecimenArmSubsystem();
            Set<Subsystem> specimenArmRequirements = Collections.singleton(specimenArmSubsystem);

            Command moveSpecimenPickupCommand = new ActionCommand(
                    new MoveSpecimenArmAction(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_PICKUP),
                    specimenArmRequirements
            );

            operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(moveSpecimenPickupCommand);

            // Register DPAD_DOWN binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.DPAD_DOWN,
                    moveSpecimenPickupCommand,
                    "Move Specimen Arm to Pickup Position."
            ));
        } else if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
            SampleLinearActuatorSubsystem sampleLinearActuatorSubsystem = robot.getSampleLinearActuatorSubsystem();
            Command runActuatorReverseCommand = new InstantCommand(sampleLinearActuatorSubsystem::runWithoutEncodersReverse);
            Command stopActuatorCommand = new InstantCommand(sampleLinearActuatorSubsystem::stopActuator);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(runActuatorReverseCommand)
                    .whenReleased(stopActuatorCommand);

            // Register DPAD_DOWN binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.DPAD_DOWN,
                    runActuatorReverseCommand, // Primary command
                    "Run Sample Linear Actuator Reverse (DPAD_DOWN)."
            ));
        }

        //////////////////////////////////////////////////////////
        // LEFT BUMPER                                          //
        //////////////////////////////////////////////////////////

        // Uncomment and implement if needed
        /*
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT)) {
            SampleLiftSubsystem sampleLiftSubsystem = robot.getSampleLiftSubsystem();
            Set<Subsystem> sampleLiftRequirements = Collections.singleton(sampleLiftSubsystem);

            Command moveSampleLiftLowCommand = new ActionCommand(
                new MoveSampleLiftAction(SampleLiftSubsystem.SampleLiftStates.LOW_BASKET),
                sampleLiftRequirements
            );

            operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(moveSampleLiftLowCommand);

            // Register LEFT_BUMPER binding
            bindingManager.registerBinding(new ButtonBinding(
                GamepadType.OPERATOR,
                GamepadKeys.Button.LEFT_BUMPER,
                moveSampleLiftLowCommand,
                "Move Sample Lift to Low Basket Position."
            ));
        }
        */

        // Command to cycle telemetry modes using DriverStationTelemetryManager
        cycleTelemetryModeCommand = new InstantCommand(() -> {
            Robot.getInstance().getDriverStationTelemetryManager().cycleTelemetryMode();
        });

        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(cycleTelemetryModeCommand);

        // Register LEFT_BUMPER binding
        bindingManager.registerBinding(new ButtonBinding(
                GamepadType.OPERATOR,
                GamepadKeys.Button.LEFT_BUMPER,
                cycleTelemetryModeCommand,
                "Cycle through telemetry display modes."
        ));

        //////////////////////////////////////////////////////////
        // RIGHT BUMPER - GO TO HIGH BASKET                     //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            SampleLiftBucketSubsystem sampleLiftBucketSubsystem = robot.getSampleLiftBucketSubsystem();
            Set<Subsystem> sampleLiftRequirements = Collections.singleton(sampleLiftBucketSubsystem);

            Command moveLiftHighBasketCommand = new ActionCommand(
                    new MoveSampleLiftAction(SampleLiftBucketSubsystem.SampleLiftStates.HIGH_BASKET),
                    sampleLiftRequirements
            );

            operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(moveLiftHighBasketCommand);

            // Register RIGHT_BUMPER binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.RIGHT_BUMPER,
                    moveLiftHighBasketCommand,
                    "Move Sample Lift to High Basket Position."
            ));
        }

        //////////////////////////////////////////////////////////
        // X BUTTON - SAMPLE INTAKE                             //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
            SampleIntakeSubsystem sampleIntakeSubsystem = robot.getSampleIntakeSubsystem();
            Set<Subsystem> sampleIntakeRequirements = Collections.singleton(sampleIntakeSubsystem);

            ChangeSampleIntakePowerAction turnOffIntake = new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
            ChangeSampleIntakePowerAction turnOnIntake = new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);

            Command turnOnIntakeCommand = new ActionCommand(turnOnIntake, sampleIntakeRequirements);
            Command turnOffIntakeCommand = new ActionCommand(turnOffIntake, sampleIntakeRequirements);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                    .whenPressed(turnOnIntakeCommand)
                    .whenReleased(turnOffIntakeCommand);

            // Register X button binding for SAMPLE_INTAKE
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.X,
                    turnOnIntakeCommand,
                    "Activate Sample Intake (Press X)."
            ));
        }

        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            SpecimenIntakeSubsystem specimenIntakeSubsystem = robot.getSpecimenIntakeSubsystem();
            Set<Subsystem> specimenIntakeRequirements = Collections.singleton(specimenIntakeSubsystem);

            ChangeSpecimenIntakePowerAction turnOffSpecimenIntake = new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
            ChangeSpecimenIntakePowerAction turnOnSpecimenIntake = new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_ON);

            Command turnOnSpecimenIntakeCommand = new ActionCommand(turnOnSpecimenIntake, specimenIntakeRequirements);
            Command turnOffSpecimenIntakeCommand = new ActionCommand(turnOffSpecimenIntake, specimenIntakeRequirements);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                    .whenPressed(turnOnSpecimenIntakeCommand)
                    .whenReleased(turnOffSpecimenIntakeCommand);

            // Register X button binding for SPECIMEN_INTAKE
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.X,
                    turnOnSpecimenIntakeCommand,
                    "Activate Specimen Intake (Press X)."
            ));
        }

        //////////////////////////////////////////////////////////
        // B BUTTON - SAMPLE REVERSE INTAKE                      //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
            Set<Subsystem> sampleIntakeRequirements = Collections.singleton(robot.getSampleIntakeSubsystem());

            ChangeSampleIntakePowerAction reverseIntake = new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_REVERSE);
            ChangeSampleIntakePowerAction turnOffIntake = new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);

            Command reverseIntakeCommand = new ActionCommand(reverseIntake, sampleIntakeRequirements);
            Command stopIntakeCommand = new ActionCommand(turnOffIntake, sampleIntakeRequirements);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                    .whenPressed(reverseIntakeCommand)
                    .whenReleased(stopIntakeCommand);

            // Register B button binding for SAMPLE_INTAKE
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.B,
                    reverseIntakeCommand,
                    "Reverse Sample Intake (Press B)."
            ));
        }

        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            Set<Subsystem> specimenIntakeRequirements = Collections.singleton(robot.getSpecimenIntakeSubsystem());

            ChangeSpecimenIntakePowerAction reverseSpecimenIntake = new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_REVERSE);
            ChangeSpecimenIntakePowerAction turnOffSpecimenIntake = new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);

            Command reverseSpecimenIntakeCommand = new ActionCommand(reverseSpecimenIntake, specimenIntakeRequirements);
            Command stopSpecimenIntakeCommand = new ActionCommand(turnOffSpecimenIntake, specimenIntakeRequirements);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                    .whenPressed(reverseSpecimenIntakeCommand)
                    .whenReleased(stopSpecimenIntakeCommand);

            // Register B button binding for SPECIMEN_INTAKE
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.B,
                    reverseSpecimenIntakeCommand,
                    "Reverse Specimen Intake (Press B)."
            ));
        }

        //////////////////////////////////////////////////////////
        // Y BUTTON                                            //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM) &&
                robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            SpecimenHandlingStateMachine specimenHandlingStateMachine = robot.getSpecimenHandlingStateMachine();

            Command specimenHandlePressCommand = new InstantCommand(specimenHandlingStateMachine::onSpecimenHandleButtonPress);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                    .whenPressed(specimenHandlePressCommand);

            // Register Y button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.Y,
                    specimenHandlePressCommand,
                    "Handle Specimen Action (Press Y)."
            ));
        }

        //////////////////////////////////////////////////////////
        // A BUTTON - RETRACTS AND DEPLOYS SAMPLES              //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) &&
                robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
            SampleHandlingStateMachine sampleHandlingStateMachine = robot.getSampleHandlingStateMachine();

            Command intakeButtonPressCommand = new InstantCommand(sampleHandlingStateMachine::onIntakeButtonPress);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                    .whenPressed(intakeButtonPressCommand);

            // Register A button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    GamepadKeys.Button.A,
                    intakeButtonPressCommand,
                    "Intake Sample (Press A)."
            ));
        }

        //////////////////////////////////////////////////////////
        // LEFT TRIGGER                                        //
        //////////////////////////////////////////////////////////
        // Implement bindings as needed
        // Example:
        /*
        if (robot.hasSubsystem(Robot.SubsystemType.SOME_SUBSYSTEM)) {
            Command someTriggerCommand = new SomeCommand();
            operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_TRIGGER)
                .whenHeld(someTriggerCommand);

            bindingManager.registerBinding(new ButtonBinding(
                GamepadType.OPERATOR,
                GamepadKeys.Button.LEFT_TRIGGER,
                someTriggerCommand,
                "Activate Some Subsystem with Left Trigger."
            ));
        }
        */

        //////////////////////////////////////////////////////////
        // RIGHT TRIGGER                                       //
        //////////////////////////////////////////////////////////
        // Implement bindings as needed
        // Example:
        /*
        if (robot.hasSubsystem(Robot.SubsystemType.SOME_OTHER_SUBSYSTEM)) {
            Command anotherTriggerCommand = new AnotherCommand();
            operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_TRIGGER)
                .whenHeld(anotherTriggerCommand);

            bindingManager.registerBinding(new ButtonBinding(
                GamepadType.OPERATOR,
                GamepadKeys.Button.RIGHT_TRIGGER,
                anotherTriggerCommand,
                "Activate Another Subsystem with Right Trigger."
            ));
        }
        */

        //////////////////////////////////////////////////////////
        // OPTIONS BUTTON                                      //
        //////////////////////////////////////////////////////////
        // Implement bindings as needed

        //////////////////////////////////////////////////////////
        // START BUTTON                                        //
        //////////////////////////////////////////////////////////
        // Implement bindings as needed
    }
}
