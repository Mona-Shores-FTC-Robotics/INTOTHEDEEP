package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.AnalogBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.ButtonBinding;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamePadBindingManager;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamepadType;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingStateMachine;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.DefaultSampleLiftCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.DefaultSampleLinearActuatorCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.ActionsAndCommands.DefaultSpecimenArmWithMotionProfileCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenHandlingStateMachine;
import java.util.function.DoubleSupplier;

public class IntoTheDeepOperatorBindings {
    GamepadEx operatorGamePad;
    Robot robot;
    GamePadBindingManager bindingManager;

    public IntoTheDeepOperatorBindings(GamepadEx gamepad, GamePadBindingManager gamePadBindingManager) {
        robot = Robot.getInstance();
        operatorGamePad = gamepad;
        bindingManager = gamePadBindingManager;

        //////////////////////////////////////////////////////////
        // LEFT STICK - Y axis - Manually Move SpecimenArm      //
        //////////////////////////////////////////////////////////
        bindManualSpecimenArmMovement(operatorGamePad::getLeftY);

        //////////////////////////////////////////////////////////
        // RIGHT STICK - X axis - Manually Move SampleIntake    //
        //////////////////////////////////////////////////////////
        bindManualSampleActuatorMovement(operatorGamePad::getRightX);

        //////////////////////////////////////////////////////////
        // RIGHT STICK - Y axis - Manually Move SampleIntake     //
        //////////////////////////////////////////////////////////
        bindManualLiftMovement(operatorGamePad::getRightY);

        //////////////////////////////////////////////////////////
        // LEFT BUMPER - Cycle Telemetry                        //
        //////////////////////////////////////////////////////////
        cycleTelemetry(GamepadKeys.Button.LEFT_BUMPER);

        //////////////////////////////////////////////////////////
        // Y BUTTON - Specimen Handling (Intake and Scoring)    //
        //////////////////////////////////////////////////////////
        bindSpecimenHandle(GamepadKeys.Button.Y);

        //////////////////////////////////////////////////////////
        // A BUTTON - Sample Intake (Automated Handoff)         //
        //////////////////////////////////////////////////////////
        bindSampleIntake(GamepadKeys.Button.A);

        //////////////////////////////////////////////////////////
        // B BUTTON - Handle Sample Scoring                     //
        //////////////////////////////////////////////////////////
        bindSampleScoring(GamepadKeys.Button.B);

        //Not bound to anything
        //////////////////////////////////////////////////////////
        // X BUTTON                                             //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        // RIGHT BUMPER                                         //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        // DPAD-UP                                              //
        //////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////
        // DPAD-LEFT                                            //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        // DPAD-RIGHT                                           //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        // DPAD-DOWN                                              //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        // BACK/SHARE BUTTON                                    //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        // START/OPTIONS BUTTON                                 //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        // RIGHT TRIGGER                                        //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        // LEFT TRIGGER                                         //
        //////////////////////////////////////////////////////////
    }

    private void bindBucket(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            SampleHandlingStateMachine sampleHandlingStateMachine = robot.getSampleHandlingStateMachine();
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

    private void bindDumping(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            SampleHandlingStateMachine sampleHandlingStateMachine = robot.getSampleHandlingStateMachine();
            Command moveSampleDumper = new InstantCommand(sampleHandlingStateMachine::onMoveSampleDumperButtonPress);

            operatorGamePad.getGamepadButton(button)
                    .whenPressed(moveSampleDumper);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    moveSampleDumper,
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
    private void bindSampleScoring(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)) {
            SampleHandlingStateMachine sampleHandlingStateMachine = robot.getSampleHandlingStateMachine();
            Command sampleScorePressCommand = new InstantCommand(sampleHandlingStateMachine::onScoreButtonPress);

            operatorGamePad.getGamepadButton(button)
                    .whenPressed(sampleScorePressCommand);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    sampleScorePressCommand,
                    "Score Sample"
            ));
        }
    }
    private void bindSampleIntake(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR))
        {
            SampleHandlingStateMachine sampleHandlingStateMachine = robot.getSampleHandlingStateMachine();
            Command sampleIntakeButtonPressCommand = new InstantCommand(sampleHandlingStateMachine::onIntakeButtonPressCommand);

            operatorGamePad.getGamepadButton(button)
                    .whenPressed(sampleIntakeButtonPressCommand);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    sampleIntakeButtonPressCommand,
                    "Intake Sample"
            ));
        }
    }
    private void bindSpecimenHandle(GamepadKeys.Button button) {
        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM) &&
                robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            SpecimenHandlingStateMachine specimenHandlingStateMachine = robot.getSpecimenHandlingStateMachine();
            Command specimenHandlePressCommand = new InstantCommand(specimenHandlingStateMachine::onSpecimenHandleButtonPress);

            operatorGamePad.getGamepadButton(button)
                    .whenPressed(specimenHandlePressCommand);

            // Register button binding
            bindingManager.registerBinding(new ButtonBinding(
                    GamepadType.OPERATOR,
                    button,
                    specimenHandlePressCommand,
                    "Handle Specimen"
            ));
        }
    }
    private void bindManualLiftMovement(DoubleSupplier analogInput) {
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
    private void bindManualSampleActuatorMovement (DoubleSupplier analogInput){
            if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
                Command defaultSampleLinearActuatorCommand = new DefaultSampleLinearActuatorCommand(
                        robot.getSampleLinearActuatorSubsystem(),
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
}
