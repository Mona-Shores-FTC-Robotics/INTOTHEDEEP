package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

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
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.SampleLiftSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.DefaultSampleLinearActuatorCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm.DefaultSpecimenArmCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm.MoveSpecimenArmAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenHandlingStateMachine;

import java.util.Collections;
import java.util.Set;

public class IntoTheDeepOperatorBindings {
    public Command cycleTelemetryModeCommand;

    public static TriggerReader rightTrigger;
    public static TriggerReader leftTrigger;

    public IntoTheDeepOperatorBindings(GamepadEx operatorGamepad) {
        Robot robot = Robot.getInstance();

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT STICK / RIGHT STICK                             //
        //                                                      //
        //////////////////////////////////////////////////////////

//        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT)) {
//
//            Command defaultSampleLiftCommand = new DefaultSampleLiftCommand(Robot.getInstance().getSampleLiftSubsystem(),
//                    operatorGamepad::getLeftY);
//
//            CommandScheduler.getInstance().setDefaultCommand(Robot.getInstance().getSampleLiftSubsystem(), defaultSampleLiftCommand);
//        }

        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            Command defaultSpecimenArmCommand = new DefaultSpecimenArmCommand(Robot.getInstance().getSpecimenArmSubsystem(),
                    operatorGamepad::getLeftY);

            CommandScheduler.getInstance().setDefaultCommand(Robot.getInstance().getSpecimenArmSubsystem(), defaultSpecimenArmCommand);
        }

        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {

            Command defaultSampleLinearActuatorCommand = new DefaultSampleLinearActuatorCommand(Robot.getInstance().getSampleLinearActuatorSubsystem(),
                    operatorGamepad::getRightY);

            CommandScheduler.getInstance().setDefaultCommand(Robot.getInstance().getSampleLinearActuatorSubsystem(), defaultSampleLinearActuatorCommand);
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-UP -                                            //
        //                                                      //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            SpecimenArmSubsystem specimenArmSubsystem = Robot.getInstance().getSpecimenArmSubsystem();
            Set<Subsystem> specimenArmRequirements = Collections.singleton(specimenArmSubsystem);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenPressed(new ActionCommand(new MoveSpecimenArmAction(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_DELIVERY), specimenArmRequirements));
        }
        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-RIGHT -                                         //
        //                                                      //
        //////////////////////////////////////////////////////////



        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-LEFT -                                          //
        //                                                      //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            SpecimenArmSubsystem specimenArmSubsystem = Robot.getInstance().getSpecimenArmSubsystem();
            Set<Subsystem> specimenArmRequirements = Collections.singleton(specimenArmSubsystem);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                    .whenPressed(new ActionCommand(new MoveSpecimenArmAction(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_STAGING), specimenArmRequirements));
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-DOWN -                                          //
        //                                                      //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
            SpecimenArmSubsystem specimenArmSubsystem = Robot.getInstance().getSpecimenArmSubsystem();
            Set<Subsystem> specimenArmRequirements = Collections.singleton(specimenArmSubsystem);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(new ActionCommand(new MoveSpecimenArmAction(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_PICKUP), specimenArmRequirements));
        }
        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT BUMPER                                          //
        //                                                      //
        //////////////////////////////////////////////////////////
//
//        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT)) {
//            SampleLiftSubsystem sampleLiftSubsystem = Robot.getInstance().getSampleLiftSubsystem();
//            Set<Subsystem> sampleLiftRequirements = Collections.singleton(sampleLiftSubsystem);
//
//            operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                    .whenPressed(() -> new ActionCommand(new MoveSampleLiftAction(SampleLiftSubsystem.SampleLiftStates.LOW_BASKET), sampleLiftRequirements));
//        }

        // Command to cycle telemetry modes using DriverStationTelemetryManager
        cycleTelemetryModeCommand = new InstantCommand(() -> {
            Robot.getInstance().getDriverStationTelemetryManager().cycleTelemetryMode();
        });

        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(cycleTelemetryModeCommand);

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT BUMPER - GO TO HIGH BASKET                     //
        //                                                      //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT)) {
            SampleLiftSubsystem sampleLiftSubsystem = Robot.getInstance().getSampleLiftSubsystem();
            Set<Subsystem> sampleLiftRequirements = Collections.singleton(sampleLiftSubsystem);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(()-> new ActionCommand(new MoveSampleLiftAction(SampleLiftSubsystem.SampleLiftStates.HIGH_BASKET), sampleLiftRequirements));
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON - SAMPLE INTAKE                                   //
        //                                                      //
        //////////////////////////////////////////////////////////

        // Only bind this button if we actually have the subsystems
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE))
        {
            SampleIntakeSubsystem sampleIntakeSubsystem = robot.getSampleIntakeSubsystem();
            Set<Subsystem> sampleIntakeRequirements = Collections.singleton(sampleIntakeSubsystem);

            ChangeSampleIntakePowerAction turnOffIntake = new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
            ChangeSampleIntakePowerAction turnOnIntake = new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                    .whenPressed(new ActionCommand(turnOnIntake, sampleIntakeRequirements))
                    .whenReleased(new ActionCommand(turnOffIntake, sampleIntakeRequirements)
                    );
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON - SAMPLE REVERSE INTAKE                    //
        //                                                      //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
            Set<Subsystem> sampleIntakeRequirements = Collections.singleton(Robot.getInstance().getSampleIntakeSubsystem());

            ChangeSampleIntakePowerAction reverseIntake = new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_REVERSE);
            ChangeSampleIntakePowerAction turnOffIntake = new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                    .whenPressed(new ActionCommand(reverseIntake, sampleIntakeRequirements))
                    .whenReleased(new ActionCommand(turnOffIntake, sampleIntakeRequirements)
                    );
        }
        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

        // this button can only be bound if we actually have all the subsystems we need
//        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) &&
//                robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT) &&
//                robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
//            SampleHandlingStateMachine sampleHandlingStateMachine = Robot.getInstance().getSampleHandlingStateMachine();
//            operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
//                    .toggleWhenPressed(
//                            new InstantCommand(sampleHandlingStateMachine::setLiftToHighBasket),
//                            new InstantCommand(sampleHandlingStateMachine::setLiftToHome)
//                    );
//        }

        if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM) &&
                robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            SpecimenHandlingStateMachine specimenHandlingStateMachine = Robot.getInstance().getSpecimenHandlingStateMachine();

            operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                    .whenPressed(new InstantCommand(specimenHandlingStateMachine::onSpecimenHandleButtonPress));
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON  - RETRACTS AND DEPLOYS SAMPLES            //
        //                                                      //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) &&
                robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
            SampleHandlingStateMachine sampleHandlingStateMachine = Robot.getInstance().getSampleHandlingStateMachine();

            operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                    .whenPressed(new InstantCommand(sampleHandlingStateMachine::onIntakeButtonPress));
        }


        //////////////////////////////////////////////////////////
        //                                                      //
        //  LEFT TRIGGER                                        //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        //  RIGHT TRIGGER                                       //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        //  OPTIONS BUTTON                                      //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        //  START BUTTON                                        //
        //                                                      //
        //////////////////////////////////////////////////////////

    }
}
