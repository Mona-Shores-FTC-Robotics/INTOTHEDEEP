package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingStateMachine;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.ChangeIntakePowerAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.SampleLiftSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;

import java.util.Collections;
import java.util.Set;

public class IntoTheDeepOperatorBindings {

    public static TriggerReader rightTrigger;
    public static TriggerReader leftTrigger;

    public IntoTheDeepOperatorBindings(GamepadEx operatorGamepad) {
        Robot robot = Robot.getInstance();


        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT STICK / RIGHT STICK                             //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT BUMPER - ROBOT UP WITH WINCH                   //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-UP -                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

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

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-DOWN -                                          //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT BUMPER  - CLIMBER ARM TO READY POSITION         //
        //                                                      //
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON - INTAKE                                   //
        //                                                      //
        //////////////////////////////////////////////////////////

        // Only bind this button if we actually have the subsystems
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE))
        {
            SampleIntakeSubsystem sampleIntakeSubsystem = robot.getSampleIntakeSubsystem();
            Set<Subsystem> sampleIntakeRequirements = Collections.singleton(sampleIntakeSubsystem);
            ChangeIntakePowerAction turnOffIntake = new ChangeIntakePowerAction(sampleIntakeSubsystem, SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
            ChangeIntakePowerAction turnOnIntake = new ChangeIntakePowerAction(sampleIntakeSubsystem, SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                    .whenPressed(new ActionCommand(turnOnIntake, sampleIntakeRequirements))
                    .whenReleased(new ActionCommand(turnOffIntake, sampleIntakeRequirements)
                    );
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON - REVERSE INTAKE                           //
        //                                                      //
        //////////////////////////////////////////////////////////

        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
            SampleIntakeSubsystem sampleIntakeSubsystem = robot.getSampleIntakeSubsystem();
            Set<Subsystem> sampleIntakeRequirements = Collections.singleton(sampleIntakeSubsystem);

            ChangeIntakePowerAction reverseIntake = new ChangeIntakePowerAction(sampleIntakeSubsystem, SampleIntakeSubsystem.SampleIntakeStates.INTAKE_REVERSE);
            ChangeIntakePowerAction turnOffIntake = new ChangeIntakePowerAction(sampleIntakeSubsystem, SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                    .whenPressed(new ActionCommand(reverseIntake, sampleIntakeRequirements))
                    .whenReleased(new ActionCommand(turnOffIntake, sampleIntakeRequirements)
                    );
        }
        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON - TOGGLE LIFTING SAMPLE HIGH BASKET/HOME   //
        //                                                      //
        //////////////////////////////////////////////////////////

        // this button can only be bound if we actually have all the subsystems we need
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) &&
                robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT) &&
                robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
            SampleHandlingStateMachine sampleHandlingStateMachine = Robot.getInstance().getSampleHandlingStateMachine();
            operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                    .toggleWhenPressed(
                            new InstantCommand(sampleHandlingStateMachine::setLiftToHighBasket),
                            new InstantCommand(sampleHandlingStateMachine::setLiftToHome)
                    );
        }

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON  - RETRACTS AND DEPLOYS SAMPLES            //
        //                                                      //
        //////////////////////////////////////////////////////////
        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) &&
                robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT) &&
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
