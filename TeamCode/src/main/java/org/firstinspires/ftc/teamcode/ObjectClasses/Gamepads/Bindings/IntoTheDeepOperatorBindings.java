package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingStateMachine;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.ChangeIntakePowerAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.SampleLiftSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.MoveSampleLiftAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.MoveLinearActuatorAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

import java.util.Collections;
import java.util.Set;

public class IntoTheDeepOperatorBindings {

    public static TriggerReader rightTrigger;
    public static TriggerReader leftTrigger;

    public IntoTheDeepOperatorBindings(GamepadEx operatorGamepad) {

        SampleIntakeSubsystem sampleIntakeSubsystem = Robot.getInstance().getSampleIntakeSubsystem();
        Set<Subsystem> sampleIntakeRequirements = Collections.singleton(sampleIntakeSubsystem);
        SampleLiftSubsystem sampleLiftSubsystem = Robot.getInstance().getSampleLiftSubsystem();
        Set<Subsystem>  sampleLiftRequirements = Collections.singleton(sampleLiftSubsystem);

        SampleLinearActuatorSubsystem sampleLinearActuatorSubsystem = Robot.getInstance().getSampleLinearActuatorSubsystem();
        Set<Subsystem>  sampleLinearActuatorRequirements = Collections.singleton(sampleLinearActuatorSubsystem);

        SampleHandlingStateMachine sampleHandlingStateMachine = Robot.getInstance().getSampleHandlingStateMachine();

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

        ChangeIntakePowerAction turnOffIntake = new ChangeIntakePowerAction(sampleIntakeSubsystem, SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
        ChangeIntakePowerAction turnOnIntake = new ChangeIntakePowerAction(sampleIntakeSubsystem, SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);

            operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                    .whenPressed(new ActionCommand(turnOnIntake, sampleIntakeRequirements))
                    .whenReleased(new ActionCommand(turnOffIntake, sampleIntakeRequirements)
                            );

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON - REVERSE INTAKE                           //
        //                                                      //
        //////////////////////////////////////////////////////////

        ChangeIntakePowerAction reverseIntake = new ChangeIntakePowerAction(sampleIntakeSubsystem, SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);

        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ActionCommand(reverseIntake, sampleIntakeRequirements))
                .whenReleased(new ActionCommand(turnOffIntake, sampleIntakeRequirements)
                );

        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON - TOGGLE LIFTING SAMPLE HIGH BASKET/HOME   //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(
                        new InstantCommand(sampleHandlingStateMachine::setLiftToHighBasket),
                        new InstantCommand(sampleHandlingStateMachine::setLiftToHome)
                );

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON  - RETRACTS AND DEPLOYS SAMPLES            //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(sampleHandlingStateMachine::onIntakeButtonPress));

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
