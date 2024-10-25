package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated.GripperSubsystem;
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
    public static ParallelCommandGroup readyToScorePixel;
    public static SequentialCommandGroup releasePixels;
    private static boolean armIsUp = false;

    public IntoTheDeepOperatorBindings(GamepadEx operatorGamepad) {

        VisionSubsystem visionSubsystem = Robot.getInstance().getVisionSubsystem();
        GripperSubsystem gripperSubsystem = Robot.getInstance().getEndEffectorSubsystem();
        ClimberSubsystem climberSubsystem = Robot.getInstance().getClimberSubsystem();
        SampleIntakeSubsystem sampleIntakeSubsystem = Robot.getInstance().getSampleIntakeSubsystem();
        Set<Subsystem> sampleIntakeRequirements = Collections.singleton(sampleIntakeSubsystem);
        SampleLiftSubsystem sampleLiftSubsystem = Robot.getInstance().getSampleLiftSubsystem();
        Set<Subsystem>  sampleLiftRequirements = Collections.singleton(sampleLiftSubsystem);
        SampleLinearActuatorSubsystem sampleLinearActuatorSubsystem = Robot.getInstance().getSampleLinearActuatorSubsystem();
        Set<Subsystem>  sampleLinearActuatorRequirements = Collections.singleton(sampleLinearActuatorSubsystem);

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT STICK / RIGHT STICK                             //
        //                                                      //
        //////////////////////////////////////////////////////////

        //do nothing

        //////////////////////////////////////////////////////////
        //                                                      //
        // RIGHT BUMPER - ROBOT UP WITH WINCH                   //
        //                                                      //
        //////////////////////////////////////////////////////////
//
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(
//                        new InstantCommand(() -> {
//                            if (MatchConfig.teleOpTimer.seconds() > END_GAME_TIME) {
//                                if (armIsUp) {
//                                    new ChangeWinchPowerCommand(climberSubsystem, ClimberSubsystem.WinchMotorStates.ROBOT_UP).schedule();
//                                }
//                            }
//                        }
//                        )
//                )
//                .whenReleased(
//                        new ChangeWinchPowerCommand(climberSubsystem, ClimberSubsystem.WinchMotorStates.OFF)
//                );

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-UP -                                            //
        //                                                      //
        //////////////////////////////////////////////////////////

//        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(()->{
//                    visionSubsystem.setDeliverHeight(LiftSlideSubsystem.LiftStates.MAX);
//                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-RIGHT -                                         //
        //                                                      //
        //////////////////////////////////////////////////////////
//
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new InstantCommand(()->{
//                    visionSubsystem.setDeliverHeight(LiftSlideSubsystem.LiftStates.HIGH);
//                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-LEFT -                                          //
        //                                                      //
        //////////////////////////////////////////////////////////

//        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new InstantCommand(()->{
//                    visionSubsystem.setDeliverHeight(LiftSlideSubsystem.LiftStates.MID);
//                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        // DPAD-DOWN -                                          //
        //                                                      //
        //////////////////////////////////////////////////////////

//        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(()->{
//                    visionSubsystem.setDeliverHeight(LiftSlideSubsystem.LiftStates.LOW);
//                }));

        //////////////////////////////////////////////////////////
        //                                                      //
        // LEFT BUMPER  - CLIMBER ARM TO READY POSITION         //
        //                                                      //
        //////////////////////////////////////////////////////////

//        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .toggleWhenPressed(
//                        new InstantCommand(() -> {
//                            if (MatchConfig.teleOpTimer.seconds() > END_GAME_TIME) {
//                                new ReadyClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.READY).schedule();
//                                armIsUp=true;
//                            }
//                        }),
//                        new InstantCommand(() -> {
//                            if (MatchConfig.teleOpTimer.seconds() > END_GAME_TIME) {
//                                new ReadyClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.STOWED).schedule();
//                                armIsUp=false;
//                            }
//                        }));

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
        //  Y BUTTON - MOVE SAMPLE TO HIGH BASKET THEN HOME     //
        //                                                      //
        //////////////////////////////////////////////////////////

        MoveSampleLiftAction sampleLiftHome = new MoveSampleLiftAction(SampleLiftSubsystem.SampleLiftStates.HOME);
        MoveSampleLiftAction highBasket = new MoveSampleLiftAction(SampleLiftSubsystem.SampleLiftStates.HIGH_BASKET);
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(
                        new ActionCommand(highBasket, sampleLiftRequirements),
                        new ActionCommand(sampleLiftHome, sampleLiftRequirements)
                        );

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON  - RETRACTS AND DEPLOYS SAMPLES            //
        //                                                      //
        //////////////////////////////////////////////////////////

        MoveLinearActuatorAction actuatorRetract = new MoveLinearActuatorAction(SampleLinearActuatorSubsystem.SampleActuatorStates.RETRACT);
        MoveLinearActuatorAction actuatorDeployFull = new MoveLinearActuatorAction(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_FULL);
        MoveLinearActuatorAction actuatorDeployMid = new MoveLinearActuatorAction(SampleLinearActuatorSubsystem.SampleActuatorStates.DEPLOY_MID);

// State counter to track the current position in the cycle (0, 1, 2)
        final int[] stateCounter = {0};  // Use an array to allow mutation within lambda

        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    // Increment state counter and wrap it around if necessary
                    stateCounter[0] = (stateCounter[0] + 1) % 3;

                    switch (stateCounter[0]) {
                        case 0:
                            // First press - Deploy Full
                            new ActionCommand(actuatorDeployMid, sampleLinearActuatorRequirements).schedule();
                            break;
                        case 1:
                            // Second press - Deploy Mid
                            new ActionCommand(actuatorDeployFull, sampleLinearActuatorRequirements).schedule();
                            break;
                        case 2:
                            // Third press - Retract
                            new ActionCommand(actuatorRetract, sampleLinearActuatorRequirements).schedule();
                            break;
                    }
                });


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

    private class MakeOperatorCombinationCommands{

        SampleLiftSubsystem sampleLiftSubsystem = Robot.getInstance().getSampleLiftSubsystem();
        private Command ReadyToScoreSampleCommand() {
            Set<Subsystem> requirements = Collections.singleton(sampleLiftSubsystem);
            MoveSampleLiftAction liftSampleHighBasketAction = new MoveSampleLiftAction(SampleLiftSubsystem.SampleLiftStates.HIGH_BASKET);
            return new ActionCommand(liftSampleHighBasketAction, requirements);
        }

        private Command ReleasePixels() {
            return null;
//            return new SequentialCommandGroup(
//                            new ActuateGripperCommand(gripperSubsystem,
//                                    GripperSubsystem.GripperStates.OPEN),
//                            new WaitCommand(325),
//                            new ParallelCommandGroup(
//                                    new MoveLiftSlideCommand(liftSlideSubsystem,
//                                            LiftSlideSubsystem.LiftStates.SAFE),
//                                    new ActuateGripperCommand(gripperSubsystem,
//                                            GripperSubsystem.GripperStates.CLOSED),
//                                    new RotateShoulderCommand(shoulderSubsystem,
//                                            ShoulderSubsystem.ShoulderStates.HALFWAY)
//                            ),
//                            new WaitCommand(250),
//                            new RotateShoulderCommand(shoulderSubsystem,
//                                    ShoulderSubsystem.ShoulderStates.INTAKE),
//                            new WaitCommand(250),
//                            new MoveLiftSlideCommand(liftSlideSubsystem,
//                                    LiftSlideSubsystem.LiftStates.HOME)
//                    );
        }
    }
}
