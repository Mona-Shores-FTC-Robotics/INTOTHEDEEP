package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.ActionCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.Deprecated.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.SampleLiftSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.MoveSampleLiftAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem;
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

        // INTAKE ON while held down, off when not
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(
//                        new SequentialCommandGroup(
//                                new ActuateGripperCommand(gripperSubsystem, GripperSubsystem.GripperStates.OPEN),
//                                new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_ON, IntakeSubsystem.IntakeStates.INTAKE_SLOW)
//                        ))
//                .whenReleased(
//                        new SequentialCommandGroup(
//                                new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF, IntakeSubsystem.IntakeStates.INTAKE_OFF),
//                                new WaitCommand(300),
//                                new ActuateGripperCommand(gripperSubsystem, GripperSubsystem.GripperStates.CLOSED)
//                        ));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON - REVERSE INTAKE                           //
        //                                                      //
        //////////////////////////////////////////////////////////

        // INTAKE ON while held down, off when not
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_REVERSE, IntakeSubsystem.IntakeStates.INTAKE_REVERSE))
//                .whenReleased(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF, IntakeSubsystem.IntakeStates.INTAKE_OFF));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON - READY TO SCORE PIXELS MID HEIGHT         //
        //                                                      //
        //////////////////////////////////////////////////////////

//        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(new InstantCommand(()-> {
//                            new MakeOperatorCombinationCommands().ReadyToScorePixelCommand().schedule();
//                        }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON  - RELEASE PIXELS                          //
        //                                                      //
        //////////////////////////////////////////////////////////

//        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new MakeOperatorCombinationCommands().ReleasePixels(), false);

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
