package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;

import static com.example.sharedconstants.FieldConstants.END_GAME_TIME;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.ActuateGripperCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.MoveLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.RotateShoulderCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ReadyClimberArmCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeCommands.ChangeIntakePowerCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

public class CenterStageOperatorBindings {

    public static TriggerReader rightTrigger;
    public static TriggerReader leftTrigger;
    public static ParallelCommandGroup readyToScorePixel;
    public static SequentialCommandGroup releasePixels;
    private static boolean armIsUp = false;

    public CenterStageOperatorBindings(GamepadEx operatorGamepad) {
        VisionSubsystem visionSubsystem = Robot.getInstance().getVisionSubsystem();
        IntakeSubsystem intakeSubsystem = Robot.getInstance().getIntakeSubsystem();
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

        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(() -> {
                            if (MatchConfig.teleOpTimer.seconds() > END_GAME_TIME) {
                                new ReadyClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.READY).schedule();
                                armIsUp=true;
                            }
                        }),
                        new InstantCommand(() -> {
                            if (MatchConfig.teleOpTimer.seconds() > END_GAME_TIME) {
                                new ReadyClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.STOWED).schedule();
                                armIsUp=false;
                            }
                        }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  X BUTTON - INTAKE                                   //
        //                                                      //
        //////////////////////////////////////////////////////////

        // INTAKE ON while held down, off when not
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new SequentialCommandGroup(
                                new ActuateGripperCommand(gripperSubsystem, GripperSubsystem.GripperStates.OPEN),
                                new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_ON, IntakeSubsystem.IntakeStates.INTAKE_SLOW)
                        ))
                .whenReleased(
                        new SequentialCommandGroup(
                                new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF, IntakeSubsystem.IntakeStates.INTAKE_OFF),
                                new WaitCommand(300),
                                new ActuateGripperCommand(gripperSubsystem, GripperSubsystem.GripperStates.CLOSED)
                        ));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  B BUTTON - REVERSE INTAKE                           //
        //                                                      //
        //////////////////////////////////////////////////////////

        // INTAKE ON while held down, off when not
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_REVERSE, IntakeSubsystem.IntakeStates.INTAKE_REVERSE))
                .whenReleased(new ChangeIntakePowerCommand(intakeSubsystem, IntakeSubsystem.IntakeStates.INTAKE_OFF, IntakeSubsystem.IntakeStates.INTAKE_OFF));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  Y BUTTON - READY TO SCORE PIXELS MID HEIGHT         //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(()-> {
                            new MakeOperatorCombinationCommands().ReadyToScorePixelCommand().schedule();
                        }));

        //////////////////////////////////////////////////////////
        //                                                      //
        //  A BUTTON  - RELEASE PIXELS                          //
        //                                                      //
        //////////////////////////////////////////////////////////

        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new MakeOperatorCombinationCommands().ReleasePixels(), false);

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

        GripperSubsystem gripperSubsystem = Robot.getInstance().getEndEffectorSubsystem();
        ShoulderSubsystem shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
        LiftSlideSubsystem liftSlideSubsystem = Robot.getInstance().getLiftSlideSubsystem();

        private Command ReadyToScorePixelCommand() {

            GripperSubsystem gripperSubsystem = Robot.getInstance().getEndEffectorSubsystem();
            ShoulderSubsystem shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
            LiftSlideSubsystem liftSlideSubsystem = Robot.getInstance().getLiftSlideSubsystem();
            return new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new ActuateGripperCommand(gripperSubsystem,
                                    GripperSubsystem.GripperStates.CLOSED),
                            new MoveLiftSlideCommand(liftSlideSubsystem,
                                    LiftSlideSubsystem.LiftStates.SAFE),
                            new RotateShoulderCommand(shoulderSubsystem,
                                    ShoulderSubsystem.ShoulderStates.BACKDROP),
                            new MoveLiftSlideCommand(liftSlideSubsystem,
                                    Robot.getInstance().getVisionSubsystem().getDeliverHeight())
                    ));
        }

        private Command ReleasePixels() {
            return new SequentialCommandGroup(
                            new ActuateGripperCommand(gripperSubsystem,
                                    GripperSubsystem.GripperStates.OPEN),
                            new WaitCommand(325),
                            new ParallelCommandGroup(
                                    new MoveLiftSlideCommand(liftSlideSubsystem,
                                            LiftSlideSubsystem.LiftStates.SAFE),
                                    new ActuateGripperCommand(gripperSubsystem,
                                            GripperSubsystem.GripperStates.CLOSED),
                                    new RotateShoulderCommand(shoulderSubsystem,
                                            ShoulderSubsystem.ShoulderStates.HALFWAY)
                            ),
                            new WaitCommand(250),
                            new RotateShoulderCommand(shoulderSubsystem,
                                    ShoulderSubsystem.ShoulderStates.INTAKE),
                            new WaitCommand(250),
                            new MoveLiftSlideCommand(liftSlideSubsystem,
                                    LiftSlideSubsystem.LiftStates.HOME)
                    );
        }
    }
}
