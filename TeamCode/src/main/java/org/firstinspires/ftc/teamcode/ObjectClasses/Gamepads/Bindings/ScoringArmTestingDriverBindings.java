package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.Bindings;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.GripperSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSlideSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.ActuateGripperCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.MoveLiftSlideCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands.RotateShoulderCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.DroneSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ReadyClimberArmCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.End_Game.ReleaseDroneCommand;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class ScoringArmTestingDriverBindings {

    public static Command rotateShoulderToBackdrop;
    public static Command rotateShoulderToIntake;
    public static Command rotateShoulderToHalfway;
    public static Command openClaw;
    public static Command closeClaw;
    public static Command liftHome;
    public static Command liftSafe;
    public static Command liftLow;
    public static Command liftMid;
    public static Command liftHigh;
    public static Command wait;
    public static ParallelRaceGroup blueBackdropBackup;
    public static ParallelRaceGroup redBackdropBackup;

    public static SequentialCommandGroup readyToScorePixel;
    public static SequentialCommandGroup releasePixels;

    public static SequentialCommandGroup moveAndReadyToScorePixel;
    public static SequentialCommandGroup releasePixelsAndMove;


    private static DriveSubsystem driveSubsystem;
    private static ShoulderSubsystem shoulderSubsystem;
    private static GripperSubsystem gripperSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static LiftSlideSubsystem liftSlideSubsystem;
    private static MecanumDriveMona mecanumDrive;
    public static DroneSubsystem droneSubsystem;

    public static TriggerReader rightTrigger;
    public ScoringArmTestingDriverBindings(GamepadEx gamepad) {
        rightTrigger = new TriggerReader(gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);

        makeSequenceCommands();

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new ActuateGripperCommand(Robot.getInstance().getEndEffectorSubsystem(), GripperSubsystem.GripperStates.OPEN),
                        new ActuateGripperCommand(Robot.getInstance().getEndEffectorSubsystem(), GripperSubsystem.GripperStates.CLOSED));

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new RotateShoulderCommand(Robot.getInstance().getShoulderSubsystem(), ShoulderSubsystem.ShoulderStates.BACKDROP),
                        new RotateShoulderCommand(Robot.getInstance().getShoulderSubsystem(), ShoulderSubsystem.ShoulderStates.INTAKE));

        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new MoveLiftSlideCommand(Robot.getInstance().getLiftSlideSubsystem(), LiftSlideSubsystem.LiftStates.LOW));

        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new MoveLiftSlideCommand(Robot.getInstance().getLiftSlideSubsystem(), LiftSlideSubsystem.LiftStates.MID));

//        gamepad.getGamepadButton(GamepadKeys.Button.B)
//                .toggleWhenPressed(readyToScorePixel, releasePixels, false);


        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new MoveLiftSlideCommand(Robot.getInstance().getLiftSlideSubsystem(), LiftSlideSubsystem.LiftStates.HOME));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new ReleaseDroneCommand(Robot.getInstance().getDroneSubsystem(), DroneSubsystem.DroneDeployState.FLY));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ReleaseDroneCommand(Robot.getInstance().getDroneSubsystem(), DroneSubsystem.DroneDeployState.HOLD));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ReadyClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.READY));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ReadyClimberArmCommand(Robot.getInstance().getClimberSubsystem(), ClimberSubsystem.ClimberArmStates.STOWED));



    }

    private void makeSequenceCommands() {

        GripperSubsystem gripperSubsystem = Robot.getInstance().getEndEffectorSubsystem();
        ShoulderSubsystem shoulderSubsystem = Robot.getInstance().getShoulderSubsystem();
        LiftSlideSubsystem liftSlideSubsystem = Robot.getInstance().getLiftSlideSubsystem();

        rotateShoulderToBackdrop = new RotateShoulderCommand(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.BACKDROP);

        rotateShoulderToIntake = new RotateShoulderCommand(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.INTAKE);

        rotateShoulderToHalfway = new RotateShoulderCommand(shoulderSubsystem,
                ShoulderSubsystem.ShoulderStates.HALFWAY);

        liftHome = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.HOME);

        liftSafe = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.SAFE);

        liftLow = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.LOW);

        liftMid = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.MID);

        liftHigh = new MoveLiftSlideCommand(liftSlideSubsystem,
                LiftSlideSubsystem.LiftStates.HIGH);

        openClaw = new ActuateGripperCommand(gripperSubsystem,
                GripperSubsystem.GripperStates.OPEN);

        closeClaw = new ActuateGripperCommand(gripperSubsystem,
                GripperSubsystem.GripperStates.CLOSED);

//        moveToPixelScoreLocation = new MoveToPointCommand(Robot.getInstance().getDriveSubsystem(),
//                mecanumDrive.pose.position.x+3,
//                mecanumDrive.pose.position.y
//                );
//
//        moveAwayFromPixelScoreLocation = new MoveToPointCommand(Robot.getInstance().getDriveSubsystem(),
//                mecanumDrive.pose.position.x-8,
//                mecanumDrive.pose.position.y
//        );

        readyToScorePixel =
                new SequentialCommandGroup(
                        new ActuateGripperCommand(gripperSubsystem,
                                GripperSubsystem.GripperStates.CLOSED),
                        new MoveLiftSlideCommand(liftSlideSubsystem,
                                LiftSlideSubsystem.LiftStates.SAFE),
                        new RotateShoulderCommand(shoulderSubsystem,
                                ShoulderSubsystem.ShoulderStates.BACKDROP),
                        new MoveLiftSlideCommand(liftSlideSubsystem,
                                LiftSlideSubsystem.LiftStates.MID)
                );

        releasePixels =
                new SequentialCommandGroup(
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
                        new ParallelCommandGroup(new RotateShoulderCommand(shoulderSubsystem,
                                ShoulderSubsystem.ShoulderStates.INTAKE),
                                new MoveLiftSlideCommand(liftSlideSubsystem,
                                        LiftSlideSubsystem.LiftStates.HOME)
                        )
                );
    }

//        moveAndReadyToScorePixel =
//                new SequentialCommandGroup(closeClaw,
//                new ParallelCommandGroup(
//                        moveToPixelScoreLocation,
//                        liftMid,
//                        new SequentialCommandGroup(
//                                new WaitCommand(100),
//                                rotateShoulderToBackdrop
//                        )));
//
//        releasePixelsAndMove =
//                new SequentialCommandGroup(
//                        openClaw,
//                        new ParallelCommandGroup(
//                                liftSafe,
//                                closeClaw,
//                                rotateShoulderToHalfway
//                        ),
//                        new ParallelCommandGroup(
//                                rotateShoulderToIntake,
//                                liftHome
//                        ),
//                        openClaw
//                );
    }
