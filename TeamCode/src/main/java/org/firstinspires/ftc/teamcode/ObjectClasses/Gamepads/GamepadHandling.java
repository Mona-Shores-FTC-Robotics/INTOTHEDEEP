package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.example.sharedconstants.FieldConstants.*;

import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Robot.getNextRobotType;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Robot.getPreviousRobotType;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem.DriveParameters;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveClasses.MecanumDriveMona;

public class GamepadHandling {
    private final GamepadEx driverGamepad;
    private final GamepadEx operatorGamepad;

    public boolean LockedSettingsFlag = false;
    private boolean adjustmentModeActive = false;

    public GamepadHandling(LinearOpMode opMode) {
        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);

        //Set Driver Gamepad to Blue
        opMode.gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS);

        //Set Operator Gamepad to White
        opMode.gamepad2.setLedColor(1, 1, 1, LED_DURATION_CONTINUOUS);

//        CreateRumbleEffects();
//        CreateLEDEffects();
    }

//    private void CreateLEDEffects() {
//        problemLedEffect = new Gamepad.LedEffect.Builder()
//                .addStep(0, 1, 0, 500) // Show green for 250ms
//                .addStep(0, 0, 0, 500) // Show white for 250ms
//                .addStep(0, 1, 0, LED_DURATION_CONTINUOUS) // Show white for 250ms
//                .build();
//    }

//    private void CreateRumbleEffects() {
//        endGameRumbleEffect = new Gamepad.RumbleEffect.Builder()
//                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
//                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
//                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
//                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
//                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
//                .build();
//
//        problemRumbleEffect = new Gamepad.RumbleEffect.Builder()
//                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
//                .addStep(0.0, 0.0, 1000)  //  Pause for 1 Sec
//                .addStep(.5, .5, 250)  //  Rumble both motors 50% for 250 mSec
//                .addStep(0.0, 0.0, 1000)  //  Pause for 1 Sec
//                .build();
//
//        //set the rumble counter to 0
//        timeoutRumbleCounter = 0;
//    }

    public GamepadEx getDriverGamepad() {
        return driverGamepad;
    }

    public GamepadEx getOperatorGamepad() {
        return operatorGamepad;
    }

    public void SelectAndLockColorAndSideAndRobotType(Telemetry telemetry) {
        telemetry.addLine("");

        if (LockedSettingsFlag) {
            telemetry.addLine("Settings Locked");
            telemetry.addLine("Alliance: " + finalAllianceColor);
            telemetry.addLine("Side: " + finalSideOfField);
            telemetry.addLine("Robot Type: " + finalRobotType);
            telemetry.addLine("Press B to unlock settings");

            if (!adjustmentModeActive) {
                telemetry.addLine("Press X to adjust drive/strafe/turn speed factors");
            }
            // Check if the user pressed X to enter adjustment mode
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.X)) {
                adjustmentModeActive = true;  // Activate adjustment mode
            }

            // While in adjustment mode, continuously adjust the speed factors
            if (adjustmentModeActive) {
                telemetry.addLine("---ADJUST drive/strafe/turn speed factors---");
                telemetry.addLine("Press B to exit adjustment mode");
                AdjustDriveStrafeTurnSpeedFactors(telemetry);
            }

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = false;  // Unlock settings if B is pressed again
                adjustmentModeActive = false;  // Deactivate adjustment mode when unlocking settings
            }
        } else {
            telemetry.addLine("Lock settings with B");
            telemetry.addLine("Alliance: " + finalAllianceColor);
            telemetry.addLine("Side: " + finalSideOfField);
            telemetry.addLine("Robot Type: " + finalRobotType);

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = true;
            }

            // Allow selection of alliance color
            telemetry.addLine("Color (DPAD-UP/DOWN)");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                finalAllianceColor = AllianceColor.BLUE;
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                finalAllianceColor = AllianceColor.RED;
            }

            // Allow selection of side of field based on alliance color
            telemetry.addLine("Side Of Field (DPAD-LEFT/RIGHT)");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (finalAllianceColor == AllianceColor.BLUE) {
                    finalSideOfField = SideOfField.NET;
                } else if (finalAllianceColor == AllianceColor.RED) {
                    finalSideOfField = SideOfField.OBSERVATION;
                }
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                if (finalAllianceColor == AllianceColor.RED) {
                    finalSideOfField = SideOfField.NET;
                } else if (finalAllianceColor == AllianceColor.BLUE) {
                    finalSideOfField = SideOfField.OBSERVATION;
                }
            }

            // Allow selection of robot type using bumpers
            telemetry.addLine("Robot Type (Left/Right Bumper)");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                finalRobotType = getPreviousRobotType(finalRobotType);
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                finalRobotType = getNextRobotType(finalRobotType);
            }
        }
    }

    public void SelectAndLockColorAndSide() {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        telemetry.addLine("");

        if (LockedSettingsFlag) {
            telemetry.addLine("Alliance Color and Side of Field Locked");
            telemetry.addLine(finalAllianceColor + " " + finalSideOfField);
            telemetry.addLine("Press B to unlock Alliance Color and Side of Field");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = false;
            }
        } else {
            telemetry.addLine("Lock Alliance Color and Side of Field with B");
            telemetry.addLine(finalAllianceColor + " " + finalSideOfField);

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = true;
            }

            telemetry.addLine("Color (DPAD-UP/DOWN) - Side Of Field (DPAD-LEFT/RIGHT)");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                finalAllianceColor = AllianceColor.BLUE;
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                finalAllianceColor = AllianceColor.RED;
            }

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (finalAllianceColor == AllianceColor.BLUE) {
                    finalSideOfField = SideOfField.NET;
                } else if (finalAllianceColor == AllianceColor.RED) {
                    finalSideOfField = SideOfField.OBSERVATION;
                }
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                if (finalAllianceColor == AllianceColor.RED) {
                    finalSideOfField = SideOfField.NET;
                } else if (finalAllianceColor == AllianceColor.BLUE) {
                    finalSideOfField = SideOfField.OBSERVATION;
                }
            }
        }
    }

    public void AdjustDriveStrafeTurnSpeedFactors(Telemetry telemetry) {
        telemetry.addLine("");  // Just formatting

        // Adjust Drive Speed Factor using the D-Pad Left/Right
        telemetry.addLine("Adjust Drive Speed (D-Pad Left - Decrease, D-Pad Right - Increase)");
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            DriveParameters.DRIVE_SPEED_FACTOR = Math.max(0, Math.min(1, DriveParameters.DRIVE_SPEED_FACTOR - 0.01));  // Decrease and cap between 0 and 1
        } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            DriveParameters.DRIVE_SPEED_FACTOR = Math.max(0, Math.min(1, DriveParameters.DRIVE_SPEED_FACTOR + 0.01));  // Increase and cap between 0 and 1
        }
        telemetry.addData("Drive Speed Factor: ", DriveParameters.DRIVE_SPEED_FACTOR);

        // Adjust Strafe Speed Factor using X/Y buttons
        telemetry.addLine("Adjust Strafe Speed (X - Decrease, Y - Increase)");
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            DriveParameters.STRAFE_SPEED_FACTOR = Math.max(0, Math.min(1, DriveParameters.STRAFE_SPEED_FACTOR - 0.01));  // Decrease and cap between 0 and 1
        } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            DriveParameters.STRAFE_SPEED_FACTOR = Math.max(0, Math.min(1, DriveParameters.STRAFE_SPEED_FACTOR + 0.01));  // Increase and cap between 0 and 1
        }
        telemetry.addData("Strafe Speed Factor: ", DriveParameters.STRAFE_SPEED_FACTOR);

        // Adjust Turn Speed Factor using A/B buttons
        telemetry.addLine("Adjust Turn Speed (A - Decrease, B - Increase)");
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            DriveParameters.TURN_SPEED_FACTOR = Math.max(0, Math.min(1, DriveParameters.TURN_SPEED_FACTOR - 0.01));  // Decrease and cap between 0 and 1
        } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
            DriveParameters.TURN_SPEED_FACTOR = Math.max(0, Math.min(1, DriveParameters.TURN_SPEED_FACTOR + 0.01));  // Increase and cap between 0 and 1
        }
        telemetry.addData("Turn Speed Factor: ", DriveParameters.TURN_SPEED_FACTOR);
    }

    public void AdjustPIDF(Telemetry telemetry) {
        telemetry.addLine("");  // Just formatting

        MecanumDriveMona.MonaTeleopParams monaTeleopParams = Robot.getInstance().getDriveSubsystem().getMecanumDrive().getMonaTeleopParams();

        // Adjust P value using D-Pad Left/Right
        telemetry.addLine("Adjust P Value (D-Pad Left - Decrease, D-Pad Right - Increase)");
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            monaTeleopParams.P = Math.max(0, monaTeleopParams.P - 0.01);  // Decrease
        } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            monaTeleopParams.P = Math.min(1, monaTeleopParams.P + 0.01);  // Increase
        }
        telemetry.addData("P: ", "%.2f", monaTeleopParams.P);

        // Adjust I value using X/Y buttons
        telemetry.addLine("Adjust I Value (X - Decrease, Y - Increase)");
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            monaTeleopParams.I = Math.max(0, monaTeleopParams.I - 0.01);  // Decrease
        } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            monaTeleopParams.I = Math.min(1, monaTeleopParams.I + 0.01);  // Increase
        }
        telemetry.addData("I: ", "%.2f", monaTeleopParams.I);

        // Adjust D value using A/B buttons
        telemetry.addLine("Adjust D Value (A - Decrease, B - Increase)");
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            monaTeleopParams.D = Math.max(0, monaTeleopParams.D - 0.01);  // Decrease
        } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
            monaTeleopParams.D = Math.min(1, monaTeleopParams.D + 0.01);  // Increase
        }
        telemetry.addData("D: ", "%.2f", monaTeleopParams.D);

        // Adjust F value using Left/Right Trigger
        telemetry.addLine("Adjust F Value (Left Trigger - Decrease, Right Trigger - Increase)");
        if (driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            monaTeleopParams.F = Math.max(0, monaTeleopParams.F - 0.01);  // Decrease
        } else if (driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            monaTeleopParams.F = Math.min(1, monaTeleopParams.F + 0.01);  // Increase
        }
        telemetry.addData("F: ", "%.2f", monaTeleopParams.F);

        telemetry.update();
    }

}
