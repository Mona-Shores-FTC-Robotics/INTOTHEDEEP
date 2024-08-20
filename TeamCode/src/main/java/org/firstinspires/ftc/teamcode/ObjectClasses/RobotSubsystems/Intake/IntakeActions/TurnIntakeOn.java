package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeActions;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem.intakeParameters;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

public class TurnIntakeOn implements Action {

    public TurnIntakeOn() {

    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        telemetryPacket.addLine("Intake On Action");
        IntakeSubsystem intakeSubsystem = Robot.getInstance().getIntakeSubsystem();

        intakeSubsystem.intake1.setDirection(DcMotor.Direction.FORWARD);
        intakeSubsystem.intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSubsystem.currentIntake1State = IntakeSubsystem.IntakeStates.INTAKE_ON;
        intakeSubsystem.intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSubsystem.intake1.setPower(intakeParameters.INTAKE_ON_POWER);

        intakeSubsystem.intake2.setDirection(DcMotor.Direction.REVERSE);
        intakeSubsystem.intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSubsystem.currentIntake2State = IntakeSubsystem.IntakeStates.INTAKE_ON;
        intakeSubsystem.intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSubsystem.intake2.setPower(intakeParameters.INTAKE_ON_POWER);

        return false;
    }
}