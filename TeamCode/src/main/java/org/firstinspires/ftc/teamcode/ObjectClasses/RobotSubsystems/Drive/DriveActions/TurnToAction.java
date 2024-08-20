package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.TurnPIDController;

@Config
public class TurnToAction implements Action{

    public static double P_TERM = .016;
    public static double I_TERM;
    public static double D_TERM;
    public static double F_TERM = .15;
    public static int COUNT_THRESHOLD = 3;
    public static double ERROR_THRESHOLD = .5;

    private double currentAngle;
    private final MecanumDriveMona mecanumDrive;
    double turn;
    int turnCompleteCounter;

    TurnPIDController turnPIDController;

    public TurnToAction(double targetDegrees) {
        mecanumDrive = Robot.getInstance().getDriveSubsystem().mecanumDrive;
        turnPIDController=new TurnPIDController(targetDegrees,P_TERM, I_TERM, D_TERM, F_TERM);

    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        currentAngle = Math.toDegrees(Robot.getInstance().getGyroSubsystem().getCurrentRelativeYawRadians());
        turn=turnPIDController.update(currentAngle);
        mecanumDrive.mecanumDriveSpeedControl(0, 0, turn);

        telemetryPacket.put("turn error", turnPIDController.error);
        telemetryPacket.put("current angle", currentAngle);

        if (Math.abs(turnPIDController.error) < ERROR_THRESHOLD )
        {
            turnCompleteCounter++;
        }

        if (turnCompleteCounter > COUNT_THRESHOLD)
        {
            turnCompleteCounter=0;
            return false;
        } else return true;
    }
}
