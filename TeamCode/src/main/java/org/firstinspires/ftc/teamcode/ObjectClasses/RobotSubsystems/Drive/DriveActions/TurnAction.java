package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;


import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.MotorParametersRR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

public final class TurnAction implements Action {
    private final TimeTurn turn;

    private double beginTs = -1;

    public TurnAction(TimeTurn turn) {
        this.turn = turn;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket p) {
        double t;
        if (beginTs < 0) {
            beginTs = Actions.now();
            t = 0;
        } else {
            t = Actions.now() - beginTs;
        }

        if (t >= turn.duration) {

            Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.setPower(0);
            Robot.getInstance().getDriveSubsystem().mecanumDrive.leftBack.setPower(0);
            Robot.getInstance().getDriveSubsystem().mecanumDrive.rightBack.setPower(0);
            Robot.getInstance().getDriveSubsystem().mecanumDrive.rightFront.setPower(0);

            return false;
        }

        Pose2dDual<Time> txWorldTarget = turn.get(t);

        PoseVelocity2d robotVelRobot = Robot.getInstance().getDriveSubsystem().mecanumDrive.updatePoseEstimate();

        PoseVelocity2dDual<Time> command = new HolonomicController(
                MotorParametersRR.axialGain, MotorParametersRR.lateralGain, MotorParametersRR.headingGain,
                MotorParametersRR.axialVelGain, MotorParametersRR.lateralVelGain, MotorParametersRR.headingVelGain
        )
                .compute(txWorldTarget, Robot.getInstance().getDriveSubsystem().mecanumDrive.pose, robotVelRobot);

        MecanumKinematics.WheelVelocities<Time> wheelVels = Robot.getInstance().getDriveSubsystem().mecanumDrive.kinematics.inverse(command);
        double voltage = Robot.getInstance().getDriveSubsystem().mecanumDrive.voltageSensor.getVoltage();
        Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.setPower(Robot.getInstance().getDriveSubsystem().mecanumDrive.feedforward.compute(wheelVels.leftFront) / voltage);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.leftBack.setPower(Robot.getInstance().getDriveSubsystem().mecanumDrive.feedforward.compute(wheelVels.leftBack) / voltage);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.rightBack.setPower(Robot.getInstance().getDriveSubsystem().mecanumDrive.feedforward.compute(wheelVels.rightBack) / voltage);
        Robot.getInstance().getDriveSubsystem().mecanumDrive.rightFront.setPower(Robot.getInstance().getDriveSubsystem().mecanumDrive.feedforward.compute(wheelVels.rightFront) / voltage);

        FlightRecorder.write("TARGET_POSE", new PoseMessage(txWorldTarget.value()));

        Canvas c = p.fieldOverlay();
        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawPoseHistory(c);

        c.setStroke("#4CAF50");
        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawRobot(c, txWorldTarget.value());

        c.setStroke("#3F51B5");
        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawRobot(c,Robot.getInstance().getDriveSubsystem().mecanumDrive.pose);

        c.setStroke("#7C4DFFFF");
        c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

        return true;
    }

    @Override
    public void preview(Canvas c) {
        c.setStroke("#7C4DFF7A");
        c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
    }
}