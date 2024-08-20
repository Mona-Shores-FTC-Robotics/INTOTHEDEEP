package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;


import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.DriveTrainConstants;
import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.MecanumDriveMona.MotorParametersRR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.util.List;

public final class FollowTrajectoryAction implements Action {
    public final TimeTrajectory timeTrajectory;
    private double beginTs = -1;

    private final double[] xPoints, yPoints;

    public FollowTrajectoryAction(TimeTrajectory t) {
        timeTrajectory = t;

        List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                0, t.path.length(),
                (int) Math.ceil(t.path.length() / 2));
        xPoints = new double[disps.size()];
        yPoints = new double[disps.size()];
        for (int i = 0; i < disps.size(); i++) {
            Pose2d p = t.path.get(disps.get(i), 1).value();
            xPoints[i] = p.position.x;
            yPoints[i] = p.position.y;
        }
    }

    @Override
    public boolean run(@NonNull TelemetryPacket p) {
        //todo just put this here to see if this gives us live updating of RR parameters while driving trajectories
        Robot.getInstance().getDriveSubsystem().mecanumDrive.SetRoadRunnerParameters();

        double t;
        if (beginTs < 0) {
            beginTs = Actions.now();
            t = 0;
        } else {
            t = Actions.now() - beginTs;
        }

        if (t >= timeTrajectory.duration) {
            Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.setPower(0);
            Robot.getInstance().getDriveSubsystem().mecanumDrive.leftBack.setPower(0);
            Robot.getInstance().getDriveSubsystem().mecanumDrive.rightBack.setPower(0);
            Robot.getInstance().getDriveSubsystem().mecanumDrive.rightFront.setPower(0);

            return false;
        }

        Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

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

//        p.put("x", Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.x);
//        p.put("y", Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.position.y);
//        p.put("heading (deg)", Math.toDegrees(Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.heading.log()));

        Pose2d error = txWorldTarget.value().minusExp(Robot.getInstance().getDriveSubsystem().mecanumDrive.pose);
//        p.put("xError", error.position.x);
//        p.put("yError", error.position.y);
//        p.put("headingError (deg)", Math.toDegrees(error.heading.log()));

        double actualSpeedLF = Math.round(100.0 * Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.getVelocity() / DriveTrainConstants.TICKS_PER_REV);
        double powerLF = Robot.getInstance().getDriveSubsystem().mecanumDrive.leftFront.getPower();
//        p.addLine("LF" + " Speed: " + JavaUtil.formatNumber(actualSpeedLF, 4, 1) + " " + "Power: " + Math.round(100.0 * powerLF) / 100.0);
//        p.put("LF Speed", actualSpeedLF);
//        p.put("LF Power", powerLF);

        // only draw when active; only one drive action should be active at a time
        Canvas c = p.fieldOverlay();
        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawPoseHistory(c);

        c.setStroke("#4CAF50");
        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawRobot(c, txWorldTarget.value());

        c.setStroke("#3F51B5");
        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawRobot(c, Robot.getInstance().getDriveSubsystem().mecanumDrive.pose);

        c.setStroke("#4CAF50FF");
        c.setStrokeWidth(1);
        c.strokePolyline(xPoints, yPoints);

        return true;
    }

    @Override
    public void preview(Canvas c) {
        c.setStroke("#4CAF507A");
        c.setStrokeWidth(1);
        c.strokePolyline(xPoints, yPoints);
    }
}
