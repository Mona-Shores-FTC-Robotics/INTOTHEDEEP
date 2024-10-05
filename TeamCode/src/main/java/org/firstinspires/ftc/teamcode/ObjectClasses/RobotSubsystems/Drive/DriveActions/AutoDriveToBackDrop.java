package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import static com.example.sharedconstants.FieldConstants.*;

import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.*;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;
@Config
public class AutoDriveToBackDrop implements Action {
    public static double P_TERM = .016;
    public static double I_TERM;
    public static double D_TERM;
    public static double F_TERM = .15;
    private DriveSubsystem driveSubsystem;
    private boolean running;
    private VisionSubsystem.DeliverLocation deliverLocation;
    private int count=0;

    public AutoDriveToBackDrop() {
        driveSubsystem = Robot.getInstance().getDriveSubsystem();
        running=true;
    }

    public AutoDriveToBackDrop(VisionSubsystem.DeliverLocation delLoc) {
        driveSubsystem = Robot.getInstance().getDriveSubsystem();
        deliverLocation=delLoc;
        running=true;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (deliverLocation!=null)
        {
            Robot.getInstance().getVisionSubsystem().setDeliverLocation(deliverLocation);
        }

        Robot.getInstance().getVisionSubsystem().LookForAprilTags();

        if (finalAllianceColor == AllianceColor.RED) {
            running = Robot.getInstance().getVisionSubsystem().AutoDriveToBackdropRed();
        } else{
            running = Robot.getInstance().getVisionSubsystem().AutoDriveToBackdropBlue();
        }

//        driveSubsystem.mecanumDrive.mecanumDriveSpeedControl(driveSubsystem.mecanumDrive.aprilTagDrive, driveSubsystem.mecanumDrive.aprilTagStrafe, driveSubsystem.mecanumDrive.aprilTagTurn);

        Robot.getInstance().getDriveSubsystem().getMecanumDrive().updatePoseEstimate();

        if (Robot.getInstance().getVisionSubsystem().resetPoseReady){
            Robot.getInstance().getVisionSubsystem().resetPoseReady=false;
            Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose = Robot.getInstance().getVisionSubsystem().resetPose;
        }

        Canvas c = telemetryPacket.fieldOverlay();

        telemetryPacket.put("x", Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose.position.x);
        telemetryPacket.put("y", Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose.position.y);
        telemetryPacket.put("heading (deg)", Math.toDegrees(Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose.heading.log()));
//        telemetryPacket.put("April Tag Drive", driveSubsystem.mecanumDrive.aprilTagDrive);
//        telemetryPacket.put("April Tag Strafe", driveSubsystem.mecanumDrive.aprilTagStrafe);
//        telemetryPacket.put("April Tag Turn", driveSubsystem.mecanumDrive.aprilTagTurn);
//        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawPoseHistory(c);

        c.setStroke("#3F51B5");
//        Robot.getInstance().getDriveSubsystem().mecanumDrive.drawRobot(c, Robot.getInstance().getDriveSubsystem().mecanumDrive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        if (!running) count++;
        //while the action is running return true
       if (count>4) {
           count=0;
           return false;}
        else return true;
    }
}
