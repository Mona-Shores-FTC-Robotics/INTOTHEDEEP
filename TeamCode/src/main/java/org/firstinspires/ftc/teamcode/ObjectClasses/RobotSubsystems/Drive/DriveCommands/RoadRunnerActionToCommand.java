package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class RoadRunnerActionToCommand {
    public static class ActionAsCommand extends CommandBase {

        private FtcDashboard dash;
        private Canvas c;
        private TelemetryPacket p;
        private boolean running = false;
        private Action action;

        public ActionAsCommand(DriveSubsystem subsystem, Action a){
            action=a;
            addRequirements(subsystem);
        }

        @Override
        public void initialize(){
            c = new Canvas();
            dash = FtcDashboard.getInstance();
            action.preview(c);
            running = false;
        }

        @Override
        public void execute(){
            p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());
            running = action.run(p);
            dash.sendTelemetryPacket(p);
        }

        @Override
        public boolean isFinished(){
            if (running)
            {
                return false;
            } else return true;
        }
    }
}
