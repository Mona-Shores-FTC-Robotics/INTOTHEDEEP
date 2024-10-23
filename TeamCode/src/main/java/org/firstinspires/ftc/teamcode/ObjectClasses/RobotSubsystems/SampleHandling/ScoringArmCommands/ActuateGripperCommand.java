package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.ScoringArmCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.GripperSubsystem;

public class ActuateGripperCommand extends CommandBase {
        private final GripperSubsystem gripperSubsystem;
        private final GripperSubsystem.GripperStates targetState;

        public ActuateGripperCommand(GripperSubsystem subsystem, GripperSubsystem.GripperStates inputState) {
            gripperSubsystem = subsystem;
            targetState = inputState;

            //add the subsystem to the requirements
            addRequirements(gripperSubsystem);
        }

        @Override
        public void initialize() {
            //set the Servo position to the target - we only have to set the servo position one time
            gripperSubsystem.endEffector.setPosition(targetState.position);
        }

        public void execute() {
            //nothing to do in the loop
        }

        @Override
        public boolean isFinished() {
            //always return true because the command simply sets the servo and we have no way of telling when the servo has finished moving
            return true;
        }

        @Override
        public void end(boolean interrupted) {
            MatchConfig.telemetryPacket.addLine("Gripper move COMPLETE From " + gripperSubsystem.currentState + " to " + targetState);
            gripperSubsystem.currentState = targetState;
        }
    }

