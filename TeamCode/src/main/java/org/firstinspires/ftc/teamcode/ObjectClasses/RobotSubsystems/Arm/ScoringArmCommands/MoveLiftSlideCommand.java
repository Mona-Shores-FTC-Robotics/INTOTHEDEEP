package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSubsystem.LIFT_PARAMS;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSubsystem;

public class MoveLiftSlideCommand extends CommandBase {
    //Declare and set a timeout threshold for the command called TIMEOUT_TIME_SECONDS - I suggest 1.5 seconds for now


    //Declare the local variable to hold the liftsubsystem
    private final LiftSubsystem liftSubsystem;

    //Declare currentTicks and targetTicks for use locally
    private int targetTicks;
    private int currentTicks;
    private LiftSubsystem.LiftStates targetState;

    //declare a timeoutTimer (type ElapsedTime) to timeout the command if it doesn't finish
    private ElapsedTime timeoutTimer;

    //declare a timeout boolean
    boolean timeout;

    public MoveLiftSlideCommand(LiftSubsystem subsystem, LiftSubsystem.LiftStates inputState) {
        liftSubsystem = subsystem;
        targetState = inputState;
        timeoutTimer = new ElapsedTime();
        //add the subsystem to the requirements
        addRequirements(liftSubsystem);
    }

        @Override
        public void initialize() {
        //When the command is first run set the targetState of the subsystem to the targetState and set the target ticks to the target ticks of that state
            liftSubsystem.setTargetState(targetState);
            liftSubsystem.setTargetTicks(liftSubsystem.getTargetState().ticks);

            //reset the timer
        timeoutTimer.reset();
        //set the timeout to false since we have not timed out yet
        timeout=false;

        //get the currentTicks and the targetTicks from the subsystem
        currentTicks = liftSubsystem.getCurrentTicks();
        targetTicks = liftSubsystem.getTargetTicks();

        //Check if targetTicks is greater than MAX_TARGET_TICKS and if it is set the target to the max
        //This makes sure that if we accidentally put a very large number as our target ticks we don't break the robot
        if (targetTicks> LIFT_PARAMS.MAX_TARGET_TICKS)
        {
            targetTicks= LIFT_PARAMS.MAX_TARGET_TICKS;
        }

        //Check if targetTicks is lower than MIN_TARGET_TICKS and if it is set the target to the min
        //This makes sure that if we accidentally put a very low negative number as our target ticks we don't break the robot
        if (targetTicks < LIFT_PARAMS.MIN_TARGET_TICKS)
        {
            targetTicks= LIFT_PARAMS.MIN_TARGET_TICKS;
        }

        //if the target ticks are higher than the current ticks, then use EXTENSION_POWER
        if (targetTicks > currentTicks) {
            liftSubsystem.lift.setPower(LIFT_PARAMS.EXTENSION_LIFT_POWER);
        }

        //if the target ticks are lower than the current ticks, then use RETRACTION_POWER
        if (targetTicks < currentTicks) {
            liftSubsystem.lift.setPower(LIFT_PARAMS.RETRACTION_LIFT_POWER);
        }

        //Set the target position using the targetTicks
        liftSubsystem.lift.setTargetPosition(targetTicks);

        //set the lift motor to RUN TO POSITION - this might not be necessary
        liftSubsystem.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void execute() {
        //The execute of this command is empty because we don't need to do anything while the command is active
        //the RUN_TO_POSITION we set in the intialize means that the lift motor is going to automatically try to reach the target
        //there is telemetry in the periodic() method of the lift subsystem that will give us updates in the dashboard about the current ticks and current state
    }

    @Override
    public boolean isFinished() {
        // Declare a boolean variable (e.g., finished)
        // Compare the currentTicks to the targetTicks to a threshold (LIFT_HEIGHT_TICK_THRESHOLD) and save as the boolean
        // For example, say our target is 2000 ticks and we are at 1997 - we would want that to count as being close enough
        boolean finished = Math.abs(liftSubsystem.getCurrentTicks() - liftSubsystem.getTargetTicks()) < LIFT_PARAMS.LIFT_HEIGHT_TICK_THRESHOLD;

        //write an if statement to change the currentState to the targetState and return true if the finished boolean is true
        if (finished){
            //if the command is finished, then return true (meaning the command will stop running once it runs end() one time)
            return true;
        }

        //compare the elapsed time to a timeout threshold and if the elapsed time is greater than the threshold return true
        timeout = timeoutTimer.seconds() > LIFT_PARAMS.TIMEOUT_TIME_SECONDS;
        if (timeout){
            return true;
        }

        //if the command isn't finished and the command isn't timed out then return false because the command should still run
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //write an if statement that tells the user the command didn't finish normally but instead timed out
        if (!timeout && !interrupted)
        {
            //Report the command finished
            MatchConfig.telemetryPacket.addLine("LiftSlide Move COMPLETE From " + liftSubsystem.getCurrentState() + " to " + liftSubsystem.getTargetState() + " in " + String.format("%.2f", timeoutTimer.seconds()) + " seconds");
            //change the current state to the target state
            liftSubsystem.setCurrentState(liftSubsystem.getTargetState());
        }
        if (timeout){
            //Put the target state in the packet
            MatchConfig.telemetryPacket.addLine("LiftSlide Move TIMEOUT");
            MatchConfig.telemetryPacket.put("Timeout Timer", timeoutTimer.seconds());
        }
        if (interrupted){
            //Put the target state in the packet
            MatchConfig.telemetryPacket.addLine("LiftSlide Move INTERRUPTED From " + liftSubsystem.getCurrentState() + " to " + liftSubsystem.getTargetState() + " at " + timeoutTimer.seconds() + " seconds");
        }
    }
}
