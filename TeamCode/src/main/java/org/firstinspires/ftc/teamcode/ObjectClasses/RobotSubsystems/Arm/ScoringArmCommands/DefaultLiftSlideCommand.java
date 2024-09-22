package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.ScoringArmCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSubsystem.LIFT_PARAMS;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultLiftSlideCommand extends CommandBase {

    //Declare the local variable to hold the liftsubsystem
    private final LiftSubsystem liftSubsystem;

    //Declare the local variable to hold the lift stick
    private DoubleSupplier liftSupplier;

    public DefaultLiftSlideCommand(LiftSubsystem subsystem, DoubleSupplier liftStick) {
        liftSubsystem = subsystem;
        liftSupplier = liftStick;

        //add the subsystem to the requirements
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        //Set the target ticks to the current ticks so that movements are relative to where the lift is right now
        liftSubsystem.setTargetTicks(liftSubsystem.getCurrentTicks());

    }

    public void execute() {
        //if liftSupplier is positive and above deadzone, then use Extension power
        if (liftSupplier.getAsDouble() > LIFT_PARAMS.LIFT_DEAD_ZONE_FOR_MANUAL_LIFT) {
            liftSubsystem.setCurrentState(LiftSubsystem.LiftStates.MANUAL);
            liftSubsystem.lift.setPower(LIFT_PARAMS.EXTENSION_LIFT_POWER);
        }

        //if liftSupplier is negative and lower than deadzone, then use Retraction power
        else if (liftSupplier.getAsDouble() < -LIFT_PARAMS.LIFT_DEAD_ZONE_FOR_MANUAL_LIFT) {
            liftSubsystem.setCurrentState(LiftSubsystem.LiftStates.MANUAL);
            liftSubsystem.lift.setPower(LIFT_PARAMS.RETRACTION_LIFT_POWER);
        }

        //if we are in the dead zone we should keep the extension lift power to maintain our position
        else {
            liftSubsystem.lift.setPower(LIFT_PARAMS.EXTENSION_LIFT_POWER);
        }

        //Use the liftSupplier to calculate the new target based on the liftSupplier
        liftSubsystem.setTargetTicks(calculateNewTargetTicks(liftSupplier));

        //set the target position to the new ticks value
        liftSubsystem.lift.setTargetPosition(liftSubsystem.getTargetTicks());

        //set the lift motor to RUN TO POSITION - this might not be necessary
        liftSubsystem.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public boolean isFinished() {
        //this command should never finish
        return false;
    }

    private int calculateNewTargetTicks(DoubleSupplier liftSupplier) {
        // Update the targetTicks based on the liftSupplier's value
        int deltaTicks = (int) Math.round(liftSupplier.getAsDouble()* LIFT_PARAMS.SCALE_FACTOR_FOR_MANUAL_LIFT);

        int newTargetTicks = liftSubsystem.getCurrentTicks() + deltaTicks;

        // Ensure the new target is within the range of MIN_TICKS to MAX_TICKS using Range.clip
        int clippedNewTargetTicks = (int) Range.clip(newTargetTicks, LIFT_PARAMS.MIN_TARGET_TICKS, LIFT_PARAMS.MAX_TARGET_TICKS);

        return clippedNewTargetTicks;
    }
}
