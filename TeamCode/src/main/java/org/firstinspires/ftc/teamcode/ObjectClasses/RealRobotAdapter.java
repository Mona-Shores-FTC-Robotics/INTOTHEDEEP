package org.firstinspires.ftc.teamcode.ObjectClasses;


import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingStateMachine.SAMPLE_HANDLING_PARAMS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveToObservationZone;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.ChangeSampleIntakePowerAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.ChangeSampleDumperPositionAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.MoveSampleLiftAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.ActionsAndCommands.MoveSpecimenArmAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.ChangeSpecimenIntakePowerAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

import java.util.function.Supplier;

public class RealRobotAdapter implements RobotAdapter {
    private final ActionFactory actionFactory;
    private FieldConstants.AllianceColor allianceColor;
    private FieldConstants.SideOfField sideOfField;

    public RealRobotAdapter() {
        actionFactory = new ActionFactory();
        this.setAllianceColor(MatchConfig.finalAllianceColor);
        this.setSideOfField(MatchConfig.finalSideOfField);
    }

    @Override
    public Action getAction(ActionType actionType) {
        return actionFactory.createAction(actionType);
    }

    @Override
    public TrajectoryActionBuilder getActionBuilder(Pose2d startPose) {
        if (isRotated()) {
            return rotatedActionBuilder(startPose);
        } else {
            return actionBuilder(startPose);
        }
   }

    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return Robot.getInstance().getDriveSubsystem().getMecanumDrive().actionBuilder(startPose);
    }

    public TrajectoryActionBuilder rotatedActionBuilder(Pose2d beginPose) {
        return Robot.getInstance().getDriveSubsystem().rotatedActionBuilder(beginPose);
    }

    public void setAllianceColor(FieldConstants.AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public void setSideOfField(FieldConstants.SideOfField sideOfField) {
        this.sideOfField = sideOfField;
    }

    // Check if the robot is on the blue alliance and therefore should use the rotated trajectory
    public boolean isRotated() {
        return this.allianceColor == FieldConstants.AllianceColor.BLUE;
    }

    // Inner ActionFactory class
    private static class ActionFactory {
        public Action createAction(ActionType actionType) {
            Robot robot = Robot.getInstance();
            DriverStationTelemetryManager telemetryManger = robot.getDriverStationTelemetryManager();

            switch (actionType) {
                case SAMPLE_INTAKE_ON:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
                        return new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON);
                    } else return problem();

                case SAMPLE_INTAKE_OFF:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
                        return new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF);
                    } else return problem();

                case SAMPLE_INTAKE_REVERSE:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
                        return new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_REVERSE);
                    } else return problem();

                case SPECIMEN_INTAKE_ON:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
                        return new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_ON);
                    } else return problem();

                case SPECIMEN_INTAKE_OFF:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
                        return new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
                    } else return problem();

                case SPECIMEN_INTAKE_REVERSE:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
                        return new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_REVERSE);
                    } else return problem();

                // Intake the preload for a moment to ensure its secured
                case SECURE_PRELOAD_SPECIMEN:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
                        return new SequentialAction(
                                new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_ON),
                                new SleepAction(0.3),
                                new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF)
                        );
                    } else return problem();

                case PICKUP_SPECIMEN_OFF_WALL_SENSOR:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {

                        Supplier<Boolean> specimenDetected = () -> Robot.getInstance().getSpecimenIntakeSubsystem().goodSpecimenDetected();
                        // Define the action if the specimen is detected
                        Action stopSpecimenIntakeAndStageSpecimen = new SequentialAction(
                                new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF),
                                new MoveSpecimenArmAction(SpecimenArmSubsystem.SpecimenArmStates.CW_ARM_HOME)
                        );

                        // Define the action if the specimen is NOT detected (keep intaking for 200 milliseconds)
                        Action continueSpecimenIntakeAction = new SequentialAction(
                                new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_ON),
                                new SleepAction(.2)
                        );

                        // Create the ConditionalAction
                        return new ConditionalAction(stopSpecimenIntakeAndStageSpecimen, continueSpecimenIntakeAction, specimenDetected);
                    } else return problem();
                case PICKUP_SPECIMEN_OFF_WALL:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {

                        Supplier<Boolean> specimenDetected = () -> Robot.getInstance().getSpecimenIntakeSubsystem().goodSpecimenDetected();
                        // Define the action if the specimen is detected
                        return new SequentialAction(
                                new SleepAction(.5),
                                new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF),
                                new MoveSpecimenArmAction(SpecimenArmSubsystem.SpecimenArmStates.CW_ARM_HOME)
                        );
                    } else return problem();

                case SPECIMEN_INTAKE_FROM_WALL:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {

                        return new SequentialAction(
                                new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_ON),
                                new MoveSpecimenArmAction(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_PICKUP)
                        );
                    } else return problem();

                case MOVE_PRELOAD_SPECIMEN_TO_CW_HOME:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
                        return robot.getSpecimenHandlingStateMachine().flipCWFastAction();
                    } else return problem();

                    //should there be a difference between these?
                case SPECIMEN_ARM_TO_HIGH_CHAMBER:
                case HANG_SPECIMEN_ON_HIGH_CHAMBER:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM) && robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
                        return robot.getSpecimenHandlingStateMachine().flipCCWFastAction();
                    } else return problem();

                case DEPOSIT_SAMPLE:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
                        return new DriveToObservationZone(3);
                    } else return problem();

                case SAMPLE_LIFT_TO_HIGH_BASKET:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
                        return new MoveSampleLiftAction(SampleLiftBucketSubsystem.SampleLiftStates.HIGH_BASKET);
                    } else return problem();

                case SAMPLE_LIFT_TO_LOW_BASKET:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
                        return new MoveSampleLiftAction(SampleLiftBucketSubsystem.SampleLiftStates.LOW_BASKET);
                    } else return problem();

                case SAMPLE_LIFT_TO_HOME:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
                        return new MoveSampleLiftAction(SampleLiftBucketSubsystem.SampleLiftStates.LIFT_HOME);
                    } else return problem();



                case GET_READY_FOR_INTAKE_FROM_GROUND:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR))
                    {
                        return  new ParallelAction(
                                        new SequentialAction(
                                                new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::runWithoutEncodersForward),
                                                new SleepAction(.3),
                                                new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::stopActuator)
                                                ),
                                        new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON)
                        );
                    } else return problem();

                case INTAKE_SAMPLE_FROM_GROUND_AND_RETRACT:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR))
                    {
                        return new SequentialAction(
                                new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF),
                                new SequentialAction(
                                        new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::runWithoutEncodersReverse),
                                        new SleepAction(.5),
                                        new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::stopActuator),
                                        new InstantAction(Robot.getInstance().getSampleHandlingStateMachine()::setIntakeReverse),
                                        new SleepAction(.6),
                                        new InstantAction(Robot.getInstance().getSampleHandlingStateMachine()::setIntakeOff)
                                )
                        );
                    } else return problem();

                case DUMP_SAMPLE_IN_OBSERVATION_ZONE:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET))
                    {
                        return new SequentialAction(
                                new ChangeSampleDumperPositionAction(SampleLiftBucketSubsystem.DumperStates.DUMPER_DUMP),
                                new SleepAction(.8),
                                new ChangeSampleDumperPositionAction(SampleLiftBucketSubsystem.DumperStates.DUMPER_HOME)
                        );
                    } else return problem();

                default:
                    telemetryManger.displayError("Unknown action type: " + actionType);
                    return new NullAction();
            }
        }

        private Action problem() {
//            Robot.getInstance().getDriverStationTelemetryManager().displayError("Subsystem Not Available");
            return new NullAction();
        }
    }
}