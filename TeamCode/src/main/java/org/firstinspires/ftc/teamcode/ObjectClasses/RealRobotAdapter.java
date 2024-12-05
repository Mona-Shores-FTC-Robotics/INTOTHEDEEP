package org.firstinspires.ftc.teamcode.ObjectClasses;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.RobotAdapter;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveActions.DriveForwardAndBack;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.ChangeSampleIntakePowerAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.ChangeSampleDumperPositionAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.MoveSampleLiftAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;
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
                        return new SequentialAction(
                                new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::setBucketToIntakePosition),
                                new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::moveLiftToHome));
                    } else return problem();

                case MOVE_PRELOAD_SPECIMEN_TO_CW_HOME:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM) && robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM)) {
                        return new InstantAction(robot.getSpecimenArmSubsystem()::flipCWFast);
                    } else return problem();



            case HANG_SPECIMEN_ON_HIGH_CHAMBER:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM) && robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
                        return new SequentialAction(
                                new InstantAction(robot.getSpecimenArmSubsystem()::flipCCWFast),
                                new InstantAction(robot.getSpecimenIntakeSubsystem()::disablePreloadMode),
                                new InstantAction(robot.getLightingSubsystem()::setLightBlack)
                        );
                    } else return problem();

                case PREPARE_TO_SCORE_IN_HIGH_BASKET:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
                        return new  SequentialAction(
                                        new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::moveLiftToHighBasket),
                                        new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::moveDumperToPreScore),
                                        new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::setBucketToScorePosition));
                    } else return problem();

                case PREPARE_TO_SCORE_IN_LOW_BASKET:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
                        return new  SequentialAction(
                                new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::moveLiftToLowBasket),
                                new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::moveDumperToPreScore),
                                new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::setBucketToScorePosition));
                    } else return problem();

                case SCORE_IN_BASKET:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_LIFT_BUCKET)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)
                            && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
                        return new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::dumpSampleInBucket);
                    } else return problem();


                case GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR))
                    {
                        return  new ParallelAction(
//                                new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::partiallyDeploy),
                                new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON),
                                new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::setFlipperHover),
                                new InstantAction(Robot.getInstance().getSampleTiwsterSubsystem()::setTwisterServoFaceOutwards)
                                );
                    } else return problem();

                case GET_READY_FOR_SAMPLE_INTAKE_FROM_GROUND_WITH_FULL_EXTENSION:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR))
                    {
                        return  new ParallelAction(
                                new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::fullyDeploy),
                                new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_ON),
                                new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::setFlipperHover),
                                new InstantAction(Robot.getInstance().getSampleTiwsterSubsystem()::setTwisterServoFaceOutwards)
                        );
                    } else return problem();

                case PICKUP_FROM_GROUND_WITHOUT_TRANSFER:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR))
                    {
                        return new SequentialAction(
                                new InstantAction(Robot.getInstance().getSampleIntakeSubsystem()::setAutomaticPickupFalse),
                                new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::setFlipperDown),
                                new SleepAction(.8),
                                new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF),
                                new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::partiallyRetract));

                    } else return problem();

                case DROP_OFF_GROUND_SAMPLE_WITHOUT_TRANSFER:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE)) {
                        return new SequentialAction(
                                new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::fullyDeploy),
                                new SleepAction(.2),
                                new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_REVERSE),
                                new SleepAction(.5),
                                new InstantAction(Robot.getInstance().getSampleIntakeSubsystem()::setAutomaticPickupTrue),
                                new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::partiallyRetract)
                                );
                    } else return problem();

                case PICKUP_FROM_GROUND:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR))
                    {
                        return  new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::setFlipperDown);
                    } else return problem();

                case GET_READY_FOR_SPECIMEN_INTAKE_FROM_WALL:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM) && robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE))
                    {
                        return new ConditionalAction(
                                new InstantAction(robot.getSpecimenArmSubsystem()::flipCWFast),
                                new ParallelAction(
                                        new InstantAction(()->Robot.getInstance().getSpecimenArmSubsystem().setTargetAngle(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_PICKUP)),
                                        new ChangeSpecimenIntakePowerAction(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_ON)),
                                Robot.getInstance().getSpecimenIntakeSubsystem().getSpecimenDetector()::haveSpecimen);
                    } else return problem();

                case SPECIMEN_PICKUP_RETRY:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM) && robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
                        Supplier<Action> redoSupplier = () -> new ConditionalAction(
                                new NullAction(),
                                new DriveForwardAndBack(7),
                                robot.getSpecimenIntakeSubsystem().getSpecimenDetector()::haveSpecimen
                        );
                        return new SequentialAction(redoSupplier.get(), redoSupplier.get(), redoSupplier.get());
                    } else return problem();

//            case INTAKE_SAMPLE_FROM_GROUND_AND_RETRACT:
//                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_INTAKE) && robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR))
//                    {
//                        return new SequentialAction(
//                                new ChangeSampleIntakePowerAction(SampleIntakeSubsystem.SampleIntakeStates.INTAKE_OFF),
//                                new SequentialAction(
//                                        new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::runWithoutEncodersReverse),
//                                        new SleepAction(.5),
//                                        new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::stopActuator),
//                                        new InstantAction(Robot.getInstance().getSampleButtonHandling()::setIntakeReverse),
//                                        new SleepAction(.6),
//                                        new InstantAction(Robot.getInstance().getSampleButtonHandling()::setIntakeOff)
//                                )
//                        );
//                    } else return problem();

//                case SAMPLE_ACTUATOR_RETRACT:
//                    try {
//                        if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
//                         return new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::fullyRetract);
//                        }
//                    } catch (Exception e) {
//                        throw new RuntimeException(e);
//                    }

                case FLIP_UP_AND_RETRACT:
                    if (robot.hasSubsystem(Robot.SubsystemType.SAMPLE_ACTUATOR)) {
                        return new InstantAction(Robot.getInstance().getSampleLinearActuatorSubsystem()::flipSampleIntakeUpAndRetract);
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

                case WAIT_FOR_SPECIMEN_INTAKE_FROM_WALL:
                    if (robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_ARM) && robot.hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE))
                    {
                        return new ConditionalWaitAction(
                                new NullAction(),  // this should make it so we just proceed when haveSpecimen is true
                                robot.getSpecimenIntakeSubsystem().getSpecimenDetector()::haveSpecimen,  //this is the condition upon which it performs the first action
                               100); // this is how often it checks the condition
                    }
                case LEVEL_1_ASCENT:
                {
                    return new InstantAction(Robot.getInstance().getSpecimenArmSubsystem()::level1Ascent);
                }

                case DEPOWER_ARM:
                {
                    return new InstantAction(Robot.getInstance().getSpecimenArmSubsystem()::depowerArm);
                }

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