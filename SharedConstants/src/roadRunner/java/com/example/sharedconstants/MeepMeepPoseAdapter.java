package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.sharedconstants.Pose2dVector2dAdapter;

public class MeepMeepPoseAdapter implements Pose2dVector2dAdapter {

    @Override
    public Pose2d createPose(double x, double y, double heading) {
        return new Pose2d(x, y, heading);
    }

    @Override
    public Vector2d createVector(double x, double y) {
        return new Vector2d(x, y);
    }
}