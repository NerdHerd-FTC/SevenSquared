package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SlowRedFarCenter {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, 3.114857287413855, Math.toRadians(190.94804165608335), 19)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -61, Math.toRadians(90)))
                                .forward(43)
                                .back(33)
                                .splineToLinearHeading(new Pose2d(-45, -31, Math.toRadians(180)), Math.toRadians(180))
                                .strafeRight(3)
                                .splineToSplineHeading(new Pose2d(70, -31.5,  Math.toRadians(0)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(70, -3), Math.toRadians(0))
                                .turn(Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}