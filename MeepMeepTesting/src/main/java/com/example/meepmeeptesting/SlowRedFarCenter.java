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
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -63, Math.toRadians(90)))
                                .forward(43)
                                .back(33)
                                .splineToLinearHeading(new Pose2d(-45, -31, Math.toRadians(180)), Math.toRadians(180))
                                .splineTo(new Vector2d(-46, -10), Math.toRadians(0))
                                .splineTo(new Vector2d(34, -10), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(51, -31.5, Math.toRadians(0)), Math.toRadians(0))
                                .strafeLeft(28)
                                .turn(Math.toRadians(180)) // Turns 45 degrees counter-clockwise
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}