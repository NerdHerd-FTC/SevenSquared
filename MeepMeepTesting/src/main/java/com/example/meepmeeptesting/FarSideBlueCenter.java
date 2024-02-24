package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FarSideBlueCenter {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, 3.114857287413855, Math.toRadians(190.94804165608335), 19)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 61, Math.toRadians(270)))
                                .splineTo(new Vector2d(-34, 25), Math.toRadians(270))
                                .back(20)
                                .strafeRight(5)
                                .splineToSplineHeading(new Pose2d(-60, 10, Math.toRadians(180)), Math.toRadians(180))
                                //.lineToSplineHeading(new Pose2d(-60, -10, Math.toRadians(180)))
                                //.splineToLinearHeading(new Pose2d(-60, -10, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)
                                .back(90)
                                .lineToSplineHeading(new Pose2d(49, 17, Math.toRadians(0)))
                                .strafeLeft(28)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}