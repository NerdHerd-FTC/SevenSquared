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
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 63, Math.toRadians(270)))
                                //.forward(42.5)
                                .splineTo(new Vector2d(-38, 20), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(-38, 49), Math.toRadians(270))
                                .splineToSplineHeading(new Pose2d(-38, 50, Math.toRadians(180)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-52, 50), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(-52, 38), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(-52, 20), Math.toRadians(270))
                                .waitSeconds(3)
                                .splineToSplineHeading(new Pose2d(67, 30.5, Math.toRadians(360)), Math.toRadians(180))
                                .splineTo(new Vector2d(64, 0), Math.toRadians(180))


                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}