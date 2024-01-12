package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FarSideRedCenter {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, 3.114857287413855, Math.toRadians(190.94804165608335), 19)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -63, Math.toRadians(90)))
                                .splineTo(new Vector2d(-34, -25), Math.toRadians(90))
                                .back(15)
                                .lineToSplineHeading(new Pose2d(-55, -34, Math.toRadians(180)))
                                .waitSeconds(1)
                                .splineToSplineHeading(new Pose2d(-16, -3, Math.toRadians(0)), Math.toRadians(0))
                                .splineTo(new Vector2d(40, -16), Math.toRadians(0))
                                .splineTo(new Vector2d(53, -37), Math.toRadians(0))
                                //.splineToLinearHeading(new Vector2d(-55, -34), Math.toRadians(180))
                               // .splineToConstantHeading()
                                //.splineTo(new Vector2d(57, -30), Math.toRadians(0))
                                //.strafeRight(48) // separate trajectory
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}