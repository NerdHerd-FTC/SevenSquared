package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FarSideBlueRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, 3.114857287413855, Math.toRadians(190.94804165608335), 19)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-30, 63, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-45, 30, Math.toRadians(180)), Math.toRadians(180))
                                .back(11)
                                .lineToConstantHeading(new Vector2d(-34, 5.95))
                                .lineToConstantHeading(new Vector2d(-55, 5.95))
                                .strafeLeft(12)
                                .lineToSplineHeading(new Pose2d(30, 6, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(67, 17), Math.toRadians(0))
                                .strafeTo(new Vector2d(64, 56))
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