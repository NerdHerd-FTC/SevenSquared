package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PixelPickUpCloseRedRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, 3.114857287413855, Math.toRadians(190.94804165608335), 19)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                                .splineTo(new Vector2d(20, -30), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(23, -48), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(51, -36, Math.toRadians(0)), Math.toRadians(0))
                                .strafeLeft(24)
                                .turn(Math.toRadians(180))
                                .splineTo(new Vector2d(-10, -3), Math.toRadians(180))
                                .splineTo(new Vector2d(-47.5, -28), Math.toRadians(180))
                                .turn(Math.toRadians(180))
                                .splineTo(new Vector2d(-16, -10), Math.toRadians(0))
                                .splineTo(new Vector2d(20, -13), Math.toRadians(0))
                                .splineTo(new Vector2d(54, -24), Math.toRadians(0))
                                .strafeRight(38)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
