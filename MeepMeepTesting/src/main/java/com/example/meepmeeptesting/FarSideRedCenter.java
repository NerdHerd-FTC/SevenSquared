package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// needs another edition to go through the center truss
public class FarSideRedCenter {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, 3.114857287413855, Math.toRadians(190.94804165608335), 19)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -61, Math.toRadians(90)))
                                .forward(36)
                                .back(19)
                                .splineToLinearHeading(new Pose2d(-53, -36, Math.toRadians(180)), Math.toRadians(180))
                               // .back(9)
                               // .lineToSplineHeading(new Pose2d(-50, -36, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-18, -59.25), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(20, -60), Math.toRadians(0))
                                //.splineTo(new Vector2d(-16, -58), Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(-65, -36, Math.toRadians(90)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(53, -37, Math.toRadians(0)), Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(53, -37, Math.toRadians(0)), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}