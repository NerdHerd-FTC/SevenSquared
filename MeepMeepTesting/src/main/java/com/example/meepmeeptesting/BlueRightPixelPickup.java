package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class BlueRightPixelPickup { //closest to backdrop
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, 3.114857287413855, Math.toRadians(190.94804165608335), 19)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 61.5, Math.toRadians(270)))
                                // .splineTo(new Vector2d(12, 34), Math.toRadians(90))
                                // .splineTo(new Vector2d(12, 52), Math.toRadians(90))
                                .splineTo(new Vector2d(5.5, 34), Math.toRadians(180))
                                .back(20)
                                // .splineToConstantHeading(new Vector2d(20, 34), Math.toRadians(180))
                                //.splineToConstantHeading(new Vector2d(20, 34), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(57, 24), Math.toRadians(0))
                                .strafeRight(21)
                                .back(80)
                                .lineToSplineHeading(new Pose2d(-50, 10, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-57, 34), Math.toRadians(180))// separate trajectory
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}