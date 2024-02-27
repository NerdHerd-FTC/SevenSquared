package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// needs another edition to go through the center truss
public class FarSideRedCenterDoor {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, 3.114857287413855, Math.toRadians(190.94804165608335), 19)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -61, Math.toRadians(90)))
                                .forward(40.5)
                                .back(33)
                                // drop arm
                                .turn(Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(-34, -31))
                                .lineToConstantHeading(new Vector2d(-46, -29.5))
                                // pickup
                                // bring arm back up
                                .lineToConstantHeading(new Vector2d(-55, -29.5))
                                .strafeRight(29)
                                .lineToSplineHeading(new Pose2d(30, -1, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(67, -31.5), Math.toRadians(0))
                                .strafeTo(new Vector2d(56, -16))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}