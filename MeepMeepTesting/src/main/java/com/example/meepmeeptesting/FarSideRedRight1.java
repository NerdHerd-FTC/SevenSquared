package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// needs another edition going through the center truss
public class FarSideRedRight1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, 3.114857287413855, Math.toRadians(190.94804165608335), 19)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -61, Math.toRadians(90)))
                                .forward(11)
                                .splineTo(new Vector2d(-26, -35), Math.toRadians(0))
                               // .splineToLinearHeading(new Pose2d(-26, -28, Math.toRadians(90)), Math.toRadians(180))
                                .back(13)
                                .lineToSplineHeading(new Pose2d(-57, -34, Math.toRadians(180)))
                                .waitSeconds(1)
                                .strafeRight(6)
                                .splineTo(new Vector2d(34, -10), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(55, -37, Math.toRadians(0)), Math.toRadians(0))
                                .strafeRight(30)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}