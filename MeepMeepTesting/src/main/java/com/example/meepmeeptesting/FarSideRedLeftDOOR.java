package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// needs another edition to go through the center truss
public class FarSideRedLeftDOOR {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, 3.114857287413855, Math.toRadians(190.94804165608335), 19)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -61, Math.toRadians(90)))
                                .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(180)), Math.toRadians(180))
                                .back(11)
                                .lineToConstantHeading(new Vector2d(-34, -5.95))
                                .lineToConstantHeading(new Vector2d(-55, -5.95))
                                .strafeRight(12)
                                .lineToSplineHeading(new Pose2d(30, 6, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(67, -17), Math.toRadians(0))
                                .strafeTo(new Vector2d(64, 0))
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

/*
Trajectory center1 = drive.trajectoryBuilder(startPose)
                .forward(42.5)
                .build();

        Trajectory center2 = drive.trajectoryBuilder(center1.end())
                .back(33)
                .build();

        TrajectorySequence center2_turn = drive.trajectorySequenceBuilder(center2.end())
                .turn(Math.toRadians(90))
                .build();

        Trajectory center2_strafeToX = drive.trajectoryBuilder(center2_turn.end())
                .lineToConstantHeading(new Vector2d(-34, -31))
                .build();

        // Spline to pixel stack
        Trajectory center3 = drive.trajectoryBuilder(center2_strafeToX.end())
                .lineToConstantHeading(new Vector2d(x_end, y_end))
                .build();

        // Raise arm up before executing 3_ series of movements - one issue may be time for later autos
        // Calibration numbers need to be tuned
        Trajectory center3_1 = drive.trajectoryBuilder(center3.end())
                .forward(-1)
                .build();

        Trajectory center3_2 = drive.trajectoryBuilder(center3_1.end())
                .strafeLeft(1)
                .build();

        Trajectory center3_3 = drive.trajectoryBuilder(center3_2.end())
                .forward(1)
                .build();

        // moving to backdrop
        Trajectory center4 = drive.trajectoryBuilder(center3.end())
                .lineToConstantHeading(new Vector2d(-55, -29.95))
                .build();

        Trajectory center5 = drive.trajectoryBuilder(center4.end())
                .strafeRight(36)
                .build();

        Trajectory center6 = drive.trajectoryBuilder(center5.end())
                .lineToSplineHeading(new Pose2d(30, 6, Math.toRadians(0)))
                .build();

        Trajectory center7 = drive.trajectoryBuilder(center6.end())
                .splineToConstantHeading(new Vector2d(67, -31.5), Math.toRadians(0))
                .build();

        // move to left corner
        Trajectory cornerCenter = drive.trajectoryBuilder(center7.end())
                .strafeTo(new Vector2d(56, -16))
                .build();

        TrajectorySequence rotateCenter = drive.trajectorySequenceBuilder(cornerCenter.end())
                .turn(Math.toRadians(180)) // Turns 45 degrees counter-clockwise
                .build();
 */