package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_LOW_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_HOME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.JOINT_AVOID_CUBE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.joint_norm_F;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.joint_ticks_per_degree;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="path follower")
public class PathFollower extends LinearOpMode {
    private enum steps {
        one,
        two,
        done
    }


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 270 degrees
        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        Trajectory stepOne = drive.trajectoryBuilder(startPose)
                .forward(35.56)
                .build();

        Trajectory stepTwo = drive.trajectoryBuilder(stepOne.end())
                .splineToConstantHeading(new Vector2d(12, 50), Math.toRadians(270))
                .splineTo(new Vector2d(45.55748457301436, 36.53054245151751), Math.toRadians(0))
                .build();

        steps step = steps.one;
        drive.followTrajectoryAsync(stepOne);

        waitForStart();

        while (opModeIsActive() && step != steps.done) {
            switch (step) {
                case one:
                    drive.update();

                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(stepTwo);
                        step = steps.two;
                    }
                case two:
                    drive.update();

                    if (!drive.isBusy()) {
                        step = steps.done;
                    }
            }
        }

    }
}