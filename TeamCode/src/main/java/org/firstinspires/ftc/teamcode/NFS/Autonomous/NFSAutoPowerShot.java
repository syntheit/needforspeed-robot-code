package org.firstinspires.ftc.teamcode.NFS.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NFS.GTP.GoToPoint;
import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Robot;
import org.firstinspires.ftc.teamcode.NFS.Vision.Vision;
import org.firstinspires.ftc.teamcode.NFS.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NFS.drive.SampleMecanumDrive;

import java.util.Arrays;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * The high goal (backup) autonomous
 * Zero Ring: 71 Points
 * One Ring: 83 Points
 * Four Rings: 119 Points
 */
@Disabled
@Config
@Autonomous(name = "NFSAutoPowerShot", group = "Linear Opmode")
public class NFSAutoPowerShot extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean runIntake;
        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(hardwareMap, telemetry, true);
        Vision vision = new Vision(hardwareMap, telemetry);
        robot.startPositions();
        SampleMecanumDrive drive = robot.drivetrain.mecanumDrive;
        GoToPoint point = new GoToPoint(drive, hardwareMap);
        Pose2d startPose = new Pose2d(-63, -44, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Trajectory[][] traj = new Trajectory[3][12];

        Trajectory firstWobbleDrop0 = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-2, -57), Math.toRadians(0))
                // .addTemporalMarker(0.1, () -> robot.flap.goToSlowPowershotPosition())
                .addTemporalMarker(.75, () -> robot.intake.down())
                .addTemporalMarker(.75, () -> robot.arm.down())
                .addTemporalMarker(1.85, () -> robot.claw.open())
                .addTemporalMarker(1.9, () -> robot.arm.up())
                .build();

        traj[0][1] = drive.trajectoryBuilder(firstWobbleDrop0.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .splineTo(new Vector2d(-6, -18), Math.toRadians(-10))
                .addTemporalMarker(.1, () -> drive.flywheels.doMaxVelocity())
                //  .addDisplacementMarker(() -> robot.flicker.launch())
                .build();
        Trajectory wobbleGrab0 = drive.trajectoryBuilder(new Pose2d(-6, -11.5, Math.toRadians(-5)), false)
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                .addTemporalMarker(1.5, () -> robot.arm.down())
                .splineTo(new Vector2d(-32.0, -20.0), Math.toRadians(-120.0))
                //.splineTo(new Vector2d(-32, -24), Math.toRadians(-132))
                .build();

        Trajectory wobbleGrab0New = drive.trajectoryBuilder(new Pose2d(-6, -11.5, Math.toRadians(180)), false)
                .splineToConstantHeading(new Vector2d(-30.4, -37), Math.toRadians(180))
                .addTemporalMarker(.1, () -> robot.arm.down())
                .build();

        Trajectory secondWobbleDrop0 = drive.trajectoryBuilder(wobbleGrab0New.end(), false)
                .splineTo(new Vector2d(-5, -48), Math.toRadians(-20))
                .addTemporalMarker(1.75, () -> robot.arm.down())
                .build();
        traj[0][4] = drive.trajectoryBuilder(secondWobbleDrop0.end(), true) //not used
                .splineTo(new Vector2d(-12, -36), Math.toRadians(-45))
                .build();
        traj[0][5] = drive.trajectoryBuilder(secondWobbleDrop0.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .splineTo(new Vector2d(12, -20), Math.toRadians(0))
                .build();

















        Trajectory firstWobbleDrop4 = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(0))
                .splineTo(new Vector2d(40, -55), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> robot.flap.setPosition(.285))
                .addTemporalMarker(.75, () -> robot.intake.down())
                .addTemporalMarker(2, () -> robot.arm.down())
                .build();

        traj[2][1] = drive.trajectoryBuilder(firstWobbleDrop4.end(), true)
                .splineTo(new Vector2d(-6, -18), Math.toRadians(160))
                .addTemporalMarker(.1, () -> drive.flywheels.doMaxVelocity())
                .build();

        Trajectory goToStack = drive.trajectoryBuilder(new Pose2d(-6, -11.5, Math.toRadians(-5.5)), true)
                .addTemporalMarker(.1, () -> {
                    //robot.intake.start();
                    drive.flywheels.halt();
                })
                .splineTo(new Vector2d(-24, -18), Math.toRadians(270) /*,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL) */)
                .build();

        Trajectory intakeThreeRings4 = drive.trajectoryBuilder(goToStack.end(), true)
               .splineTo(new Vector2d(-24, -32), Math.toRadians(270),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineTo(new Vector2d(-24, -44), Math.toRadians(270),
                .splineTo(new Vector2d(-24, -40), Math.toRadians(270),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(.5, () -> robot.intake.start())
                .addDisplacementMarker(() -> robot.intake.stop())
                .build();

        Trajectory wobbleGrab4 = drive.trajectoryBuilder(intakeThreeRings4.end().plus(new Pose2d(0, 0, Math.toRadians(70))))
                .splineTo(new Vector2d(-35, -39), Math.toRadians(165))
                .addDisplacementMarker(() -> {
                    robot.claw.close();
                    robot.intake.stop();
                })
                .build();
        Trajectory wobbleGrab4New = drive.trajectoryBuilder(intakeThreeRings4.end().plus(new Pose2d(0, 0, Math.toRadians(70))))
                .splineToConstantHeading(new Vector2d(-37, -39), Math.toRadians(165))
                .addDisplacementMarker(() -> {
                    robot.claw.close();
                    robot.intake.stop();
                })
                .build();

        Trajectory shootThreeHigh4 = drive.trajectoryBuilder(wobbleGrab4New.end().plus(new Pose2d(0, 0, Math.toRadians(-165))), false)
                .splineTo(new Vector2d(-4.5, -40), Math.toRadians(-.5))
                .addTemporalMarker(.1, () -> {
                    drive.flywheels.doMaxVelocity();
                    robot.flap.goToHighGoalPosition();
                })
                .build();

        Trajectory intakeOneRing4 = drive.trajectoryBuilder(shootThreeHigh4.end(), true)
                .splineTo(new Vector2d(-30, -55), Math.toRadians(-139))
                .addTemporalMarker(.1, () -> robot.intake.start())
                .build();

        Trajectory shootOneHigh4 = drive.trajectoryBuilder(intakeOneRing4.end(), false)
                .splineTo(new Vector2d(-4.5, -40), Math.toRadians(0))
                .addDisplacementMarker(() -> robot.intake.stop())
                .addTemporalMarker(.1, () -> drive.flywheels.doMaxVelocity())
                .build();

        Trajectory secondWobbleDrop4 = drive.trajectoryBuilder(shootOneHigh4.end(), false)
                .splineTo(new Vector2d(44, -50), Math.toRadians(-20))
                .addTemporalMarker(.75, () -> robot.arm.down())
                .addDisplacementMarker(() -> robot.claw.open())
                .build();

        Trajectory park = drive.trajectoryBuilder(secondWobbleDrop4.end(), true)
                .splineTo(new Vector2d(12, -36), Math.toRadians(180))
                .addTemporalMarker(.1, () -> {
                    robot.arm.up();
                })
                .build();

        while (!isStarted() && vision.isNotReadyToRead()) {
            telemetry.addData("Vision Status", "Not Ready");
            telemetry.update();
        }
        UGContourRingPipeline.Height height = vision.getHeight();
        while (!isStarted() && vision.isNotReady()) {
            height = vision.getHeight();
            telemetry.addData("Vision Status", "Not Ready");
            telemetry.addData("Camera Initialization Time: ", vision.cameraInitTime());
            telemetry.update();
        }
        while (!isStarted()) {
            height = vision.getHeight();
            telemetry.addData("Vision Status", "Ready");
            telemetry.addData("Ring count: ", height);
            telemetry.addData("Camera Initialization Time: ", vision.cameraInitTime());
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        drive.setPoseEstimate(startPose);
        switch (height) {
            case ZERO:
                telemetry.addData("There are no rings", "");
                vision.stopStreaming();

                //wobble drop
                drive.followTrajectory(firstWobbleDrop0);

                Trajectory reverse1 = drive.trajectoryBuilder(firstWobbleDrop0.end())
                        .back(6)
                        .build();
                drive.followTrajectory(reverse1);
//                point.notDone();
//                while (!point.isDone4) point.goToPointAuto(-8, -57, 0);

                //first ps
                point.notDone();
                robot.flap.goToSlowPowershotPosition();
                robot.flywheels.doPowershotSlowVelocity();
                while (!point.isDone4) point.goToPointPS(-6, -11.5, 6, .5, .5);
                robot.delayWithAllPID(300);
                robot.flicker.launch();

                //second ps
                point.notDone();
                timer.reset();
                while (!point.isDone4 && timer.milliseconds() < 1000)
                    point.goToPointPS(-6, -11.5, 0, .5, .5);
                robot.delayWithAllPID(300);
                robot.flicker.launch();

                //third ps
                point.notDone();
                timer.reset();
                while (!point.isDone4 && timer.milliseconds() < 1000)
                    point.goToPointPS(-6, -11.5, -5, .5, .5);
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(30);
                point.notDone();
                drive.flywheels.halt();
                timer.reset();
                while (!point.isDone4 && timer.milliseconds() < 1000)
                    point.goToPointPS(-6, -11.5, 180, .5, .5);


                //wobble grab
                drive.followTrajectory(wobbleGrab0New);
                robot.claw.close();
                robot.delayWithAllPID(300);
                robot.arm.up();
                robot.delayWithAllPID(750);

                //wobble drop
                drive.followTrajectory(secondWobbleDrop0);
                robot.claw.open();
                robot.delayWithAllPID(200);
                robot.arm.up();
                robot.delayWithAllPID(750);
                robot.claw.close();


                Trajectory reverse2 = drive.trajectoryBuilder(secondWobbleDrop0.end())
                        .back(6)
                        .build();
                drive.followTrajectory(reverse2);


                //park
                point.notDone();
                timer.reset();
                while (!point.isDone4 && timer.milliseconds() < 2000)
                    point.goToPointExtra(5, -30, 0, 2, 2, .5, 2, 5, .4, .4, .75);

                break;



            case ONE:
                telemetry.addData("There is one ring", "");
                vision.stopStreaming();

                //1st wobble drop
    Trajectory firstWobbleDrop1 = drive.trajectoryBuilder(startPose, false)
            .splineTo(new Vector2d(-12, -55), Math.toRadians(0))
            .addTemporalMarker(.75, () -> robot.intake.down())
            // .splineTo(new Vector2d(17.5, -42), Math.toRadians(30))
            .splineTo(new Vector2d(20, -41), Math.toRadians(30))
            .addDisplacementMarker( () -> robot.arm.down())
            .build();


                drive.followTrajectory(firstWobbleDrop1);
                robot.delayWithAllPID(700);
                robot.claw.open();
                robot.delayWithAllPID(150);
                robot.arm.up();
                robot.delayWithAllPID(50);


                //first ps
                point.notDone();
                robot.flap.goToSlowPowershotPosition();
                robot.flywheels.doPowershotSlowVelocity();
                while (!point.isDone4) point.goToPointPS(-6, -11.5, 6, .5, .5);
                robot.delayWithAllPID(300);
                robot.flicker.launch();

                //second ps
                point.notDone();
                timer.reset();
                while (!point.isDone4 && timer.milliseconds() < 1000)
                    point.goToPointPS(-6, -11.5, 0, .5, .5);
                robot.delayWithAllPID(300);
                robot.flicker.launch();

                //third ps
                point.notDone();
                timer.reset();
                while (!point.isDone4 && timer.milliseconds() < 1000)
                    point.goToPointPS(-6, -11.5, -5, .5, .5);
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(30);
                point.notDone();
                drive.flywheels.halt();


                drive.followTrajectory(goToStack);

                Trajectory intakeOneRing1 = drive.trajectoryBuilder(goToStack.end(), true)
                        .splineTo(new Vector2d(-24, -40), Math.toRadians(270),
                                new MinVelocityConstraint(Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                        .addTemporalMarker(.01, () -> robot.intake.start())
                        .build();

                drive.followTrajectory(intakeOneRing1);



                robot.arm.down();
                drive.turn(Math.toRadians(90));

                Trajectory wobbleGrab1 = drive.trajectoryBuilder(intakeOneRing1.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                        .splineToConstantHeading(new Vector2d(-20, -35), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-35, -38), Math.toRadians(180))
                        .addDisplacementMarker(() -> {
                            robot.intake.stop();
                        })
                        .build();

                drive.followTrajectory(wobbleGrab1);
                robot.delayWithAllPID(300);
                robot.claw.close();
                robot.delayWithAllPID(300);
                robot.arm.up();
                robot.delayWithAllPID(150);
                //point.turn(-165);
                // point.goToPoint(-37, -40, 0);
                drive.turn(Math.toRadians(-165));

                //shoot high (3 rings)
                drive.followTrajectory(shootThreeHigh4);
                robot.delayWithAllPID(20);
                robot.flicker.launch();

                Trajectory secondWobbleDrop1 = drive.trajectoryBuilder(shootThreeHigh4.end(), false)
                       .lineToLinearHeading(new Pose2d(12, -36, 0))
                        .addTemporalMarker(.5, () ->  robot.arm.down())
                        .build();

                drive.followTrajectory(secondWobbleDrop1);
                robot.delayWithAllPID(500);
                robot.claw.open();
                robot.delayWithAllPID(300);
                robot.arm.up();
                robot.delayWithAllPID(300);

                //park
                point.notDone();
                while (!point.isDone4) point.goToPointAuto(10, -36, 0);


                break;


            case FOUR:
                telemetry.addData("There are four rings", "");
                vision.stopStreaming();

                //first wobble drop
                drive.followTrajectory(firstWobbleDrop4);
                robot.claw.open();
                robot.delayWithAllPID(50);
                robot.arm.up();
                robot.delayWithAllPID(50);

                //first ps
                point.notDone();
                //robot.flap.goToSlowPowershotPosition();
                robot.flap.goToAutoPowershotPosition();
                robot.flywheels.doPowershotSlowVelocity();
                while (!point.isDone4) point.goToPointPS(-6, -11.5, 6, .5, .5);
                robot.delayWithAllPID(300);
                robot.flicker.launch();

                //second ps
                robot.flywheels.setTargetVelocity(1250);
                point.notDone();
                while (!point.isDone4) point.goToPointPS(-6, -11.5, 0, .5, .4);
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                timer.reset();

                //third ps
                robot.flap.setPosition(.277);
                robot.flywheels.setSlowPowerShotVelocity();
                point.notDone();
                while (!point.isDone4 && timer.milliseconds() < 2000) point.goToPointPS(-6, -11.5, -5.5, .5, .5);
                robot.delayWithAllPID(300);
                robot.flicker.launch();

                //intake stack (3 rings)
                drive.followTrajectory(goToStack);
                drive.followTrajectory(intakeThreeRings4);

                //wobble grab
                robot.arm.down();
                drive.turn(Math.toRadians(70));
                robot.intake.start();
                drive.followTrajectory(wobbleGrab4New);
                robot.delayWithAllPID(300);
                robot.arm.up();
                robot.delayWithAllPID(70);
                //point.turn(-165);
                // point.goToPoint(-37, -40, 0);
                drive.turn(Math.toRadians(-165));

                //shoot high (3 rings)
                drive.followTrajectory(shootThreeHigh4);
                robot.delayWithAllPID(20);
                robot.flicker.launch();
                robot.delayWithAllPID(200);
                robot.flicker.launch();
                robot.delayWithAllPID(200);
                robot.flicker.launch();
                robot.delayWithAllPID(30);
                drive.flywheels.halt();

                //wobble drop


                /*
                timer.reset();
                point.notDone();
                while (!point.isDone4) {
                    point.goToPointExtra(44, -47, 0, 2, 1.5, 1, 5, 5, .5, .5, 1);
                    if (timer.milliseconds() > 50 && robot.flywheels.running())
                        drive.flywheels.halt();
                    if (Math.abs(point.xError) < 3 && Math.abs(point.yError) < 3 && Math.abs(point.headingError) < Math.toRadians(5) && robot.arm.isUp())
                        robot.arm.down();
                }


                 */

                Trajectory wobbleDrop4 = drive.trajectoryBuilder(shootThreeHigh4.end())
                        .splineToConstantHeading(new Vector2d(44, -47), 0) //todo split spline and do velo constraints
                        .addDisplacementMarker(() -> robot.arm.down())  //todo convert to temporal
                        .build();
                drive.followTrajectory(wobbleDrop4);
                //robot.arm.down();
                //robot.delayWithAllPID(600);
                robot.delayWithAllPID(400);
                robot.claw.open();
                robot.delayWithAllPID(40);
                robot.arm.up();
                robot.delayWithAllPID(30);

                //intake stack
                point.notDone();
                timer.reset();
                robot.intake.start();
               Trajectory intakeStack4 = drive.trajectoryBuilder(wobbleDrop4.end(), true)
                       .lineToLinearHeading(new Pose2d(-50, -60, Math.toRadians(20)))
                       .build();
               drive.followTrajectory(intakeStack4);

                /*
                point.notDone();
                while (!point.isDone4) {
                    //point.goToPointAuto(-40, -60, 3);
                    double maxPower;
                    if(point.xError < 10) maxPower = .6;
                    else maxPower = 1;
                    point.goToPointExtra(-40, -60, 3, 1.5, 1.5, 1.5, 4, 5, .4, .4, maxPower);
                    //  if(timer.milliseconds()>500 && !robot.intake.isRunning()) robot.intake.start();

                }


                 */

                //drive to shoot
                point.notDone();
                timer.reset();
                robot.flap.goToHighGoalPosition();
                robot.flywheels.doMaxVelocity();
                runIntake = false;
                Trajectory shootHigh4 = drive.trajectoryBuilder(intakeStack4.end())
                        .lineToLinearHeading(new Pose2d(-4.5, -40, Math.toRadians(1)))
                        .addTemporalMarker(.5, () -> robot.intake.stop())
                        .build();
                drive.followTrajectory(shootHigh4);
                /*
                while (!point.isDone4) {
                    point.goToPointAuto(-4.5, -40, 0);
//                    if(timer.milliseconds()>300 && robot.intake.isRunning() && !runIntake){
//                        robot.intake.stop();
//                        //robot.intake.reverse();
//                        runIntake = true;
//                    }
                }
                robot.intake.stop();

                 */

                //shoot
                robot.delayWithAllPID(20);
                robot.flicker.launch();
                robot.delayWithAllPID(200);
                robot.flicker.launch();
                robot.delayWithAllPID(200);
                robot.flicker.launch();
                robot.delayWithAllPID(30);
                drive.flywheels.halt();

                //park
                point.notDone();
                timer.reset();
                while (!point.isDone4 && timer.milliseconds() < 2000)
                    point.goToPointExtra(10, -36, 0, 2, 2, .5, 2, 5, .4, .4, .75);
                //point.goToPointAuto(12, -36, 0);

                break;
        }
        telemetry.update();
    }
}