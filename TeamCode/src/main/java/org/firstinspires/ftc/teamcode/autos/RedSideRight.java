package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.drive.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.helperclasses.RoadRunner;
import org.firstinspires.ftc.teamcode.signalfinding.GreenFinder;
import org.firstinspires.ftc.teamcode.signalfinding.OrangeFinder;
import org.firstinspires.ftc.teamcode.signalfinding.PurpleFinder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Red Right Auto", group = "Autonomous")
public class RedSideRight extends LinearOpMode
{
    int autoNumber = 1;

    boolean intakeCone, depositCone, counterSpin;

    Trajectory preloadGoal0, backAway1, loopSetup2, lineUp3, stackLineup4, stackIntake5, stackAway6, stackScore7, park1X, park2X, park3X;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    ElapsedTime matchTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        //camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        RoadRunner robot = new RoadRunner(hardwareMap);

        Pose2d startPose = new Pose2d(-64, -40);
        robot.setPoseEstimate(startPose);

        preloadGoal0 = robot.trajectoryBuilder(startPose)
                .addDisplacementMarker(() -> {
                    counterSpin = true;
                })
                .splineTo(new Vector2d(-50, -14), 0,
                        RoadRunner.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 3;
                })
                .splineTo(new Vector2d(-28, -5), Math.toRadians(55),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    counterSpin = false;
                    depositCone = true;
                })
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(backAway1);
                })
                .build();

        backAway1 = robot.trajectoryBuilder(preloadGoal0.end(), Math.toRadians(50))
                .strafeTo(new Vector2d(-36, -15),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                    robot.followTrajectoryAsync(loopSetup2);
                })
                .build();

        loopSetup2 = robot.trajectoryBuilder(backAway1.end())
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                })
                .splineTo(new Vector2d(-4, -17), Math.toRadians(310),
                        RoadRunner.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(lineUp3);
                })
                .build();

        lineUp3 = robot.trajectoryBuilder(loopSetup2.end())
                .strafeTo(new Vector2d(-13, -8),
                        RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                })
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(stackLineup4);
                })
                .build();

        stackLineup4 = robot.trajectoryBuilder(lineUp3.end())
                .splineToConstantHeading((new Vector2d(-15, -10)), Math.toRadians(270),
                        RoadRunner.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 1;
                })
                .splineTo(new Vector2d(-14, -40), Math.toRadians(270),
                        RoadRunner.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-10, -59, Math.toRadians(270)), Math.toRadians(270),
                        RoadRunner.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                    robot.followTrajectoryAsync(stackIntake5);
                })
                .build();

        stackIntake5 = robot.trajectoryBuilder(stackLineup4.end())
                .lineTo(new Vector2d(-10, -61))
                .addDisplacementMarker(() -> {
                    robot.intakeCone();
                    counterSpin = true;
                    robot.transferLevel = 1;
                    robot.followTrajectoryAsync(stackAway6);
                })
                .build();

        stackAway6 = robot.trajectoryBuilder(stackIntake5.end())
                .strafeTo(new Vector2d(-12, -8),
                        RoadRunner.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(stackScore7);
                })
                .build();

        stackScore7 = robot.trajectoryBuilder(stackAway6.end())
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 3;
                })
                .splineTo(new Vector2d(-4, -17), Math.toRadians(310),
                        RoadRunner.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    counterSpin = false;
                    depositCone = true;
                })
                .addDisplacementMarker(() -> {
                    if(autoNumber == 1)
                    {
                        if(matchTime.seconds() < 16)
                            robot.followTrajectoryAsync(lineUp3);
                        else
                            robot.followTrajectoryAsync(park1X);
                    }
                    else
                    if(matchTime.seconds() < 18)
                        robot.followTrajectoryAsync(lineUp3);
                    else
                    {
                        if(autoNumber == 3)
                            robot.followTrajectoryAsync(park3X);
                        else
                            robot.followTrajectoryAsync(park2X);
                    }
                })
                .build();

        park1X = robot.trajectoryBuilder(stackScore7.end())
                .strafeTo(new Vector2d(-13,-9))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                })
                .build();

        park2X = robot.trajectoryBuilder(stackScore7.end())
                .strafeTo(new Vector2d(-13,-8))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                })
                .splineTo(new Vector2d(-10,-35), Math.toRadians(270),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        park3X = robot.trajectoryBuilder(stackScore7.end())
            .strafeTo(new Vector2d(-13,-8))
            .addDisplacementMarker(() -> {
                robot.transferLevel = 0;
            })
            .splineTo(new Vector2d(-12,-35), Math.toRadians(270),
                    RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .splineTo(new Vector2d(-12,-60), Math.toRadians(270),
                    RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .build();

        //runs camera and associated programs
        while(!isStopRequested() && !isStarted())
        {
            double orangeScore = OrangeFinder.orangeScore;
            double greenScore = GreenFinder.greenScore;
            double purpleScore = PurpleFinder.purpleScore;

            webcam.setPipeline(new OrangeFinder());
            webcam.setPipeline(new GreenFinder());
            webcam.setPipeline(new PurpleFinder());

            if(greenScore >= orangeScore && greenScore >= purpleScore)
                autoNumber = 2;
            else if(purpleScore > orangeScore && purpleScore > greenScore)
                autoNumber = 3;
            else
                autoNumber = 1;

            dashboardTelemetry.addData("auto", autoNumber);
            dashboardTelemetry.update();

            telemetry.addData("auto", autoNumber);
            telemetry.update();
        }

        waitForStart();

        matchTime.reset();
        matchTime.startTime();

        robot.followTrajectoryAsync(preloadGoal0);

        while(opModeIsActive())
        {
            robot.update();

            robot.transfer();

            if(intakeCone)
            {
                robot.intakeCone();
                intakeCone = false;
            }

            if(depositCone)
            {
                robot.depositCone();
                depositCone = false;
            }

            robot.runCounterSpin(counterSpin);

            telemetry.addData("transfer level", robot.transferLevel);
            telemetry.update();

            dashboardTelemetry.addData("match time", matchTime.seconds());
            dashboardTelemetry.update();
        }
    }
}
