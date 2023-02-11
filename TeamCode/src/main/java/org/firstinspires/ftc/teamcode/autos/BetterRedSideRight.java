package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.drive.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.signalfinding.GreenFinder;
import org.firstinspires.ftc.teamcode.signalfinding.OrangeFinder;
import org.firstinspires.ftc.teamcode.signalfinding.PurpleFinder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Better Red Right Auto", group = "Autonomous")
public class BetterRedSideRight extends LinearOpMode
{
    int autoNumber = 1;

    boolean counterSpin, podFailure;

    Trajectory preloadGoal0, backAway1, lineUp2, stackIntake3, poleLineUp4, stackScore5, backAway6, lineUp7, stackIntake8, poleLineUp9, stackScore10, park1X, park2X, park3X;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode()
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

        ElapsedTime matchTime = new ElapsedTime();

        Pose2d startPose = new Pose2d(-64, -40);
        robot.setPoseEstimate(startPose);

        preloadGoal0 = robot.trajectoryBuilder(startPose)
                .addDisplacementMarker(() -> {
                    counterSpin = true;
                })
                .splineTo(new Vector2d(-50, -14), 0,
                        RoadRunner.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 3;
                })
                .splineTo(new Vector2d(-27, -4), Math.toRadians(50),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    counterSpin = false;
                    robot.depositCone();
                })
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(backAway1);
                })
                .build();

        backAway1 = robot.trajectoryBuilder(preloadGoal0.end())
                .strafeTo(new Vector2d(-36, -15),
                        RoadRunner.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                    robot.followTrajectoryAsync(lineUp2);
                })
                .build();

        lineUp2 = robot.trajectoryBuilder(backAway1.end())
                .splineTo(new Vector2d(-33, -13), 0,
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 1;
                })
                .splineToSplineHeading(new Pose2d(-11, -61, Math.toRadians(270)), Math.toRadians(270),
                        RoadRunner.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                    counterSpin = false;
                    robot.followTrajectoryAsync(stackIntake3);
                })
                .build();

        stackIntake3 = robot.trajectoryBuilder(lineUp2.end())
                .lineTo(new Vector2d(-11, -63))
                .addDisplacementMarker(() -> {
                    robot.intakeCone();
                    counterSpin = true;
                    robot.transferLevel = 1;
                    robot.followTrajectoryAsync(poleLineUp4);
                })
                .build();

        poleLineUp4 = robot.trajectoryBuilder(stackIntake3.end(), Math.toRadians(270))
                .lineTo(new Vector2d(-11, -55),
                        RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-12, -8), Math.toRadians(90),
                        RoadRunner.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(stackScore5);
                })
                .build();

        stackScore5 = robot.trajectoryBuilder(poleLineUp4.end())
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 3;
                })
                .splineTo(new Vector2d(-3, -19), Math.toRadians(330),
                        RoadRunner.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    counterSpin = false;
                    robot.depositCone();
                })
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(backAway6);
                })
                .build();

        backAway6 = robot.trajectoryBuilder(stackScore5.end())
                .strafeTo(new Vector2d(-13, -8),
                        RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                })
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(lineUp7);
                })
                .build();

        lineUp7 = robot.trajectoryBuilder(backAway6.end())
                .splineToConstantHeading((new Vector2d(-14, -10)), Math.toRadians(270),
                        RoadRunner.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 1;
                })
                .splineToSplineHeading(new Pose2d(-12, -40, Math.toRadians(270)), Math.toRadians(270),
                        RoadRunner.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-10, -61, Math.toRadians(270)), Math.toRadians(270),
                        RoadRunner.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                    robot.followTrajectoryAsync(stackIntake8);
                })
                .build();

        stackIntake8 = robot.trajectoryBuilder(lineUp7.end())
                .lineTo(new Vector2d(-10, -63))
                .addDisplacementMarker(() -> {
                    robot.intakeCone();
                    counterSpin = true;
                    robot.transferLevel = 1;
                    robot.followTrajectoryAsync(poleLineUp9);
                })
                .build();

        poleLineUp9 = robot.trajectoryBuilder(stackIntake8.end(), Math.toRadians(270))
                .strafeTo(new Vector2d(-10, -55),
                        RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-12, -8), Math.toRadians(90),
                        RoadRunner.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(stackScore10);
                })
                .build();

        stackScore10 = robot.trajectoryBuilder(poleLineUp9.end())
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 3;
                })
                .splineTo(new Vector2d(-3, -19), Math.toRadians(330),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    counterSpin = false;
                    robot.depositCone();
                })
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(park1X);
                })
                .build();

        park1X = robot.trajectoryBuilder(stackScore10.end())
                .strafeTo(new Vector2d(-13, -11),
                        RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                    if(autoNumber != 1)
                        robot.followTrajectoryAsync(park2X);
                })
                .build();

        park2X = robot.trajectoryBuilder(park1X.end())
                .strafeTo(new Vector2d(-11, -35))
                .addDisplacementMarker(() -> {
                    if(autoNumber == 3)
                        robot.followTrajectoryAsync(park3X);
                })
                .build();

        park3X = robot.trajectoryBuilder(park2X.end())
                .strafeTo(new Vector2d(-11, -60))
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
            robot.runCounterSpin(counterSpin);

            podFailure = robot.failSafe(matchTime);
            if(podFailure)
                requestOpModeStop();

            dashboardTelemetry.addData("Match time", matchTime);
            dashboardTelemetry.addData("counterspin?", counterSpin);
            dashboardTelemetry.addData("transfer level", robot.transferLevel);
            dashboardTelemetry.update();

            telemetry.addData("transfer level", robot.transferLevel);
            telemetry.update();
        }
    } //ending of runOpModeLoop
}