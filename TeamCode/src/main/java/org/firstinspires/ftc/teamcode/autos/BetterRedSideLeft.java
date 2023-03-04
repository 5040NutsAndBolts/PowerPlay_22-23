package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "Better Red Left Auto", group = "Autonomous")
public class BetterRedSideLeft extends LinearOpMode
{
    int autoNumber = 1;
    boolean rbPressed;

    boolean counterSpin, podFailure, nearLine, reset;
    public double correctedX;

    Trajectory preloadGoal0, park1_1, park2_1, park3_1, backAway1, lineUp2, stackIntake3, poleLineUp4,
            stackScore5, backAway6, lineUp7, stackIntake8, poleLineUp9, stackScore10, park1_X, park2_X, park3_X;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    enum config
    {
        preload,
        oneCycle,
        twoCycle
    }

    config toRun = config.twoCycle;

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

        Pose2d startPose = new Pose2d(-64, 30);
        robot.setPoseEstimate(startPose);

        preloadGoal0 = robot.trajectoryBuilder(startPose)
                .addDisplacementMarker(() -> {
                    counterSpin = true;
                })
                .splineTo(new Vector2d(-50, 14), 0,
                        RoadRunner.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 3;
                })
                .splineTo(new Vector2d(-26, 4), Math.toRadians(310),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    counterSpin = false;
                    robot.depositCone();
                })
                .addDisplacementMarker(() -> {
                    if(toRun == config.preload)
                        robot.followTrajectoryAsync(park3_1);
                    else
                        robot.followTrajectoryAsync(backAway1);
                })
                .build();

        park3_1 = robot.trajectoryBuilder(preloadGoal0.end())
                .strafeTo(new Vector2d(-36, 13),
                        RoadRunner.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                    if(autoNumber == 2)
                        robot.followTrajectoryAsync(park2_1);
                    else if(autoNumber == 1)
                        robot.followTrajectoryAsync(park1_1);
                })
                .build();

        park2_1 = robot.trajectoryBuilder(park3_1.end())
                .strafeTo(new Vector2d(-36, 35))
                .build();

        park1_1 = robot.trajectoryBuilder(park3_1.end())
                .strafeTo(new Vector2d(-36, 60))
                .build();


        backAway1 = robot.trajectoryBuilder(preloadGoal0.end())
                .strafeTo(new Vector2d(-35, 12),
                        RoadRunner.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                    robot.followTrajectoryAsync(lineUp2);
                })
                .build();

        lineUp2 = robot.trajectoryBuilder(backAway1.end())
                .splineTo(new Vector2d(-30, 11), 0,
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 1;
                })
                .splineToSplineHeading(new Pose2d(-11, 40, Math.toRadians(90)), Math.toRadians(90),
                        RoadRunner.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-10.75, 60, Math.toRadians(90)), Math.toRadians(90),
                        RoadRunner.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 1;
                    counterSpin = false;
                    robot.followTrajectoryAsync(stackIntake3);
                })
                .build();

        stackIntake3 = robot.trajectoryBuilder(lineUp2.end())
                .lineTo(new Vector2d(-10.75, 62.5),
                        RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.intakeCone();
                    counterSpin = true;
                    robot.transferLevel = 1;
                    robot.followTrajectoryAsync(poleLineUp4);
                })
                .build();

        poleLineUp4 = robot.trajectoryBuilder(stackIntake3.end(), Math.toRadians(90))
                .lineTo(new Vector2d(-10.75, 55),
                        RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-12, 8), Math.toRadians(270),
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
                .splineTo(new Vector2d(-2.5, 19), Math.toRadians(45),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    counterSpin = false;
                    robot.depositCone();
                })
                .addDisplacementMarker(() -> {
                    if(toRun == config.oneCycle)
                        robot.followTrajectoryAsync(park3_X);
                    else
                        robot.followTrajectoryAsync(backAway6);
                })
                .build();

        backAway6 = robot.trajectoryBuilder(stackScore5.end())
                .strafeTo(new Vector2d(-13, 8),
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
                .splineToConstantHeading((new Vector2d(-14, 10)), Math.toRadians(90),
                        RoadRunner.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 1;
                })
                .splineToSplineHeading(new Pose2d(-12, 40, Math.toRadians(90)), Math.toRadians(90),
                        RoadRunner.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-11, 60, Math.toRadians(90)), Math.toRadians(90),
                        RoadRunner.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                    robot.followTrajectoryAsync(stackIntake8);
                })
                .build();

        stackIntake8 = robot.trajectoryBuilder(lineUp7.end())
                .lineTo(new Vector2d(-11, 62.7),
                        RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.intakeCone();
                    counterSpin = true;
                    robot.transferLevel = 1;
                    robot.followTrajectoryAsync(poleLineUp9);
                })
                .build();

        poleLineUp9 = robot.trajectoryBuilder(stackIntake8.end(), Math.toRadians(90))
                .strafeTo(new Vector2d(-11, 55),
                        RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-12, 8), Math.toRadians(270),
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
                .splineTo(new Vector2d(-1.5, 19.5), Math.toRadians(45),
                        RoadRunner.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    counterSpin = false;
                    robot.depositCone();
                })
                .addDisplacementMarker(() -> {
                    robot.followTrajectoryAsync(park3_X);
                })
                .build();

        park3_X = robot.trajectoryBuilder(stackScore10.end())
                .strafeTo(new Vector2d(-13, 10),
                        RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.transferLevel = 0;
                    if(autoNumber == 2)
                        robot.followTrajectoryAsync(park2_X);
                    else if(autoNumber == 1)
                        robot.followTrajectoryAsync(park1_X);
                })
                .build();

        park2_X = robot.trajectoryBuilder(park3_X.end())
                .strafeTo(new Vector2d(-12, 35))
                .build();

        park1_X = robot.trajectoryBuilder(park3_X.end())
                .strafeTo(new Vector2d(-14, 57))
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

            if(gamepad1.right_bumper && !rbPressed)
            {
                if(toRun == config.twoCycle)
                    toRun = config.preload;
                else if(toRun == config.preload)
                    toRun = config.oneCycle;
                else
                    toRun = config.twoCycle;

                rbPressed = true;
            }
            else if(!gamepad1.right_bumper)
                rbPressed = false;

            dashboardTelemetry.addData("auto", autoNumber);
            dashboardTelemetry.addData("running", toRun);
            dashboardTelemetry.update();

            telemetry.addData("auto", autoNumber);
            telemetry.addData("running", toRun);
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

            /*if(nearLine)
            {
                if(correctedX != 12)
                {
                    correctedX = robot.findRed(reset, robot.getPoseEstimate().getX());
                    reset = false;
                }
            }*/

            dashboardTelemetry.addData("Match time", matchTime);
            dashboardTelemetry.addData("counterspin?", counterSpin);
            dashboardTelemetry.addData("transfer level", robot.transferLevel);
            dashboardTelemetry.update();

            telemetry.addData("transfer level", robot.transferLevel);
            telemetry.update();
        }
    } //ending of runOpModeLoop
}