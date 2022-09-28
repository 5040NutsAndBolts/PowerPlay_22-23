package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//camera import
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.signalfinding.GreenFinder;
import org.firstinspires.ftc.teamcode.signalfinding.OrangeFinder;
import org.firstinspires.ftc.teamcode.signalfinding.PurpleFinder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (name = "Signal Auto", group = "Autonomous")
public class SignalAuto extends LinearOpMode
{
    int autoNumber = 1;

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

        //initializes robot
        Hardware robot = new Hardware(hardwareMap);

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

            telemetry.addData("auto", autoNumber);
            telemetry.update();
        }

        waitForStart();

        //runs once program is started
        while(opModeIsActive())
        {
            if(autoNumber == 1)
            {
                ElapsedTime timer = new ElapsedTime();
                timer.startTime();

                while(timer.seconds() < .25 && opModeIsActive())
                {
                    robot.drive(0,-.5,0);
                    telemetry.addLine("moving sideways");
                    telemetry.update();
                }

                timer.reset();
                timer.startTime();

                while (timer.seconds() < 1 && opModeIsActive())
                {
                    robot.drive(-.5,0,0);
                    telemetry.addLine("moving forward");
                    telemetry.update();
                }

            }

            else if(autoNumber == 2)
            {
                ElapsedTime timer = new ElapsedTime();
                timer.startTime();

                while (timer.seconds() < 1.5 && opModeIsActive())
                    robot.drive(-.5,0,0);
            }

            else
            {
                ElapsedTime timer = new ElapsedTime();
                timer.startTime();

                while(timer.seconds() < 1 && opModeIsActive())
                    robot.drive(-.5,.5,0);

                while (timer.seconds() < 1.5 && opModeIsActive())
                    robot.drive(-.5,0,0);
            }

            robot.drive(0,0,0);
            telemetry.addLine("stopped");
            telemetry.update();
        }
    }
}
