package org.firstinspires.ftc.teamcode.demobots;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name="Teleop", group="Teleop")
public class TestClas extends LinearOpMode
{
    //drive train motors declaration
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;

    public TestClas(HardwareMap hardwareMap)
    {
        //drive train motors initialization
        frontLeft = hardwareMap.get(DcMotorEx.class,"Front Left");
        backLeft = hardwareMap.get(DcMotorEx.class,"Back Left");
        frontRight = hardwareMap.get(DcMotorEx.class,"Front Right");
        backRight = hardwareMap.get(DcMotorEx.class,"Back Right");
    }

    //this method drives the robot, feel free to look at it, you shouldn't have to change it
    public void drive(double forward, double sideways, double rotation)
    {
        //adds all the inputs together to get the number to scale it by
        double scale = abs(rotation) + abs(forward) + abs(sideways);

        //scales the inputs when needed
        if (scale > 1)
        {
            forward /= scale;
            rotation /= scale;
            sideways /= scale;
        }

        //setting the motor powers to move
        frontLeft.setPower(forward - rotation - sideways);
        backLeft.setPower(forward - rotation + sideways);
        frontRight.setPower(forward + rotation + sideways);
        backRight.setPower(forward + rotation - sideways);
    }

    @Override
    //think of this as the main method, it starts when the code is initialized from the driver hub
    public void runOpMode() throws InterruptedException
    {
        //creates an object of this class called robot
        TestClas robot = new TestClas(hardwareMap);

        waitForStart();
        //this starts when the code is started and keeps going until the code is stopped
        while(opModeIsActive())
        {
            //drive method call
            robot.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        }
    }
}
