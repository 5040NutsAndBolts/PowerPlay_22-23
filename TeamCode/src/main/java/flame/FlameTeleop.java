import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Flame", group = "Teleop")
public class Flame extends OpMode
{
    public boolean rDrive = true;
    public boolean slowMode = false;
    public double driveSpeed = 1;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public boolean b1Pressed = false;

    public BNO055IMU imu;
    public double adjust;

    @Override
    public void loop()
    {
        //robot oriented drive method call
        if (rDrive)
            robotODrive(gamepad1.left_stick_y * driveSpeed, gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed);

        //field oriented drive method call
        if (!rDrive)
            fieldODrive(gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed, gamepad1.right_stick_button);

        //drive mode toggles
        if (gamepad1.right_bumper)
            rDrive = true;
        if (gamepad1.left_bumper)
            rDrive = false;

        //slowmode setup
        if (gamepad1.b && !b1Pressed)
        {
            slowMode = !slowMode;
            b1Pressed = true;
        } else if (!gamepad1.b)
            b1Pressed = false;

        if (slowMode)
            driveSpeed = .5;
        else
            driveSpeed = 1;
    }
}