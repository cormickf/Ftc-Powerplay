package org.firstinspires.ftc.teamcode;


// ToDo fix bupers and turning they need to be reversed



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum Teleop")
public class MecanumTeleOp extends LinearOpMode {

    private FtcGamePad driverGamePad;
    private FtcGamePad operatorGamePad;
    public BNO055IMU imu;
    public boolean fastMode;
    private boolean slowMode;



    @Override
    public void runOpMode() throws InterruptedException {

        driverGamePad = new FtcGamePad("Driver GamePad", gamepad1, this::OnDriverGamePadChange);
        operatorGamePad = new FtcGamePad("Operator GamePad", gamepad2, this::OnOperatorGamePadChange);

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                     = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit                = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit                = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled           = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);





        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // TODO: Only calculate x and v with math and calculate r from imu so 180 button can be implemented
            double y = Range.clip(Math.pow(-gamepad1.left_stick_y, 3), -1.0, 1.0);
            double xt = Math.pow(gamepad1.right_trigger, 3) - Math.pow(gamepad1.left_trigger, 3);
            double x = -Range.clip(Math.pow(gamepad1.left_stick_x, 3) + xt, -1.0, 1.0) * 1.1;
            double rx = Range.clip(-Math.pow(gamepad1.right_stick_x, 3), -1.0, 1.0);

            // Read inverse IMU heading, as the UMG heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle + 0;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            motorFrontLeft.setPower(fastMode ? frontLeftPower : frontLeftPower / 2);
            motorFrontRight.setPower(fastMode ?  frontRightPower : frontRightPower / 2);
            motorBackLeft.setPower(fastMode ? backLeftPower : backLeftPower / 2);
            motorBackRight.setPower(fastMode ? backRightPower : backRightPower / 2);


            driverGamePad.update();
            operatorGamePad.update();

            telemetry.update();
        }
    }

    private void OnDriverGamePadChange(FtcGamePad ftcGamePad, int button, boolean pressed) {

        switch (button) {
            case FtcGamePad.GAMEPAD_A:

                if(pressed)
                    fastMode = !fastMode;

                break;
            case FtcGamePad.GAMEPAD_B:

                if(pressed)
                    slowMode = !slowMode;

                break;



        }
    }

    private void OnOperatorGamePadChange(FtcGamePad ftcGamePad, int button, boolean pressed) {



    }


}