/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Basic: Linear OpMode", group="Linear Opmode")

public class BasicOpMode_Linear extends LinearOpMode {
    DistanceSensor distance;
    DistanceSensor distance2;

    private static final double WHEEL_CIRCUMFERENCE = 3.5433 * Math.PI;
    public static  final double TICKS_PER_INCH = 288 / WHEEL_CIRCUMFERENCE;
    private static final int Turn_Time_90 = 1265;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;


    private void Waitmilli(int milli){
        ElapsedTime timer = new ElapsedTime();
        while(opModeIsActive()&& timer.milliseconds() < milli){


        }


    }

    private void DriveInches(double inches, double power) {
        final int ticksToDrive = (int) (inches * TICKS_PER_INCH);

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + ticksToDrive);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(power);



        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + ticksToDrive);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setPower(power);


        while(opModeIsActive() && leftDrive.isBusy() || rightDrive.isBusy());

        rightDrive.setPower(0);
        leftDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    @Override
    public void runOpMode() {


        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        distance = hardwareMap.get(DistanceSensor.class, "Color");
        distance2 = hardwareMap.get(DistanceSensor.class, "Color2");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        leftDrive.setPower(1);
        rightDrive.setPower(1);

        while(opModeIsActive()){
            if(distance.getDistance(DistanceUnit.MM) <= 100){
                rightDrive.setPower(0);
                leftDrive.setPower(0);
                Waitmilli(1000);
                DriveInches(-12,1);
                Waitmilli(1000);
                leftDrive.setPower(0);
                rightDrive.setPower(-1);
                Waitmilli(100);
                rightDrive.setPower(1);
                leftDrive.setPower(1);
            }
            if(distance2.getDistance(DistanceUnit.MM) <= 40){
                rightDrive.setPower(0);
                leftDrive.setPower(0);
                Waitmilli(1000);
                DriveInches(12,1);
                Waitmilli(1000);
                leftDrive.setPower(0);
                rightDrive.setPower(1);
                Waitmilli(100);
                rightDrive.setPower(1);
                leftDrive.setPower(1);

            }


        }



    }
}
