package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Distance extends LinearOpMode {
    // Define a variable for our color sensor
    DistanceSensor distance;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        distance = hardwareMap.get(DistanceSensor.class, "Color");
        distance.getDistance(DistanceUnit.MM);



        waitForStart();   

        // wait untill the distance is less then 100 then stop

        while (opModeIsActive()) {

            telemetry.addData("Distance", distance.getDistance(DistanceUnit.MM));

            telemetry.update();


            }
        }
    }
