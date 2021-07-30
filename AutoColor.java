package org.firstinspires.ftc.teamcode.Old;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Arm Test", group="Linear Opmode")
@Disabled public class AutoColor extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor sensorColorRight;
    ColorSensor sensorColorLeft;
    ColorSensor sensorColorMiddle;
    DistanceSensor sensorDistanceL;
    DistanceSensor sensorDistanceR;
    Servo rightArm;
    Servo leftArm;

    final int WHITE_ALPHA = 150;

    boolean kicked = false;

    @Override
    public void runOpMode() {

        sensorColorRight = hardwareMap.get(ColorSensor.class, "rightColor");
        sensorColorLeft = hardwareMap.get(ColorSensor.class, "leftColor");
        sensorColorMiddle = hardwareMap.get(ColorSensor.class, "middleColor");
        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "leftColor");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "rightColor");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (!kicked){

                telemetry.addData("Alpha Left", sensorColorLeft.alpha());
                telemetry.addData("Alpha Middle", sensorColorMiddle.alpha());
                telemetry.addData("Alpha Right", sensorColorRight.alpha());
                telemetry.addData("Distance Left", sensorDistanceL.getDistance(DistanceUnit.CM));
                telemetry.addData("Distance Right", sensorDistanceR.getDistance(DistanceUnit.CM));

                Log.i("Alpha Left", "" + sensorColorLeft.alpha());
                Log.i("Alpha Middle", "" +sensorColorMiddle.alpha());
                Log.i("Alpha Right", "" +sensorColorRight.alpha());
                Log.i("Distance Left", "" +sensorDistanceL.getDistance(DistanceUnit.CM));
                Log.i("Distance Right", "" +sensorDistanceR.getDistance(DistanceUnit.CM));

                if (sensorColorLeft.alpha() < WHITE_ALPHA){
                    telemetry.addData("Action", "Right Up");
                    //rightArm.setPosition(0.2);
                    kicked = true;
                } else if (sensorColorRight.alpha() < WHITE_ALPHA){
                    telemetry.addData("Action", "Left Up");
                    //leftArm.setPosition(0.8);
                    kicked = true;
                } else if (sensorColorMiddle.alpha() < WHITE_ALPHA){
                    telemetry.addData("Action", "Go Forward");
                    //rightArm.setPosition(0.2);
                    //leftArm.setPosition(0.8);
                    kicked = true;
                } else{
                    telemetry.addData("Action", "Confused");
                    kicked = true;
                }

                telemetry.update();
            }

            }
    }

}
