package org.firstinspires.ftc.teamcode.Old;

import android.text.method.Touch;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="motors", group="Linear Opmode")
@Disabled public class motors extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor sensorColorRight;
    ColorSensor sensorColorLeft;
    ColorSensor sensorColorMiddle;
    DistanceSensor sensorDistanceL;
    DistanceSensor sensorDistanceR;
    TouchSensor limitSwitch;
    Servo rightArm;
    Servo leftArm;
    Servo linearServo;
    DcMotor motor;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    final int WHITE_ALPHA = 150;

    boolean kicked = false;

    @Override
    public void runOpMode() {

        sensorColorRight = hardwareMap.get(ColorSensor.class, "rightColor");
        sensorColorLeft = hardwareMap.get(ColorSensor.class, "leftColor");
        sensorColorMiddle = hardwareMap.get(ColorSensor.class, "middleColor");
        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "leftColor");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "rightColor");
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        linearServo = hardwareMap.get(Servo.class, "linearServo");
        motor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (!kicked){

                telemetry.addData("Alpha Left", sensorColorLeft.alpha());
                
                motor.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);


            }

        }
    }
}
