package org.firstinspires.ftc.teamcode.Old;

import android.text.method.Touch;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="AutonomousTest", group="Linear Opmode")
@Disabled public class AutonomousTest extends LinearOpMode {

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
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (!kicked){

                telemetry.addData("Alpha Left", sensorColorLeft.alpha());

                linearServo.scaleRange(0.0, 1.0);

                linearServo.setDirection(Servo.Direction.REVERSE);
                linearServo.setPosition(0.7);

                motor.setDirection(DcMotor.Direction.REVERSE);
                motor.setPower(0.7);




            }

        }
    }
}
