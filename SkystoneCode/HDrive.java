package org.firstinspires.ftc.teamcode.SkystoneCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.SkystoneCode.Constants.*;

@TeleOp(name="HDrive")
public class HDrive extends OpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor middleWheelMotor;

    DcMotor linearSlideMotor;
    DcMotor linkageMotor;

    Servo gripperServo;
    Servo capstoneServo;
    Servo gripRotationServo;

    @Override
    public void init() {

        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        middleWheelMotor = hardwareMap.get(DcMotor.class, "middleWheelMotor");

        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        linkageMotor = hardwareMap.get(DcMotor.class, "linkageMotor");
        linkageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        capstoneServo = hardwareMap.get(Servo.class, "capstoneServo");
        gripRotationServo = hardwareMap.get(Servo.class, "gripRotationServo");
        linkageMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linkageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gripperServo.setPosition(0.50);
        capstoneServo.setPosition(1.00);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        int position = linkageMotor.getCurrentPosition();
        telemetry.addData("Encoder Position", position);

        // prints the values of the left and right stick for driver one to the phone in order to help with bug fixing between the controllers and the phone.

        telemetry.addData("leftstick", gamepad1.left_stick_y);
        telemetry.addData("rightstick", gamepad1.right_stick_y);

        //Drive Train Controls
        //When you push the stick up, the robot moves forward on the specified side.
        //When you push the stick down, the robot moves backward on the specified side.

        if (gamepad1.right_stick_y > 0.01 || gamepad1.right_stick_y < -0.01) {

            rightMotor.setPower(gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y));

        } else {

            rightMotor.setPower(0.0);

        }

        if (gamepad1.left_stick_y > 0.01 || gamepad1.left_stick_y < -0.01) {

            leftMotor.setPower(gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));

        } else {

            leftMotor.setPower(0.0);

        }

        //If the left bumper is pressed the robot will go left.
        //If right bumper is pressed the robot will go right.

        if (gamepad1.left_bumper) {

            middleWheelMotor.setPower(-1 * MIDDLE_WHEEL_POWER);

        } else if (gamepad1.right_bumper) {

            middleWheelMotor.setPower(MIDDLE_WHEEL_POWER);

        } else {

            middleWheelMotor.setPower(0.0);

        }

        // Arm controls
        // If the right stick is pushed up, the arm goes up.
        // If the right stick is pushed down, the arm goes down.

        if (gamepad2.right_stick_y > -0.01 && linkageMotor.getCurrentPosition() > 0 || gamepad2.right_stick_y < 0.01 && linkageMotor.getCurrentPosition() < 1130) {

            linkageMotor.setPower(gamepad2.right_stick_y * Math.abs(gamepad2.right_stick_y / 2) * -1);

        } else if (gamepad2.right_stick_y < 0.01) {

            linkageMotor.setPower(0.00);

        }

        //Linear Slide Controls
        //If the left stick is pushed up, the linear slide goes up.
        //If the left stick is pushed down, the linear slide goes down.

        if (gamepad2.left_stick_y > 0.01 || gamepad2.left_stick_y < -0.01) {

            linearSlideMotor.setPower(gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y));

        } else {

            linearSlideMotor.setPower(0.00);

        }

        //Gripper Controls
        //X is all the way in. b is all the way out. y is half way inbetween.

        if (gamepad2.x) {

            gripperServo.setPosition(GRIPPER_MINIMUM_POSITION);

        }

        if (gamepad2.b) {

            gripperServo.setPosition(GRIPPER_MAXIMUM_POSITION);

        } if (gamepad2.y)  {

            gripperServo.setPosition(GRIPPER_MIDDLE_POSITION);

        }

        //Capstone Servo Controls
        //If the right bumper is pressed, the capstone servo goes down.
        //If the left bumper is pressed, the capstone servo goes up.

        if (gamepad2.left_bumper) {

            capstoneServo.setPosition(CAPSTONE_MAXIMUM_POSITION);

        }

        if (gamepad2.right_bumper) {

            capstoneServo.setPosition(CAPSTONE_MINIMUM_POSITION);

        }

        //Gripper Rotation Servo Controls
        //Continuous rotation servo
        //If the top of the dpad is pressed, the gripRotationServo spins counterclockwise.
        //If the bottom of the dpad is pressed, the gripRotationServo spins clockwise.

        if (gamepad2.dpad_up) {

            gripRotationServo.setPosition(0.0);//Turn Clockwise

        } else if (gamepad2.dpad_down) {

            gripRotationServo.setPosition(1.0);//Turn counterclockwise

        } else {

            gripRotationServo.setPosition(0.50);//Stop

        }
    }
}
