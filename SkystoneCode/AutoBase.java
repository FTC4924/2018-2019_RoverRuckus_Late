package org.firstinspires.ftc.teamcode.SkystoneCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.ArrayList;

public abstract class AutoBase extends OpMode {

    BNO055IMU imu;
    Orientation angles;

    ArrayList<Command> commands;
    Command currentCommand;
    int commandIndex;

    final double ANGLE_TOLERANCE = 1.00;
    final int POSITION_TOLERANCE = 5;

    DcMotor leftMotor;
    DcMotor rightMotor;

    DcMotor linearSlideMotor;
    DcMotor linkageMotor;

    DcMotor middleWheelMotor;

    int angleDirection = 0;

    ColorSensor color_sensor;
    int initialLight;

    public enum CommandType {
        TURN,
        MOVE,
        MOVE_STRAIGHT,
        MOVE_STRAIGHT_COLOR,
        MOVE_ARM,
        MOVE_LIFTER,
        MOVE_MIDDLE_WHEEL,
        MOVE_MIDDLE_WHEEL_COLOR
    }

    @Override
    public void init() {
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        linkageMotor = hardwareMap.get(DcMotor.class, "linkageMotor");
        linkageMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linkageMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        middleWheelMotor = hardwareMap.get(DcMotor.class, "middleWheelMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        commands = getCommands();
        currentCommand = commands.get(0);
        resetStartTime();
        commandIndex = -1;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        color_sensor = hardwareMap.colorSensor.get("colorSensor");

        imu.initialize(parameters);
        initialLight = color_sensor.alpha();
    }

    @Override
    public void loop() {

        int position = linkageMotor.getCurrentPosition();
        telemetry.addData("encoder mode", linkageMotor.getMode());
        telemetry.addData("Encoder Position", position);
        telemetry.addData("Encoder Target Position", linkageMotor.getTargetPosition());
        telemetry.addData("Color sensor: alpha", color_sensor.alpha()); // Total luminosity)

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("angle", angles);

        switch (currentCommand.commandType) {

            case MOVE:

                if (time > currentCommand.distance) {
                    startNextCommand();
                } else {
                    rightMotor.setPower(currentCommand.rightPower);
                    leftMotor.setPower(currentCommand.leftPower);
                }

                break;

            case MOVE_STRAIGHT:

                if (time > currentCommand.time) {

                    startNextCommand();

                } else {

                    rightMotor.setPower(currentCommand.power - (angles.firstAngle - currentCommand.angleTarget) * 0.05);
                    leftMotor.setPower(currentCommand.power + (angles.firstAngle - currentCommand.angleTarget) * 0.05);

                }

                break;

            case TURN:

                if (angles.firstAngle > currentCommand.angleTarget - 5.0 && angles.firstAngle < currentCommand.angleTarget + 5.0) {
                    startNextCommand();
                } else {

                    rightMotor.setPower(currentCommand.power * -1 * angleDirection);
                    leftMotor.setPower(currentCommand.power * angleDirection);

                    if (currentCommand.angleTarget > angles.firstAngle){
                        if (currentCommand.angleTarget - angles.firstAngle > 180) {
                            angleDirection = 1;
                        } else {
                            angleDirection = -1;
                        }
                    } else {
                        if (angles.firstAngle - currentCommand.angleTarget > 180) {
                            angleDirection = -1;
                        } else {
                            angleDirection = 1;
                        }
                    }
                }

                break;

            case MOVE_ARM:

                if (position >= currentCommand.position - POSITION_TOLERANCE && position <= currentCommand.position + POSITION_TOLERANCE) {
                    startNextCommand();
                } else {
                    linkageMotor.setTargetPosition(currentCommand.position);
                    linkageMotor.setPower(currentCommand.power);
                }

                break;

            case MOVE_LIFTER:

                if (time > currentCommand.distance) {
                    startNextCommand();
                } else {
                    linearSlideMotor.setPower(currentCommand.power);
                }

                break;

            case MOVE_MIDDLE_WHEEL:

                if(time > currentCommand.time) {
                    startNextCommand();
                } else {
                    middleWheelMotor.setPower(currentCommand.power);
                    rightMotor.setPower(-1 * (angles.firstAngle - currentCommand.angleTarget) * 0.05);
                    leftMotor.setPower((angles.firstAngle - currentCommand.angleTarget) * 0.05);
                }

                break;

            case MOVE_MIDDLE_WHEEL_COLOR:

                if (color_sensor.alpha() > initialLight + 100) {
                    startNextCommand();
                } else {
                    middleWheelMotor.setPower(currentCommand.power);
                    rightMotor.setPower(-1 * (angles.firstAngle - currentCommand.angleTarget) * 0.05);
                    leftMotor.setPower((angles.firstAngle - currentCommand.angleTarget) * 0.05);
                }

                break;

            case MOVE_STRAIGHT_COLOR:

                if (color_sensor.alpha() > initialLight + 100) {
                    startNextCommand();
                } else {
                    rightMotor.setPower(currentCommand.power - (angles.firstAngle - currentCommand.angleTarget) * 0.05);
                    leftMotor.setPower(currentCommand.power + (angles.firstAngle - currentCommand.angleTarget) * 0.05);
                }
        }
    }

    public void stopMotors() {

            linkageMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            leftMotor.setPower(0.0);
            linearSlideMotor.setPower(0.0);
            middleWheelMotor.setPower(0.0);

    }

    public void startNextCommand() {
        stopMotors();
        commandIndex ++;
        if (commandIndex < commands.size()) {
            currentCommand = commands.get(commandIndex);
            resetStartTime();
        }
    }

    public abstract ArrayList<Command> getCommands();
}