package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


@Autonomous(name = "ArnavTest Auto", group = "4924")

public class ArnavTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor extension;
    DcMotor rotation;
    Servo armBump;
    TouchSensor limitSwitch;

    boolean rotationOut = false;
    boolean done1 = false;

    static BNO055IMU imu;

    @Override
    public void runOpMode() {
        extension = hardwareMap.get(DcMotor.class, "extension");
        rotation = hardwareMap.get(DcMotor.class, "rotation");
        armBump = hardwareMap.get(Servo.class, "armBump");
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        armBump.setPosition(.45);

        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status:", "Starting");
        telemetry.update();

        while (opModeIsActive()) {
            if(!done1) {
                armBump.setPosition(0);
                rotation.setTargetPosition(1322);
                rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotation.setPower(1);
                if (!rotationOut) {
                    if (rotation.getCurrentPosition() > 300) {
                        rotation.setPower(0);
                        extension.setPower(-1);
                        sleep(1400);
                        extension.setPower(0);
                        rotationOut = true;

                    }
                }

                if (rotationOut) {
                    rotation.setPower(1);
                    if (rotation.getCurrentPosition() > 1300) {
                        rotation.setPower(0);
                        extension.setPower(-1);
                        if (limitSwitch.isPressed()) {
                            extension.setPower(0);
                            done1 = true;
                        }
                    }
                }
            }
        }
    }
}