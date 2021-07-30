package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


@Autonomous(name = "linearMotor down", group = "4924")

    public class linearMotordown extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor linearMotor;

    static BNO055IMU imu;

    @Override
    public void runOpMode() {
        linearMotor = hardwareMap.get(DcMotor.class, "linearMotor");

        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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

            linearMotor.setTargetPosition(7122);
            linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearMotor.setPower(0.5);
            if (linearMotor.getCurrentPosition() > 7100) {
                linearMotor.setPower(0);
            }

        }
    }
}