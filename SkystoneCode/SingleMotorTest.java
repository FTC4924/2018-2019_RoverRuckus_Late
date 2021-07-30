package org.firstinspires.ftc.teamcode.SkystoneCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="GyroTest")
public class SingleMotorTest extends OpMode {

    BNO055IMU gyroSensor;

    @Override
    public void init() {
        gyroSensor = hardwareMap.get(BNO055IMU.class, "gyroSensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        gyroSensor.initialize(parameters);
    }

    @Override
    public void loop() {
        Orientation heading = gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        telemetry.addData("x", heading.firstAngle);
        telemetry.addData("y", heading.secondAngle);
        telemetry.addData("z", heading.thirdAngle);
    }
}
