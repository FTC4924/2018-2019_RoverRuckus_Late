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

@Autonomous(name="ExtensionReset", group="Linear Opmode")
public class ExtensionReset extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    TouchSensor limitSwitch;
    TouchSensor limitSwitch2;
    DcMotor extension;

    final int WHITE_ALPHA = 150;

    boolean kicked = false;

    @Override
    public void runOpMode() {

        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        limitSwitch2 = hardwareMap.get(TouchSensor.class, "limitSwitch2");
        extension = hardwareMap.get(DcMotor.class, "extension");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (!kicked){

                extension.setPower(1);

                if (limitSwitch2.isPressed()) {
                    kicked = true;
                    extension.setPower(0);
                }

            }

        }
    }
}
