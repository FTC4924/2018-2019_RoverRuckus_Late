package org.firstinspires.ftc.teamcode.SkystoneCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.SkystoneCode.Constants.*;

@Autonomous(name = "RaiseArm")
public class RaiseArm extends AutoBase {

    @Override
    public ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(

                        new Command(CommandType.MOVE_ARM, ARM_DEFAULT_POSITION, 0.3)

                )
        );
    }
}