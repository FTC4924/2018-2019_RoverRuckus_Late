package org.firstinspires.ftc.teamcode.SkystoneCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.SkystoneCode.Constants.*;

@Autonomous(name = "MoveFoundationRed")
public class MoveFoundationRed extends AutoBase{

    @Override
    public ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(

                        new Command(CommandType.MOVE_ARM, ARM_DEFAULT_POSITION, 0.3),
                        new Command(CommandType.MOVE_LIFTER, 1.5, 0.5),
                        new Command(CommandType.MOVE_MIDDLE_WHEEL, 0, 1.0, 0.5),
                        new Command(CommandType.MOVE_STRAIGHT, 0, 2.8, 0.5),
                        new Command(CommandType.MOVE_ARM, ARM_FOUNDATION_POSITION, 0.3),
                        new Command(CommandType.MOVE_STRAIGHT, 0, 3.5, -0.5),
                        new Command(CommandType.MOVE_ARM, ARM_DEFAULT_POSITION, 0.3),
                        new Command(CommandType.MOVE_MIDDLE_WHEEL, -5.0, 1.0, -0.5),
                        new Command(CommandType.MOVE_MIDDLE_WHEEL_COLOR, -5.0,-0.5),
                        new Command(CommandType.MOVE_STRAIGHT, 0, .5, -0.25)
                )
        );
    }
}
