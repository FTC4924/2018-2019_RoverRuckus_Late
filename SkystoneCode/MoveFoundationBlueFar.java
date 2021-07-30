package org.firstinspires.ftc.teamcode.SkystoneCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "MoveFoundationBlueFar")
public class MoveFoundationBlueFar extends AutoBase {

    @Override
    public ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(

                        new Command(CommandType.MOVE_ARM, 1.0, 0.25),
                        new Command(CommandType.MOVE_LIFTER, 1.75, 0.25),
                        new Command(CommandType.MOVE, 0.5, 0.5),
                        new Command(CommandType.TURN, 0.5, -0.5),
                        new Command(CommandType.MOVE, 1.0, 0.5),
                        new Command(CommandType.TURN, 0.5, 0.5),
                        new Command(CommandType.MOVE, 1.25, 0.5),
                        new Command(CommandType.MOVE, 1.0, 0.0),
                        new Command(CommandType.MOVE, 3.5, -0.5),
                        new Command(CommandType.MOVE_ARM, 0.5, 0.25),
                        new Command(CommandType.MOVE_MIDDLE_WHEEL, 0.5, 1.0),
                        new Command(CommandType.MOVE, 0.5, 0.25),
                        new Command(CommandType.MOVE_MIDDLE_WHEEL, 0.25, 0.25)
                )
        );
    }
}
