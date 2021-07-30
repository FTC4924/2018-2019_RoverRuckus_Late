package org.firstinspires.ftc.teamcode.SkystoneCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.ArrayList;
import java.util.Arrays;

@Disabled
@Autonomous(name = "AutoTestReverse")
public class AutoTestReverse extends AutoBase {

    @Override
    public ArrayList<Command> getCommands() {
        return new ArrayList<>(
            Arrays.asList(
                new Command(CommandType.MOVE_MIDDLE_WHEEL, 5.00, .01)
            )
        );
    }
}
