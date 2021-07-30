package org.firstinspires.ftc.teamcode.SkystoneCode;

import static org.firstinspires.ftc.teamcode.SkystoneCode.AutoBase.CommandType;

public class Command {

    public double power;
    public double leftPower;
    public double rightPower;
    public double angleTarget;
    public double distance;
    public double time;
    public int position;
    public CommandType commandType;

    public Command(CommandType moveType, double target, double leftPower, double rightPower) {

        commandType = moveType;

        if (moveType == CommandType.TURN || moveType == CommandType.MOVE || moveType == CommandType.MOVE_MIDDLE_WHEEL_COLOR || moveType == CommandType.MOVE_STRAIGHT_COLOR) {

            angleTarget = target;

        } else if (moveType == CommandType.MOVE_ARM) {

            position = (int)target;

        } else if (moveType == CommandType.MOVE) {

            this.leftPower = leftPower;
            this.rightPower = rightPower;

        } else if (moveType == CommandType.MOVE_STRAIGHT || moveType == CommandType.MOVE_MIDDLE_WHEEL) {

            this.angleTarget = target;
            this.time = leftPower;
            this.power = rightPower;

        } else {

            distance = target;

        }

    }

    public Command(CommandType moveType, double power) {

        commandType = moveType;
        this.power = power;

    }

    public Command(CommandType moveType, double target, double power) {

        this(moveType, target, power, power);

        this.power = power; //For using the robot attachments

    }

}