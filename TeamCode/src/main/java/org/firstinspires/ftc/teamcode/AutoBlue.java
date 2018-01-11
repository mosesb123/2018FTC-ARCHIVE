package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoBlue Close", group="Pushbot")
public class AutoBlue extends NewAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        String teamColor = "blue";
        setTeamColor(teamColor);
        setDistance("close");
        super.runOpMode();
    }
}

