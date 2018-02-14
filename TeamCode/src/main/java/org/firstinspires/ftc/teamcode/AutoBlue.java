package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Close", group="Pushbot")
public class AutoBlue extends NewAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        String teamColor = "blue";
        setTeamColor(teamColor);
        setDistance("close");
        super.runOpMode();
    }
}

