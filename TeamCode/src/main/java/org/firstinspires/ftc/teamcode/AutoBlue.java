package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoBlue", group="Pushbot")
public class AutoBlue extends AutoOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        String teamColor = "blue";
        setTeamColor(teamColor);
        super.runOpMode();
    }
}

