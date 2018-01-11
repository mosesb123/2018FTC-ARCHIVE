package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoBlue Far", group="Pushbot")
public class AutoBlueFar extends NewAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        String teamColor = "blue";
        setTeamColor(teamColor);
        setDistance("far");
        super.runOpMode();
    }
}

