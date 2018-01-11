package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoBlueRed Far", group="Pushbot")
public class AutoRedFar extends NewAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        String teamColor = "red";
        setTeamColor(teamColor);
        setDistance("far");
        super.runOpMode();
    }
}
