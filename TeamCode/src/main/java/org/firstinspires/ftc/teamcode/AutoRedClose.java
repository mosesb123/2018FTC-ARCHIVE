package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Close", group="Pushbot")
public class AutoRedClose extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        String teamColor = "red";
        setTeamColor(teamColor);
        setDistance("close");
        super.runOpMode();
    }
}
