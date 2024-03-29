package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Far", group="Pushbot")
public class AutoBlueFar extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        String teamColor = "blue";
        setTeamColor(teamColor);
        setDistance("far");
        super.runOpMode();
    }
}

