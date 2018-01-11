package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoBlueRed", group="Pushbot")
public class AutoRed extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        String teamColor = "red";
        setTeamColor(teamColor);
        super.runOpMode();
    }
}
