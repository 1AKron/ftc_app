package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwarePushbot;

@Autonomous(name="Main Autonomous")
public class MainAutonomous extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.encoderDrive(this, 5, 2, 2, 2, 2);
    }
}
