package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.FlipCone;
import org.firstinspires.ftc.teamcode.subsystem.ConeFlipper;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
@TeleOp
public class TeleOpMain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(this);
        Gripper gripper = new Gripper(this);
        ConeFlipper coneFlipper = new ConeFlipper(this);
        Drive drive = new Drive(this,true);

        FlipCone flipCone = new FlipCone(coneFlipper, lift, gripper);

        gripper.open();

        waitForStart();
        while (opModeIsActive()) {
            lift.controlWithGamepad();
            gripper.controlWithGamepad();

            flipCone.runCommandManual(gamepad2);

            drive.manualControl();
            telemetry.update();
        }

    }
}
