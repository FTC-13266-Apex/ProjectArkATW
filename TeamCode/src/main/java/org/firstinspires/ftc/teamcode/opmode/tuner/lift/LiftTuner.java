package org.firstinspires.ftc.teamcode.opmode.tuner.lift;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.FlipConeWithGamepad;
import org.firstinspires.ftc.teamcode.command.OptimizeStuff;
import org.firstinspires.ftc.teamcode.command.Sensor;
import org.firstinspires.ftc.teamcode.command.Sussy;
import org.firstinspires.ftc.teamcode.subsystem.ConeFlipper;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp(group="a tuner")
public class LiftTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(this);
        new OptimizeStuff(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            lift.runIteratively();
//            lift.updateDashboard();
            telemetry.update();
        }

    }
}
