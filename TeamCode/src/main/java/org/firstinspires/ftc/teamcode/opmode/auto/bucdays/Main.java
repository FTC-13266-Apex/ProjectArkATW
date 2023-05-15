package org.firstinspires.ftc.teamcode.opmode.auto.bucdays;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auto.VisionAuto;

@Autonomous
public class Main extends VisionAuto {
    @Override
    protected boolean usingVision() {
        return true;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void onStart() {
        double power =.3;
        forward(power);
        sleep(1700);
        forward(0);
        sleep(1000);

        switch (getPosition()){
            case MID:

                break;
            case LEFT:
                strafe(-power);
                sleep(2700);
                forward(0);
                break;
            case RIGHT:
                strafe(power);
                sleep(2700);
                forward(0);

        }
        forward(power);
        sleep(1000);
        forward(0);
    }

    @Override
    public void run() {

    }
    private void forward (double power){
        drive.setMotorPowers(power,power,power,power);
    }
    private void strafe(double power){
        drive.setMotorPowers(power,-power,power,-power);
    }
}
