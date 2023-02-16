package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ConeFlipper extends Subsystem {
    public static class Constants {
        public static Hardware hardware;
        public static Position position;
        private static class Hardware{

            public static Servo.Direction BOTTOM_DIRECTION = Servo.Direction.REVERSE;
            public static Servo.Direction TOP_DIRECTION = Servo.Direction.REVERSE;

        }
        private static class Position {
            public static volatile double
                    LIFT = .7,
                    DROP = .36;
            public static volatile double
                    HIDDEN = 1,
                    GET_CONE = 0.65,
                    FEED_CONE = .3,
                    SIGNAL_CONE_PUSHER = .4,
                    SIGNAL_CONE_YEET = .8;
        }
    }

    private final Servo coneFlipperBottom;
    private final Servo coneFlipperTop;

    public ConeFlipper(@NonNull OpMode opMode) {
        super(opMode);
        coneFlipperBottom = opMode.hardwareMap.get(Servo.class, "coneFlipperBottom");
        coneFlipperTop = opMode.hardwareMap.get(Servo.class, "coneFlipperTop");

        coneFlipperTop.setDirection(Constants.Hardware.TOP_DIRECTION);
        coneFlipperBottom.setDirection(Constants.Hardware.BOTTOM_DIRECTION);

        hide();
        lift();
    }

    @Override
    protected void manualControl() {

    }

    public void hide() {
        coneFlipperTop.setPosition(Constants.Position.HIDDEN);
    }

    public void lift() {
        coneFlipperBottom.setPosition(Constants.Position.LIFT);
    }

    public void getCone() {
        coneFlipperTop.setPosition(Constants.Position.GET_CONE);
    }

    public void drop() {
        coneFlipperBottom.setPosition(Constants.Position.DROP);
    }

    public void feedCone() {
        coneFlipperTop.setPosition(Constants.Position.FEED_CONE);
    }

    public void SignalConePusher(){coneFlipperTop.setPosition(Constants.Position.SIGNAL_CONE_PUSHER);}

    public void SignalConeYeet(){coneFlipperTop.setPosition(Constants.Position.SIGNAL_CONE_YEET);}
}
