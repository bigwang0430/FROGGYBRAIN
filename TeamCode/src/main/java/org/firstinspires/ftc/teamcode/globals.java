package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class globals {
    @Config
    public static class launcher {
        public static float targetRPM = 3000F;
        public static boolean SquidOn = false;
        public static boolean launcherOn = false;
        public static float p =0.01F; //0.001
        public static float i = 0.1F;
        public static float d = 0F;
        public static float ks = 0.154F; //0.0000216
        public static float kv = 0.000195F; //0.000000120871
        public static float squP = 0F;
        public static float squI = 0F;
        public static float squD = 0F;
        public static float squKs = 0F;
        public static float squKv = 0F;
        public static float hoodAng = 40F;
    }

    @Config
    public static class kalman {

        // Starting uncertainty (variance) in odometry estimate, in inches^2
        public static double pX0 = 0.001;
        public static double pY0 = 0.001;

        // How much uncertainty (variance) you add each update loop, in inches^2 per loop
        public static double qX = 0.00001;
        public static double qY = 0.00001;

        // Camera measurement noise (variance), in inches^2
        public static double rX = 1_000_000_000.0;
        public static double rY = 1_000_000_000.0;
    }

    @Config
    public static class offsets{
        public static double xoff = -129;
        public static double yoff = -128;
    }
}
