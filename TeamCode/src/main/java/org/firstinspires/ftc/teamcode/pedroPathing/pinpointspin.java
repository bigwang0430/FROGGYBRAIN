package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals;

@TeleOp(name = "Pinpoint Spin Offset Test (Degrees)", group = "Test")
public class pinpointspin extends LinearOpMode {

    // ================== TUNABLES ==================
    // Offsets: per goBILDA Pinpoint guide:
    // X offset = LEFT/RIGHT of the FORWARD-tracking (X) pod. Left +, Right -
    // Y offset = FORWARD/BACK of the STRAFE-tracking (Y) pod. Forward +, Back -
    private static final double X_POD_OFFSET_MM = globals.offsets.xoff;
    private static final double Y_POD_OFFSET_MM = globals.offsets.yoff;

    // Spin behavior
    private static final double SPIN_POWER = 0.30;      // 0.30–0.50 typical
    private static final double TEST_TIMEOUT_S = 20.0;  // give it time to reach 360
    private static final double TARGET_DEG = 360.0;     // one full revolution

    // Stop when close enough (degrees)
    private static final double STOP_TOL_DEG = 4.0;

    // ================== HW ==================
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        // ---- Drive motors ----
        leftFront  = hardwareMap.get(DcMotorEx.class, "fl");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        leftBack   = hardwareMap.get(DcMotorEx.class, "bl");
        rightBack  = hardwareMap.get(DcMotorEx.class, "br");

        // Typical mecanum directions (adjust if yours differ)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        setBrake(true);
        setDrivePower(0, 0, 0);

        // ---- Pinpoint ----
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set offsets in mm (make sure these are from TRACKING POINT to WHEEL CENTER)
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);

        // Encoder directions (if your push-tests disagree, swap FORWARD/REVERSE for that pod)
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,    // X pod (forward-tracking)
                GoBildaPinpointDriver.EncoderDirection.FORWARD     // Y pod (strafe-tracking)
        );

        // Set pod type (choose the one you actually use!)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        // Helpful: show status before start
        telemetry.addLine("Pinpoint Spin Offset Test (Degrees)");
        telemetry.addLine("A = resetPosAndIMU (robot still!)");
        telemetry.addLine("B = auto spin CW 360");
        telemetry.addLine("X = auto spin CCW 360");
        telemetry.addLine("LB/RB = manual spin");
        telemetry.update();

        waitForStart();

        // Start with a clean slate (robot must be motionless)
        pinpoint.resetPosAndIMU();

        boolean lastA = false, lastB = false, lastX = false;

        while (opModeIsActive()) {
            pinpoint.update();

            // Rising-edge button detection
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;

            if (a && !lastA) {
                // Robot MUST be stationary
                pinpoint.resetPosAndIMU();
            }
            if (b && !lastB) {
                runSpinTest(+1); // CW
            }
            if (x && !lastX) {
                runSpinTest(-1); // CCW
            }

            lastA = a;
            lastB = b;
            lastX = x;

            // Manual bumpers for quick testing
            double turn = 0.0;
            if (gamepad1.right_bumper) turn = +SPIN_POWER;
            if (gamepad1.left_bumper)  turn = -SPIN_POWER;

            setDrivePower(0, 0, turn);

            // Telemetry (live)
            double xIn = pinpoint.getPosX(DistanceUnit.INCH);
            double yIn = pinpoint.getPosY(DistanceUnit.INCH);
            double hDeg = pinpoint.getHeading(AngleUnit.DEGREES);

            telemetry.addLine("Pinpoint Spin Offset Test (Degrees)");
            telemetry.addLine("A: reset | B: CW 360 | X: CCW 360 | LB/RB: manual spin");
            telemetry.addData("Offsets (mm)", "x=%.1f  y=%.1f", X_POD_OFFSET_MM, Y_POD_OFFSET_MM);
            telemetry.addData("Pos (in)", "X=%.2f  Y=%.2f", xIn, yIn);
            telemetry.addData("Heading (deg)", "%.1f", hDeg);
            telemetry.update();
        }
    }

    /**
     * Automatic ~360° spin test using DEGREES with unwrap + accumulation.
     * dir = +1 CW, -1 CCW
     */
    private void runSpinTest(int dir) {
        // Snapshot start pose
        pinpoint.update();
        double startX = pinpoint.getPosX(DistanceUnit.INCH);
        double startY = pinpoint.getPosY(DistanceUnit.INCH);

        // Accumulate heading deltas with unwrap so we can exceed 180°
        double lastH = pinpoint.getHeading(AngleUnit.DEGREES);
        double accumulated = 0.0;

        ElapsedTime t = new ElapsedTime();

        while (opModeIsActive() && t.seconds() < TEST_TIMEOUT_S) {
            pinpoint.update();

            double h = pinpoint.getHeading(AngleUnit.DEGREES);
            double d = h - lastH;

            // unwrap: assume heading wraps at +/-180 (common)
            while (d > 180.0) d -= 360.0;
            while (d < -180.0) d += 360.0;

            accumulated += d;
            lastH = h;

            // Stop when we reach ~360 in the commanded direction
            if (Math.abs(accumulated) >= (TARGET_DEG - STOP_TOL_DEG)) {
                break;
            }

            setDrivePower(0, 0, SPIN_POWER * dir);

            telemetry.addLine("AUTO SPIN TEST RUNNING...");
            telemetry.addData("t (s)", "%.2f / %.1f", t.seconds(), TEST_TIMEOUT_S);
            telemetry.addData("Accum rot (deg)", "%.1f / %.1f", accumulated, TARGET_DEG);
            telemetry.addData("X,Y (in)", "%.2f, %.2f",
                    pinpoint.getPosX(DistanceUnit.INCH),
                    pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.update();
        }

        setDrivePower(0, 0, 0);
        sleep(200);

        // Final readings
        pinpoint.update();
        double endX = pinpoint.getPosX(DistanceUnit.INCH);
        double endY = pinpoint.getPosY(DistanceUnit.INCH);

        double driftX = endX - startX;
        double driftY = endY - startY;
        double driftMag = Math.hypot(driftX, driftY);

        telemetry.clearAll();
        telemetry.addLine("AUTO SPIN TEST DONE");
        telemetry.addData("Rotation achieved (deg)", "%.1f", Math.abs(accumulated));
        telemetry.addData("Start (in)", "X=%.2f  Y=%.2f", startX, startY);
        telemetry.addData("End (in)", "X=%.2f  Y=%.2f", endX, endY);
        telemetry.addData("Drift (in)", "dX=%.2f  dY=%.2f  |d|=%.2f", driftX, driftY, driftMag);
        telemetry.addLine("");
        telemetry.addLine("Interpretation:");
        telemetry.addLine("- Good: X/Y stay small while spinning; drift after 360 is small.");
        telemetry.addLine("- If only Y is big: check Y pod direction + Y offset sign (front/back).");
        telemetry.addLine("- If only X is big: check X pod direction + X offset sign (left/right).");
        telemetry.addLine("- If both big: swapped pods, slip, or offset sign/magnitude issue.");
        telemetry.update();

        sleep(3000);
    }

    /**
     * Robot-centric mecanum mixing.
     * x = strafe (+ right), y = forward (+ forward), turn = +CW
     */
    private void setDrivePower(double x, double y, double turn) {
        double lf = y + x + turn;
        double rf = y - x - turn;
        double lb = y - x + turn;
        double rb = y + x - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));

        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);
    }

    private void setBrake(boolean brake) {
        DcMotor.ZeroPowerBehavior zpb = brake
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;

        leftFront.setZeroPowerBehavior(zpb);
        rightFront.setZeroPowerBehavior(zpb);
        leftBack.setZeroPowerBehavior(zpb);
        rightBack.setZeroPowerBehavior(zpb);
    }
}