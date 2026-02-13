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

/**
 * PinpointOffsetAutoTuner
 *
 * Finds optimal Pinpoint odometry pod offsets by minimising XY positional drift
 * during in-place spins. Works by coordinate-descent: it tests ±step mm around
 * the current best X offset, locks the winner, then does the same for Y, and
 * iterates until nothing improves.
 *
 * ── HOW TO USE ──────────────────────────────────────────────────────────────
 *  1. Physically measure your pod offsets with calipers first and set them in
 *     globals.offsets.xoff / yoff (mm). The auto-tuner refines a good starting
 *     guess — it is NOT a cold-search from 0,0.
 *
 *  2. Place the robot on a flat surface, perfectly still.
 *
 *  3. In init (before pressing Play), the Pinpoint IMU calibrates. Wait for
 *     telemetry to say "Ready — press Play".
 *
 *  4. Press Play.
 *
 *  5. Controls:
 *       A — reset Pinpoint pose (keep robot still!)
 *       B — single CW spin test (diagnostic)
 *       X — single CCW spin test (diagnostic)
 *       Y — run full auto-tune (takes 1-2 min)
 *
 *  6. After auto-tune, telemetry shows the best offsets in both mm and inches,
 *     plus the exact Pedro PinpointConstants lines to copy-paste.
 *
 * ── COORDINATE CONVENTIONS ──────────────────────────────────────────────────
 *  Pinpoint's setOffsets(xMm, yMm):
 *    xMm = how far LEFT of robot centre the X (forward) pod is. Left = positive.
 *    yMm = how far FORWARD of robot centre the Y (strafe) pod is. Forward = positive.
 *
 *  Pedro's PinpointConstants (inches, different axis names!):
 *    forwardPodY = lateral (left/right) distance of the forward pod. Left = positive.
 *    strafePodX  = longitudinal (front/back) distance of the strafe pod. Forward = positive.
 *  => forwardPodY = xMm / 25.4  (same physical quantity, different name)
 *  => strafePodX  = yMm / 25.4  (same physical quantity, different name)
 *
 * ── KEY FIXES VS. ORIGINAL ──────────────────────────────────────────────────
 *  - setOffsets(x, y) takes no DistanceUnit — it always expects mm.
 *  - IMU calibration takes ~250 ms; settle sleeps are now 400 ms minimum.
 *  - Coordinate descent now loops until convergence (not just one pass per step).
 *  - Each candidate is scored over RUNS_PER_CANDIDATE averaged spins to reduce noise.
 *  - Heading unwrap uses a delta method robust to slow loops.
 *  - Final output prints ready-to-paste Pedro constants.
 */
@TeleOp(name = "Pinpoint Offset Auto Tuner", group = "Tuning")
public class PinpointOffsetAutoTuner extends LinearOpMode {

    // ── Initial guess (mm) — set these to your physical measurement ──────────
    private static final double X0_MM = globals.offsets.xoff;
    private static final double Y0_MM = globals.offsets.yoff;

    // ── Spin parameters ───────────────────────────────────────────────────────
    private static final double SPIN_POWER      = 0.28;   // lower = more repeatable
    private static final double TARGET_DEG      = 360.0;
    private static final double STOP_TOL_DEG    = 5.0;    // stop when within this of 360
    private static final double SPIN_TIMEOUT_S  = 25.0;

    // ── Scoring: average over this many CW+CCW pairs per candidate ───────────
    private static final int RUNS_PER_CANDIDATE = 2;      // 2 pairs = 4 spins per candidate

    // ── Coordinate-descent step schedule (mm): coarse → medium → fine ────────
    private static final double[] STEPS_MM = {12.0, 5.0, 2.0, 0.75};

    // ── Convergence: stop early if best doesn't change for this many passes ──
    private static final int MAX_PASSES_PER_STEP = 5;

    // ── Settle time after stopping / before reset (ms) ───────────────────────
    // goBILDA user guide: IMU calibration takes ~250 ms. Use 400 ms to be safe.
    private static final int SETTLE_MS = 400;

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        // ── Hardware map ─────────────────────────────────────────────────────
        leftFront  = hardwareMap.get(DcMotorEx.class, "fl");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        leftBack   = hardwareMap.get(DcMotorEx.class, "bl");
        rightBack  = hardwareMap.get(DcMotorEx.class, "br");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        setBrake(true);
        setDrivePower(0, 0, 0);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED, // X (forward) pod
                GoBildaPinpointDriver.EncoderDirection.FORWARD   // Y (strafe) pod
        );
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Calibrate IMU during init — robot must be still
        // goBILDA docs: "It is critical that the robot be stationary when this
        // zero offset measurement is taken."
        pinpoint.resetPosAndIMU();
        sleep(SETTLE_MS);

        telemetry.addLine("=== Pinpoint Offset Auto Tuner ===");
        telemetry.addLine("Keep robot still during IMU calibration.");
        telemetry.addData("Initial guess (mm)", "x=%.1f  y=%.1f", X0_MM, Y0_MM);
        telemetry.addLine("A=reset | B=CW test | X=CCW test | Y=auto-tune");
        telemetry.addLine("Ready — press Play");
        telemetry.update();

        waitForStart();

        // Working best offsets — updated by auto-tuner
        double bestX = X0_MM;
        double bestY = Y0_MM;
        applyOffsets(bestX, bestY);

        boolean lastA = false, lastB = false, lastX = false, lastY = false;

        while (opModeIsActive()) {
            pinpoint.update();

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;

            // A — reset pose
            if (a && !lastA) {
                setDrivePower(0, 0, 0);
                sleep(SETTLE_MS);
                pinpoint.resetPosAndIMU();
                sleep(SETTLE_MS);
                telemetry.addLine("Pose reset.");
            }

            // B — single CW diagnostic spin
            if (b && !lastB) {
                applyOffsets(bestX, bestY);
                resetPose();
                SpinResult r = spin(+1);
                showSpinResult("CW diagnostic", bestX, bestY, r);
            }

            // X — single CCW diagnostic spin
            if (x && !lastX) {
                applyOffsets(bestX, bestY);
                resetPose();
                SpinResult r = spin(-1);
                showSpinResult("CCW diagnostic", bestX, bestY, r);
            }

            // Y — full auto-tune
            if (y && !lastY) {
                setDrivePower(0, 0, 0);
                sleep(SETTLE_MS);

                telemetry.clearAll();
                telemetry.addLine("AUTO-TUNE STARTED — do not touch robot");
                telemetry.update();

                double[] result = autoTune(bestX, bestY);
                bestX = result[0];
                bestY = result[1];

                applyOffsets(bestX, bestY);
                showFinalResult(bestX, bestY);
            }

            lastA = a; lastB = b; lastX = x; lastY = y;

            // Normal loop telemetry
            telemetry.addLine("A=reset | B=CW | X=CCW | Y=auto-tune");
            telemetry.addData("Current offsets (mm)", "x=%.2f  y=%.2f", bestX, bestY);
            telemetry.addData("Current offsets (in)", "x=%.4f  y=%.4f",
                    bestX / 25.4, bestY / 25.4);
            telemetry.addData("Pos (in)", "X=%.3f  Y=%.3f",
                    pinpoint.getPosX(DistanceUnit.INCH),
                    pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.2f",
                    pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // AUTO-TUNER
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Coordinate-descent optimisation over X then Y offsets.
     * For each step size, alternates between tuning X and Y until neither axis
     * improves (convergence), then shrinks the step and repeats.
     *
     * Returns double[] { bestXmm, bestYmm }.
     */
    private double[] autoTune(double startX, double startY) {
        double curX = startX;
        double curY = startY;

        for (double step : STEPS_MM) {
            if (!opModeIsActive()) break;

            boolean improved = true;
            int pass = 0;

            // Loop until neither axis improves at this step size
            while (improved && pass < MAX_PASSES_PER_STEP && opModeIsActive()) {
                improved = false;
                pass++;

                telemetry.clearAll();
                telemetry.addData("Step (mm)", "%.2f  |  Pass %d / %d",
                        step, pass, MAX_PASSES_PER_STEP);
                telemetry.addData("Current (mm)", "x=%.2f  y=%.2f", curX, curY);
                telemetry.update();

                // ── Tune X ───────────────────────────────────────────────────
                double newX = tuneAxis(curX, curY, step, true);
                if (Math.abs(newX - curX) > 0.01) {
                    curX = newX;
                    improved = true;
                }

                if (!opModeIsActive()) break;

                // ── Tune Y ───────────────────────────────────────────────────
                double newY = tuneAxis(curX, curY, step, false);
                if (Math.abs(newY - curY) > 0.01) {
                    curY = newY;
                    improved = true;
                }
            }
        }

        return new double[]{curX, curY};
    }

    /**
     * Tests three candidates for one axis (base, +step, -step) and returns
     * the value that gives the lowest average drift score.
     */
    private double tuneAxis(double xMm, double yMm, double stepMm, boolean tuneX) {
        String axisName = tuneX ? "X" : "Y";

        double base = tuneX ? xMm : yMm;
        double plus  = base + stepMm;
        double minus = base - stepMm;

        // Build three candidate pairs
        double[] candidates = {base, plus, minus};
        String[] labels     = {"base", "+step", "-step"};
        double[] scores     = new double[3];

        for (int i = 0; i < 3; i++) {
            if (!opModeIsActive()) break;

            double cx = tuneX ? candidates[i] : xMm;
            double cy = tuneX ? yMm : candidates[i];

            telemetry.clearAll();
            telemetry.addData("Tuning", "axis=%s  step=%.2fmm", axisName, stepMm);
            telemetry.addData("Testing", "%s  (%.2f mm)", labels[i], candidates[i]);
            telemetry.update();

            scores[i] = scoreOffsets(cx, cy);
        }

        // Pick winner
        int bestIdx = 0;
        for (int i = 1; i < 3; i++) {
            if (scores[i] < scores[bestIdx]) bestIdx = i;
        }

        telemetry.clearAll();
        telemetry.addData("Axis " + axisName + " result", "");
        for (int i = 0; i < 3; i++) {
            telemetry.addData("  " + labels[i], "%.3f in  %s",
                    scores[i], i == bestIdx ? "<< BEST" : "");
        }
        telemetry.addData("Winner", "%.2f mm", candidates[bestIdx]);
        telemetry.update();
        sleep(500);

        return candidates[bestIdx];
    }

    /**
     * Scores an offset pair by averaging XY drift magnitude over RUNS_PER_CANDIDATE
     * CW+CCW spin pairs. Lower score = less drift = better offsets.
     */
    private double scoreOffsets(double xMm, double yMm) {
        applyOffsets(xMm, yMm);

        double totalDrift = 0.0;
        int count = 0;

        for (int run = 0; run < RUNS_PER_CANDIDATE && opModeIsActive(); run++) {
            // CW spin
            resetPose();
            SpinResult cw = spin(+1);
            totalDrift += cw.driftMagIn;
            count++;

            if (!opModeIsActive()) break;

            // CCW spin
            resetPose();
            SpinResult ccw = spin(-1);
            totalDrift += ccw.driftMagIn;
            count++;
        }

        return count > 0 ? totalDrift / count : Double.MAX_VALUE;
    }

    // ═════════════════════════════════════════════════════════════════════════
    // SPIN PRIMITIVE
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Spins in place ~360° and returns the XY drift from start.
     * dir = +1 for CW (negative yaw accumulation), -1 for CCW.
     *
     * Uses delta-based heading accumulation which is robust to slow loops:
     * each tick's delta is clamped to ±180° so even a skipped update can't
     * cause a false wrap.
     */
    private SpinResult spin(int dir) {
        pinpoint.update();
        double startX = pinpoint.getPosX(DistanceUnit.INCH);
        double startY = pinpoint.getPosY(DistanceUnit.INCH);

        double lastHeadingDeg  = pinpoint.getHeading(AngleUnit.DEGREES);
        double accumulatedDeg  = 0.0;

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < SPIN_TIMEOUT_S) {
            pinpoint.update();

            double h   = pinpoint.getHeading(AngleUnit.DEGREES);
            double dh  = h - lastHeadingDeg;

            // Normalise delta to (-180, +180]
            while (dh >  180.0) dh -= 360.0;
            while (dh < -180.0) dh += 360.0;

            accumulatedDeg += dh;
            lastHeadingDeg  = h;

            if (Math.abs(accumulatedDeg) >= (TARGET_DEG - STOP_TOL_DEG)) break;

            setDrivePower(0, 0, SPIN_POWER * dir);
        }

        setDrivePower(0, 0, 0);
        sleep(150); // let robot physically stop before reading final pose

        pinpoint.update();
        double endX = pinpoint.getPosX(DistanceUnit.INCH);
        double endY = pinpoint.getPosY(DistanceUnit.INCH);

        double dx  = endX - startX;
        double dy  = endY - startY;
        double mag = Math.hypot(dx, dy);

        return new SpinResult(dx, dy, mag, Math.abs(accumulatedDeg));
    }

    // ═════════════════════════════════════════════════════════════════════════
    // HELPERS
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Stops motors, waits for settle, then resets Pinpoint pose + IMU.
     * The goBILDA user guide states IMU calibration takes ~250 ms, so we wait
     * SETTLE_MS (400 ms) after the reset command before trusting the IMU.
     */
    private void resetPose() {
        setDrivePower(0, 0, 0);
        sleep(SETTLE_MS);
        pinpoint.resetPosAndIMU();
        sleep(SETTLE_MS);
    }

    /**
     * Sets Pinpoint offsets. Per the goBILDA Java driver API, setOffsets takes
     * two doubles in MILLIMETRES — there is no DistanceUnit overload.
     *   xMm: left of centre = positive (forward pod lateral offset)
     *   yMm: forward of centre = positive (strafe pod longitudinal offset)
     */
    private void applyOffsets(double xMm, double yMm) {
        pinpoint.setOffsets(xMm, yMm, DistanceUnit.MM);
    }

    private void showSpinResult(String label, double xMm, double yMm, SpinResult r) {
        telemetry.clearAll();
        telemetry.addData("Test", label);
        telemetry.addData("Offsets (mm)", "x=%.2f  y=%.2f", xMm, yMm);
        telemetry.addData("Rotation (deg)", "%.1f", r.rotDeg);
        telemetry.addData("Drift X (in)", "%.4f", r.driftXIn);
        telemetry.addData("Drift Y (in)", "%.4f", r.driftYIn);
        telemetry.addData("Drift |d| (in)", "%.4f", r.driftMagIn);
        telemetry.update();
        sleep(2000);
    }

    /**
     * Shows final auto-tune result, including ready-to-paste Pedro constants.
     *
     * Coordinate mapping:
     *   Pedro forwardPodY = Pinpoint xMm / 25.4
     *   Pedro strafePodX  = Pinpoint yMm / 25.4
     *
     * Note: Pedro uses INCH and its own axis naming (forwardPodY / strafePodX).
     * These map to the same physical distances as Pinpoint's xOffset / yOffset
     * respectively, just in different units.
     */
    private void showFinalResult(double xMm, double yMm) {
        double xIn = xMm / 25.4;
        double yIn = yMm / 25.4;

        telemetry.clearAll();
        telemetry.addLine("══ AUTO-TUNE COMPLETE ══════════════════");
        telemetry.addData("Best X offset", "%.2f mm  /  %.4f in", xMm, xIn);
        telemetry.addData("Best Y offset", "%.2f mm  /  %.4f in", yMm, yIn);
        telemetry.addLine("");
        telemetry.addLine("── Copy into PinpointConstants (Pedro): ──");
        telemetry.addData(".forwardPodY", "(%.4f)   // inches", xIn);
        telemetry.addData(".strafePodX",  "(%.4f)   // inches", yIn);
        telemetry.addLine("");
        telemetry.addLine("── Or into globals.offsets (mm): ──");
        telemetry.addData(".xoff", "= %.2f", xMm);
        telemetry.addData(".yoff", "= %.2f", yMm);
        telemetry.addLine("════════════════════════════════════════");
        telemetry.update();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // DATA CLASSES
    // ═════════════════════════════════════════════════════════════════════════

    private static class SpinResult {
        final double driftXIn, driftYIn, driftMagIn, rotDeg;

        SpinResult(double driftXIn, double driftYIn, double driftMagIn, double rotDeg) {
            this.driftXIn   = driftXIn;
            this.driftYIn   = driftYIn;
            this.driftMagIn = driftMagIn;
            this.rotDeg     = rotDeg;
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // DRIVE HELPERS
    // ═════════════════════════════════════════════════════════════════════════

    /** Robot-centric mecanum power mixing with automatic normalisation. */
    private void setDrivePower(double x, double y, double turn) {
        double lf = y + x + turn;
        double rf = y - x - turn;
        double lb = y - x + turn;
        double rb = y + x - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

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
