package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.NotOpModes.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Big Blue Booti")
public class BlueAuto extends OpMode {

    double maxp = 0.85;

    /* =========================
       STATE MACHINE
       ========================= */
    private enum AutoState {
        PRE_DIDDY,
        SHOOT_1,

        TAKE_CP_PHOTOS,
        
        JERK_AND_JACK,
        
        PARK,
        DONE,
    }

    private AutoState state;

    /* =========================
       HARDWARE
       ========================= */
    private Follower follower;
    private Drivetrain drivetrain;
    private Paths paths;
    private TelemetryManager telemetryM;
    private final Timer stateTimer = new Timer();
    private int[] jackOrder;
    private int i = 1;
    private int progress = 0;
    /* =========================
       INIT
       ========================= */
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(26.959, 127.625, Math.toRadians(135))
        );
        follower.update();

        paths = new Paths(follower);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        drivetrain.setFlywheelRPM(-2767);
        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);

        follower.followPath(paths.Shooting1);

        state = AutoState.PRE_DIDDY;
        stateTimer.resetTimer();
    }

    /* =========================
       LOOP
       ========================= */
    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        drivetrain.setFlywheelRPM(-2767);

        switch (state) {
            case PRE_DIDDY:

                drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
                if (stateTimer.getElapsedTimeSeconds() > 2.367) {
                    setShootingState();
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
                }
            
            case SHOOT_1:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 3) {
                    setShootingState();
                    follower.followPath(paths.takePhotos);
                    state = AutoState.TAKE_CP_PHOTOS;
                }
                break;
            case TAKE_CP_PHOTOS:
                jackOrder = useCamera();
                if (jackOrder.length > 0) {
                    state = AutoState.JERK_AND_JACK; 
                }
                break;
            
            case JERK_AND_JACK:
            {                
                switch (progress) {
                    case 0:
                        goToJerk(jackOrder[i]);
                        break;
                    case 1:
                        jerk(jackOrder[i], 0.4); //the double (0.4) is movement speed
                        break;
                    case 2:
                        if (cockBlocked()) {
                            goToJerk(jackOrder[i]);
                        } else progress++;
                        break;
                    case 3:
                        goToJack(jackOrder[i], new double[] {1.4, 1.8, 1.9}); //double is the times to wait before jacking
                        break;
                    case 4:
                        jack();
                        break;
                        
                }
                break;
            }

            case PARK:
                if (!follower.isBusy()) {
                    state = AutoState.DONE;
                }
                break;
 
            case DONE:
                drivetrain.setIntakePower(0);
                drivetrain.setFeederPower(0);
                break;
        }

        telemetryM.debug("Auto State", state);
        telemetryM.debug("State Time", stateTimer.getElapsedTimeSeconds());
    }
    /* =========================
       PICK UP ORDER FOR SHOOOTING
       ========================= */
    private int[] useCamera() {
        return new int[] {0, 1, 2, 3}; //can comment out once figurout how to use camera
        int result = 1;
        switch (result) {
            case 1:
                return new int[] {0, 1, 2, 3};
            case 2:
                return new int[] {0, 2, 1, 3};
            case 3:
                return new int[] {0, 3, 1, 2};
        }
    }

    private boolean cockBlocked() {
        return jackOrder[i] > i; //stops robot from going through balls that it hasnt picked up yet
    }
    private void goToJerk(int stack) {
        if (!follower.isBusy()) {
            switch (stack) {
            case 1:
                follower.followPath(paths.PickStart1);
                progress++;
                break;
            case 2:
                follower.followPath(paths.PickStart2);
                progress++;
                break;
            case 3:
                follower.followPath(paths.PickStart3);
                progress++;
                break;
            }
        }
    }

    public void jerk(int stack, double power) {
        if (!follower.isBusy()) {
            drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
            setPickupState();
            follower.setMaxPower(power);
            switch (stack) {
                case 1:
                    follower.followPath(paths.PickEnd1);
                    break;
                case 2:
                    follower.followPath(paths.PickEnd2);
                    break;
                case 3:
                    follower.followPath(paths.PickEnd1);
                    break;
            }
            follower.setMaxPower(maxp);
            progress++;
        }
    }    
    private void goToJack(int stack, double[] times) {
        if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= times[i]) { 
            drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
            follower.setMaxPower(maxp);
            switch (stack) {
                case 1:
                    follower.followPath(paths.Shooting2);
                    break;
                case 2:
                    follower.followPath(paths.Shooting3);
                    break;
                case 3:
                    follower.followPath(paths.Shooting4);
                    break;
            }
            stateTimer.resetTimer();
            progress++;
        }
    }

    private void jack() {
        if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2.5) {
            follower.setMaxPower(maxp);
            if (i != 3) {
                setPickupState();
                stateTimer.resetTimer();
            } else state = AutoState.PARK;
            i++;
            progress = 0;
        }
    }
                
              
    /* =========================
       MECHANISM STATES
       ========================= */
    private void setShootingState() {
        drivetrain.setIntakePower(-1.0);
        drivetrain.setFeederPower(1.0);
    }

    private void setPickupState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
        drivetrain.setIntakePower(-1.0);
        drivetrain.setFeederPower(0.6);
    }

    private void setIdleState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
        drivetrain.setIntakePower(0);
        drivetrain.setFeederPower(0);
    }

    /* =========================
       PATH DEFINITIONS (NEW)
       ========================= */
    public static class Paths {

        public PathChain Shooting1;
        public PathChain takePhotos;
        
        public PathChain PickStart1;
        public PathChain PickEnd1;

        public PathChain Shooting2;

        public PathChain PickStart2;
        public PathChain PickEnd2;

        public PathChain Shooting3;

        public PathChain PickStart3;
        public PathChain PickEnd3;

        public PathChain Shooting4;

        public PathChain END;

        public Paths(Follower follower) {

            Shooting1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.959, 127.625),
                                    new Pose(56.208, 87.056)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                    .build();
            takePhotos = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.959, 127.625),
                                    new Pose(56.208, 87.056)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                    .build();
            PickStart1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.208, 87.056),
                                    new Pose(46.813, 82.658)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            PickEnd1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.813, 82.658),
                                    new Pose(15, 82.394)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setVelocityConstraint(0.25)

                    .build();

            Shooting2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15, 82.394),
                                    new Pose(56.737, 86.721)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            PickStart2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.737, 86.721),
                                    new Pose(47.993, 58.614)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            PickEnd2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.993, 58.614),
                                    new Pose(16.5, 58.826)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setVelocityConstraint(0.25)

                    .build();

            Shooting3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.5, 58.826),
                                    new Pose(56.635, 86.983)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            PickStart3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.635, 86.983),
                                    new Pose(47.307, 33.913)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            PickEnd3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.307, 33.913),
                                    new Pose(12, 33.970)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setVelocityConstraint(0.25)

                    .build();

            Shooting4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12, 33.970),
                                    new Pose(56.321, 86.903)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            END = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.321, 86.903),
                                    new Pose(48.563, 13.541)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }
}
