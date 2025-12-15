package org.firstinspires.ftc.teamcode.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "Pedro Blue Near Auto 0.06", group = "Autonomous")
public class Sample_Auto_Pathing extends OpMode {
    private Follower follower;

    private Timer pathTimer, opModeTimer;


    public enum PathState{
        //START POSITION_ END POSITION
        //DRIVE-MOVEMENT STATE
        //SHOOT-ATTEPTING TO SCORE


        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        SHOOT_POS_INTAKE_ROW1

    }

    PathState pathState;
    private final Pose startPose = new Pose(18.1,117, Math.toRadians(-40));
    private final Pose shootPose1 = new Pose(56.4,87, Math.toRadians(-40));
    private final Pose intakePose1 = new Pose(15.6,82, Math.toRadians(-90));
    private PathChain driveStartPosShootPos, driveShootPosIntakePos;


    public void buildPaths(){
        // PUT IN CORDINATES FOR START AND END POSITIONS
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

    driveShootPosIntakePos = follower.pathBuilder()
            .addPath(new BezierCurve(shootPose1, intakePose1))
            .setLinearHeadingInterpolation(shootPose1.getHeading(), intakePose1.getHeading())
            .build();




    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);// reset the timer and makes new state.
                break;
            case SHOOT_PRELOAD:
            //check if follower is done with previous path
                if(!follower.isBusy()){
                    pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
                    follower.followPath(driveStartPosShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }

            case SHOOT_POS_INTAKE_ROW1:
                if(!follower.isBusy()){
                    pathState = PathState.SHOOT_PRELOAD ;
                }
                break;
            default:
                break;
        }
    }


    public  void setPathState (PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

    }

    @Override
    public void init (){
    pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
    pathTimer = new Timer();
    opModeTimer = new Timer();
    follower = Constants.createFollower(hardwareMap);
    //TODO add in any other things we need to initialize flywheel, intake, etc.
        buildPaths();
        follower.setStartingPose(startPose);
        
    }


    public void start(){

        opModeTimer.resetTimer();
        setPathState(pathState);
    }
    @Override
    public void loop(){

        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }


}
