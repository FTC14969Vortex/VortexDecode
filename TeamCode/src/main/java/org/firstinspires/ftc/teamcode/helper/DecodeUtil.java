package org.firstinspires.ftc.teamcode.helper;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
public class DecodeUtil {

    public static enum AutoType {
        BLUE_NEAR,
        BLUE_FAR,
        RED_NEAR,
        RED_FAR
    }

    public static String getAprilTagType(AutoType autoType){
        String aprilTagType = "";
        if (autoType == AutoType.BLUE_NEAR)
            aprilTagType = DecodeAprilTag.BLUE_APRIL_TAG;
        else if (autoType == AutoType.BLUE_FAR)
            aprilTagType = DecodeAprilTag.BLUE_APRIL_TAG;
        else if (autoType == AutoType.RED_NEAR)
            aprilTagType = DecodeAprilTag.RED_APRIL_TAG;
        else if (autoType == AutoType.RED_FAR)
            aprilTagType = DecodeAprilTag.RED_APRIL_TAG;

        return aprilTagType;

    }

    public static enum NearAutoStages {
        BACK_UP,
        SHOOT,
        GET_MORE_BALLS,
        GO_BACK_TO_SHOOTING_ZONE,
        SHOOT_AGAIN,
        MOVE_OUT_OF_SHOOTING_ZONE,
        END
    }

    public static enum FarAutoStages {
        MOVE_TO_SHOOTING_ZONE,
        SHOOT,
        MOVE_OUT_OF_SHOOTING_ZONE,
        END
    }

    public static Double findRobotDistanceFromAprilTag(DecodeAprilTag aprilTag, AutoType autoType){
        Double robotDistanceFromAprilTag = 0.0;
        AprilTagPoseFtc aprilTagPoseFtc = null;

        if (aprilTag.findAprilTag(getAprilTagType(autoType))) {
            aprilTagPoseFtc = aprilTag.getCoordinate(getAprilTagType(autoType));
            if (aprilTagPoseFtc != null) {
                robotDistanceFromAprilTag = aprilTagPoseFtc.range;
            }
        }
        return robotDistanceFromAprilTag;
    }
}
