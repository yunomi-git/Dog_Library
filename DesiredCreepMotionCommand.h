#ifndef __CREEPMOTIONCOMMAND__
#define __CREEPMOTIONCOMMAND__

struct DesiredCreepMotionCommand {
    Point translation;
    Rot rotation;
    Rot leg_rotation;
    Rot current_rotation;

    // DesiredCreepMotionCommand() {
    //     translation = POINT_ZERO;
    //     rotation = ROT_ZERO;
    //     leg_rotation = ROT_ZERO;
    // }
};

#endif