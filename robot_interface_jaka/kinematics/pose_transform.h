#ifndef POSE_TRANSFORM_H
#define POSE_TRANSFORM_H

struct PoseTransform
{
    float roll;
    float pitch;
    float yaw;
    float offset_x;
    float offset_y;
    float offset_z;

    PoseTransform():
    roll(0), pitch(0), yaw(0), offset_x(0), offset_y(0), offset_z(0)
    {;}
    PoseTransform(int a1, int a2, int a3, int a4, int a5, int a6):
    offset_x(a1), offset_y(a2), offset_z(a3), roll(a4), pitch(a5), yaw(a6)
    {;}
};

#endif