#include "Global/Global.hpp"

float64 Deg2Rad(const float64 &f64_Deg)
{
    return f64_Deg * M_PI / 180.0;
}

float32 Deg2Rad(const float32 &f32_Deg)
{
    return f32_Deg * M_PI / 180.0f;
}

float64 Rad2Deg(const float64 &f64_Rad)
{
    return f64_Rad * 180.0 / M_PI;
}

float32 Rad2Deg(const float32 &f32_Rad)
{
    return f32_Rad * 180.0f / M_PI;
}
// ==================================== RAD<->DEG ==================================== //

// ==================================== Scale ==================================== //
float32 Scale2D(float32 &f32_X, float32 &f32_Y)
{
    return sqrt(pow(f32_X, 2) + pow(f32_Y, 2));
}

float32 Scale2D(float32 *arf_X, float32 *arf_Y, int32 s32_Idx)
{
    return sqrt(pow(arf_X[s32_Idx], 2) + pow(arf_Y[s32_Idx], 2));
}
float32 Scale2D(Point2D &st_Point2D)
{
    return sqrt(pow(st_Point2D.f32_X, 2) + pow(st_Point2D.f32_Y, 2));
}

float32 Scale3D(float32 &f32_X, float32 &f32_Y, float32 &f32_Z)
{
    return sqrt(pow(f32_X, 2) + pow(f32_Y, 2) + pow(f32_Z, 2));
}

float32 Scale3D(float32 *arf_X, float32 *arf_Y, float32 *arf_Z, int32 s32_Idx)
{
    return sqrt(pow(arf_X[s32_Idx], 2) + pow(arf_Y[s32_Idx], 2) + pow(arf_Z[s32_Idx], 2));
}

float32 Scale3D(Point3D &st_Point3D)
{
    return sqrt(pow(st_Point3D.f32_X, 2) + pow(st_Point3D.f32_Y, 2) + pow(st_Point3D.f32_Z, 2));
}
// ==================================== Scale ==================================== //


// ==================================== Normalize ==================================== //
float32 Normalize2D(float32 f32_X, float32 f32_Y)
{
    return sqrt(pow(f32_X, 2) + pow(f32_Y, 2));
}

float32 Normalize2D(float32 f32_X1, float32 f32_Y1, float32 f32_X2, float32 f32_Y2)
{
    return sqrt(pow(f32_X1 - f32_X2, 2) + pow(f32_Y1 - f32_Y2, 2));
}

float32 Normalize2D(float32 *arf_X, float32 *arf_Y, int32 s32_Idx1, int32 s32_Idx2)
{
    return sqrt(pow(arf_X[s32_Idx1] - arf_X[s32_Idx2], 2) + pow(arf_Y[s32_Idx1] - arf_Y[s32_Idx2], 2));
}

float32 Normalize2D(float32 *arf_X, float32 *arf_Y, int32 s32_Idx1, float32 &f32_X, float32 &f32_Y)
{
    return sqrt(pow(arf_X[s32_Idx1] - f32_X, 2) + pow(arf_Y[s32_Idx1] - f32_Y, 2));
}

float32 Normalize2D(float32 *arf_X, float32 *arf_Y, int32 s32_Idx1, Point2D &st_P2)
{
    return sqrt(pow(arf_X[s32_Idx1] - st_P2.f32_X, 2) + pow(arf_Y[s32_Idx1] - st_P2.f32_Y, 2));
}

float32 Normalize2D(Point2D &st_P1, Point2D &st_P2)
{
    return sqrt(pow(st_P1.f32_X - st_P2.f32_X, 2) + pow(st_P1.f32_Y - st_P2.f32_Y, 2));
}

float32 Normalize2D(std::vector<float32> &vf_P1, Point2D &st_P2)
{
    return sqrt(pow(vf_P1[0] - st_P2.f32_X, 2) + pow(vf_P1[1] - st_P2.f32_Y, 2));
}

float32 Normalize2D(std::vector<float32> &vf_P1, float32 &f32_X, float32 &f32_Y)
{
    return sqrt(pow(vf_P1[0] - f32_X, 2) + pow(vf_P1[1] - f32_Y, 2));
}

float32 Normalize3D(float32 &f32_X1, float32 &f32_Y1, float32 &f32_Z1, float32 &f32_X2, float32 &f32_Y2, float32 &f32_Z2)
{
    return sqrt(pow(f32_X1 - f32_X2, 2) + pow(f32_Y1 - f32_Y2, 2) + pow(f32_Z1 - f32_Z2, 2));
}

float32 Normalize3D(float32 *arf_X, float32 *arf_Y, float32 *arf_Z, int32 &s32_Idx1, int32 &s32_Idx2)
{
    return sqrt(pow(arf_X[s32_Idx1] - arf_X[s32_Idx2], 2) + pow(arf_Y[s32_Idx1] - arf_Y[s32_Idx2], 2) + pow(arf_Z[s32_Idx1] - arf_Z[s32_Idx2], 2));
}

float32 Normalize3D(Point3D &st_P1, Point3D &st_P2)
{
    return sqrt(pow(st_P1.f32_X - st_P2.f32_X, 2) + pow(st_P1.f32_Y - st_P2.f32_Y, 2) + pow(st_P1.f32_Z - st_P2.f32_Z, 2));
}

float32 NormalizeAngle(float32 angle)
{
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return angle;
}

float32 NormalizeDWAFactor(float32* arr, int32 size, int32 idx) {
    float32 min_val = arr[0], max_val = arr[0];
    for (int32 i = 1; i < size; ++i) {
        if (arr[i] < min_val) min_val = arr[i];
        if (arr[i] > max_val) max_val = arr[i];
    }
    float32 range = max_val - min_val;
    if (range == 0.0f)
        return 0.0f;
    else
        return (arr[idx] - min_val) / range;
}
// ==================================== Normalize ==================================== //


// ==================================== Velocity ==================================== //
float32 KPH2MPS(float32 f32_KPH)
{
    return f32_KPH / 3.6;
}

float32 MPS2KPH(float32 f32_MPS)
{
    return f32_MPS * 3.6;
}
// ==================================== Velocity ==================================== //


// ==================================== Vector ==================================== //
Point2D UnitVector(Point2D &st_Point2DStart, Point2D &st_Point2DEnd)
{
    float32 f32_Scale = Normalize2D(st_Point2DStart, st_Point2DEnd);
    Point2D st_UnitVector = {(st_Point2DEnd.f32_X - st_Point2DStart.f32_X) / f32_Scale,
                               (st_Point2DEnd.f32_Y - st_Point2DStart.f32_Y) / f32_Scale};
    return st_UnitVector;
}
Point2D UnitVector(float32 f32_XStart, float32 f32_YStart, float32 f32_XEnd, float32 f32_YEnd)
{
    float32 f32_Scale = Normalize2D(f32_XStart, f32_YStart, f32_XEnd, f32_YEnd);
    Point2D st_UnitVector = {(f32_XEnd - f32_XStart) / f32_Scale,
                               (f32_YEnd - f32_YStart) / f32_Scale};
    return st_UnitVector;
}

float32 CrossProduct(Point2D &st_RefPoint, Point2D &st_Point)
{
    return st_RefPoint.f32_X * st_Point.f32_Y - st_RefPoint.f32_Y * st_Point.f32_X;
}
// ==================================== Vector ==================================== //


// ==================================== LoadData ==================================== //
void LoadData(std::ifstream &st_LoadFile, Path &st_Path)
{
    std::string s_FileLine = "";
    float32 f32_X = 0, f32_Y = 0;
    int32 s32_Size = 0;
    while (std::getline(st_LoadFile, s_FileLine))
    {
        std::istringstream st_Stream(s_FileLine);
        st_Stream >> f32_X >> f32_Y;
        st_Path.arf32_E[s32_Size] = f32_X;
        st_Path.arf32_N[s32_Size] = f32_Y;
        s32_Size++;
    }
    st_Path.s32_Size = s32_Size;
    st_LoadFile.close();
}

// void LoadData(std::ifstream &st_LoadFile, HighWayPath &st_Path)
// {
//     std::string s_FileLine = "";
//     float32 f32_X = 0, f32_Y = 0;
//     int32 s32_Size = 0;
//     while (std::getline(st_LoadFile, s_FileLine))
//     {
//         std::istringstream st_Stream(s_FileLine);
//         st_Stream >> f32_X >> f32_Y;
//         st_Path.arf32_E[s32_Size] = f32_X;
//         st_Path.arf32_N[s32_Size] = f32_Y;
//         s32_Size++;
//     }
//     st_Path.s32_Size = s32_Size;
//     st_LoadFile.close();
// }

void LoadData(std::ifstream &st_LoadFile, Map &st_Map)
{
    std::string s_FileLine = "";
    float32 f32_X = 0, f32_Y = 0;
    int32 s32_Size = 0;
    while (std::getline(st_LoadFile, s_FileLine))
    {
        std::istringstream st_Stream(s_FileLine);
        st_Stream >> f32_X >> f32_Y;
        st_Map.arf32_E[s32_Size] = f32_X;
        st_Map.arf32_N[s32_Size] = f32_Y;
        s32_Size++;
    }
    st_Map.s32_Size = s32_Size;
    st_LoadFile.close();
}
void LoadIdx(std::ifstream &st_LoadFile, IdxSet &st_IdxSet)
{
    std::string s_FileLine = "";
    int32 s32_Size = 0;
    int32 s32_Start = 0, s32_End = 0;
    while (std::getline(st_LoadFile, s_FileLine))
    {
        std::istringstream st_Stream(s_FileLine);
        st_Stream >> s32_Start >> s32_End;
        st_IdxSet.arp_Idx[s32_Size].first = s32_Start;
        st_IdxSet.arp_Idx[s32_Size].second = s32_End;
        s32_Size++;
    }
    st_IdxSet.s32_Num = s32_Size;
}
// ==================================== LoadData ==================================== //

// ==================================== Sigmoid ==================================== //
float32 Sigmoid(float32 f32_Value)
{
    return 1 / (1 + expf(-f32_Value));
}
// ==================================== Sigmoid ==================================== //

// ==================================== RayCasting ==================================== //
bool InOrOut(Planning &st_Planning, float32 f32_PointX, float32 f32_PointY)
{
    int32 s32_MeetNum = 0;
    float32 f32_X, f32_NextX, f32_Y, f32_NextY, f32_Value;
    Map *st_Map[2] = {&st_Planning.st_InBoundary1,
                        &st_Planning.st_OutBoundary};

    for (int32 s32_i = 0; s32_i < 2; s32_i++)
    {
        for (int32 s32_j = 0; s32_j < st_Map[s32_i]->s32_Size; s32_j++)
        {
            if (st_Map[s32_i]->s32_Size == 0)
                break;
            f32_X = st_Map[s32_i]->arf32_E[s32_j];
            f32_NextX = st_Map[s32_i]->arf32_E[(s32_j + 1) % st_Map[s32_i]->s32_Size];
            f32_Y = st_Map[s32_i]->arf32_N[s32_j];
            f32_NextY = st_Map[s32_i]->arf32_N[(s32_j + 1) % st_Map[s32_i]->s32_Size];

            if ((f32_Y >= f32_PointY && f32_NextY < f32_PointY) || (f32_Y < f32_PointY && f32_NextY >= f32_PointY))
            {
                f32_Value = (f32_NextX + (f32_PointY - f32_NextY) / (f32_Y - f32_NextY) * (f32_X - f32_NextX));
                if (f32_Value < f32_PointX)
                    s32_MeetNum += 1;
            }
        }
    }

     if (s32_MeetNum % 2 == 0)
        return false;
    else
        return true;
}

bool InOrOut_lane(Planning &st_Planning, float32 f32_PointX, float32 f32_PointY)
{
    int32 s32_MeetNum = 0;
    float32 f32_X, f32_NextX, f32_Y, f32_NextY, f32_Value;
    Map *st_Map[2] = {&st_Planning.st_InBoundary2,
                        &st_Planning.st_InBoundary3};

    for (int32 s32_i = 0; s32_i < 2; s32_i++)
    {
        for (int32 s32_j = 0; s32_j < st_Map[s32_i]->s32_Size; s32_j++)
        {
            if (st_Map[s32_i]->s32_Size == 0)
                break;
            f32_X = st_Map[s32_i]->arf32_E[s32_j];
            f32_NextX = st_Map[s32_i]->arf32_E[(s32_j + 1) % st_Map[s32_i]->s32_Size];

            f32_Y = st_Map[s32_i]->arf32_N[s32_j];
            f32_NextY = st_Map[s32_i]->arf32_N[(s32_j + 1) % st_Map[s32_i]->s32_Size];

            if ((f32_Y >= f32_PointY && f32_NextY < f32_PointY) || (f32_Y < f32_PointY && f32_NextY >= f32_PointY))
            {
                f32_Value = (f32_NextX + (f32_PointY - f32_NextY) / (f32_Y - f32_NextY) * (f32_X - f32_NextX));
                if (f32_Value < f32_PointX)
                    s32_MeetNum += 1;
            }
        }
    }

    if (s32_MeetNum % 2 == 0)
        return false;
        
    else
        return true;
}
// ==================================== RayCasting ==================================== //


// ==================================== FindNear ==================================== //
int32 FindNear(Planning &st_Planning, float32 &f32_X, float32 &f32_Y, int32 s32_FindNearLen, int32 s32_PathType)
{
    int32 s32_NearIdx = 0;
    Path *st_Path = NULL;
    float32 arf_Dist[s32_FindNearLen] = {0};

    if (s32_PathType == c_GLOBALPATH)
    {
        st_Path = &st_Planning.st_GlobalPath;
        s32_NearIdx = st_Planning.st_GlobalPath.s32_NearIdx;
    }
    else if (s32_PathType == c_LOCALPATH)
        st_Path = &st_Planning.st_LocalPath;

    for (int32 s32_i = s32_NearIdx; s32_i < s32_NearIdx + s32_FindNearLen; s32_i++)
    {
        arf_Dist[s32_i - s32_NearIdx] = Normalize2D(st_Path->arf32_E, st_Path->arf32_N, s32_i % st_Path->s32_Size, f32_X, f32_Y);
    }

    return s32_NearIdx = (s32_NearIdx + std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + s32_FindNearLen))) % st_Path->s32_Size;
}

int32 FindNear(Planning &st_Planning, Point2D &st_Point2D, int32 s32_StartIdx, int32 s32_FindNearLen, int32 s32_PathType)
{
    int32 s32_NearIdx = s32_StartIdx;
    Path *st_Path;
    float32 arf_Dist[s32_FindNearLen] = {0};
    if (s32_PathType == c_GLOBALPATH)
        st_Path = &st_Planning.st_GlobalPath;
    else if (s32_PathType == c_LOCALPATH)
        st_Path = &st_Planning.st_LocalPath;

    for (int32 s32_i = 0; s32_i < s32_FindNearLen; s32_i++)
    {
        arf_Dist[s32_i] = Normalize2D(st_Path->arf32_E, st_Path->arf32_N, (s32_i + s32_StartIdx) % st_Path->s32_Size, st_Point2D);
    }

    return s32_NearIdx = (s32_NearIdx + std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + s32_FindNearLen))) % st_Path->s32_Size;
}

int32 FindClostIdx(Planning &st_Planning, float32 f32_E, float32 f32_N)
{
    Path& st_TmpPath = st_Planning.s32_PathType == c_LOCALPATH ? st_Planning.st_LocalPath : st_Planning.st_GlobalPath;
    int32 s32_StartIndex = 0;
    int32 s32_EndIndex = st_TmpPath.s32_Size;
    float32 f32_MinDist = Normalize2D(f32_E, f32_N, st_TmpPath.arf32_E[s32_StartIndex], st_TmpPath.arf32_N[s32_StartIndex]);
    int32 s32_ClostIdx = s32_StartIndex;

    for (int32 index = s32_StartIndex + 1; index <= s32_EndIndex; index++)
    {
        float32 f32_Dist = Normalize2D(f32_E, f32_N, st_TmpPath.arf32_E[index], st_TmpPath.arf32_N[index]);
        if (f32_Dist < f32_MinDist)
        {
            f32_MinDist = f32_Dist;
            s32_ClostIdx = index;
        }
    }
    return s32_ClostIdx;
}

int32 FindClostIdx(Planning &st_Planning, float32 f32_E, float32 f32_N, int32 PathType)
{
    Path& st_TmpPath = PathType == c_LOCALPATH ? st_Planning.st_LocalPath : st_Planning.st_GlobalPath;
    int32 s32_StartIndex = 0;
    int32 s32_EndIndex = st_TmpPath.s32_Size;
    float32 f32_MinDist = Normalize2D(f32_E, f32_N, st_TmpPath.arf32_E[s32_StartIndex], st_TmpPath.arf32_N[s32_StartIndex]);
    int32 s32_ClostIdx = s32_StartIndex;

    for (int32 index = s32_StartIndex + 1; index <= s32_EndIndex; index++)
    {
        float32 f32_Dist = Normalize2D(f32_E, f32_N, st_TmpPath.arf32_E[index], st_TmpPath.arf32_N[index]);
        if (f32_Dist < f32_MinDist)
        {
            f32_MinDist = f32_Dist;
            s32_ClostIdx = index;
        }
    }
    return s32_ClostIdx;
}

void FindClostIdx(GPS_IMU& st_GPSIMU, Planning &st_Planning)
{
    int32 s32_StartIndex = 0;
    int32 s32_EndIndex;
    float32 f32_MinDist;
    s32_EndIndex = st_Planning.st_GlobalPath.s32_Size;
    f32_MinDist = Normalize2D(st_GPSIMU.st_GPS.f64_E, st_GPSIMU.st_GPS.f64_N, st_Planning.st_GlobalPath.arf32_E[s32_StartIndex], st_Planning.st_GlobalPath.arf32_N[s32_StartIndex]);
    int32 s32_ClostIdx = s32_StartIndex;
    for (int32 index = s32_StartIndex + 1; index < s32_EndIndex; index++)
    {
        float32 f32_Dist = Normalize2D(st_GPSIMU.st_GPS.f64_E, st_GPSIMU.st_GPS.f64_N, st_Planning.st_GlobalPath.arf32_E[index], st_Planning.st_GlobalPath.arf32_N[index]);
        if (f32_Dist < f32_MinDist)
        {
            f32_MinDist = f32_Dist;
            s32_ClostIdx = index;
        }
    }
    if(st_GPSIMU.st_GPS.f64_E == 0.0f) s32_ClostIdx = 23;
    st_Planning.st_GlobalPath.s32_NearIdx = s32_ClostIdx;

    if(st_Planning.s32_PathType == c_LOCALPATH){
        st_Planning.st_LocalPath.s32_NearIdx = FindClostIdx(st_Planning,st_GPSIMU.st_GPS.f64_E, st_GPSIMU.st_GPS.f64_N);
    }
}


int32 FindClostIdx(Path &st_Path, Point2D& st_Point, int32 s32_SearchLen)
{
    const int32 s32_Size = st_Path.s32_Size;
    if (s32_Size <= 0) return 0;

    int32 s32_StartIndex = st_Path.s32_NearIdx;
    if (s32_StartIndex < 0) s32_StartIndex = 0;
    if (s32_StartIndex >= s32_Size) s32_StartIndex = s32_Size - 1;

    int32 s32_Left = std::max(0, s32_StartIndex - s32_SearchLen);
    int32 s32_Right = std::min(s32_Size - 1, s32_StartIndex + s32_SearchLen);

    float32 f32_MinDist = std::numeric_limits<float32>::max();
    int32 s32_ClostIdx = s32_StartIndex;

    for (int32 s32_Index = s32_Left; s32_Index <= s32_Right; s32_Index++)
    {
        float32 f32_Dx = st_Path.arf32_E[s32_Index] - st_Point.f32_X;
        float32 f32_Dy = st_Path.arf32_N[s32_Index] - st_Point.f32_Y;
        float32 f32_Dist = f32_Dx * f32_Dx + f32_Dy * f32_Dy;
        if (f32_Dist < f32_MinDist)
        {
            f32_MinDist = f32_Dist;
            s32_ClostIdx = s32_Index;
        }
    }
    return s32_ClostIdx;
}

// ==================================== FindNear ==================================== //
int32 FindTarget(Planning &st_Planning, Point2D &st_Point2D, Lateral_Control &st_LCtrl, int32 s32_StartIdx, int32 s32_FindNearLen, int32 s32_PathType)
{
    int32 s32_NearIdx = FindNear(st_Planning, st_Point2D, s32_StartIdx, s32_FindNearLen, s32_PathType);

    Path *st_Path;
    if (s32_PathType == c_GLOBALPATH)
        st_Path = &st_Planning.st_GlobalPath;
    else if (s32_PathType == c_LOCALPATH)
        st_Path = &st_Planning.st_LocalPath;

    const int32 s32_Size = st_Path->s32_Size;
    const float32 f32_Ld = st_LCtrl.f32_LD;

    int32 s32_TargetIdx = s32_NearIdx;

    for (int32 index = s32_NearIdx; index < s32_Size; index++)
    {
        float32 f32_Dist = Normalize2D(st_Path->arf32_E[index], st_Path->arf32_N[index], st_Point2D.f32_X, st_Point2D.f32_Y);
        if (f32_Dist >= f32_Ld)
        {
            s32_TargetIdx = index;
            break;
        }
    }

    st_Planning.s32_WayIdx = s32_TargetIdx;
    return st_Planning.s32_WayIdx;
}

void GetTan(Path &path)
{
    int32 s32_Size = path.s32_Size;

    for (int32 i = 0; i < s32_Size - 1; ++i)
    {
        float32 f32_Dx = path.arf32_E[i + 1] - path.arf32_E[i];
        float32 f32_Dy = path.arf32_N[i + 1] - path.arf32_N[i];
        float32 f32_Dist = Normalize2D(f32_Dx, f32_Dy);

        float32 f32_Tx = f32_Dx / f32_Dist;
        float32 f32_Ty = f32_Dy / f32_Dist;

        float32 f32_Angle_T = Rad2Deg(atan2f(f32_Ty, f32_Tx));
        float32 f32_Angle_N = Rad2Deg(atan2f(-f32_Tx, f32_Ty));
        float32 f32_Angle = Rad2Deg(atan2f(f32_Ty, f32_Tx));

        path.arf32_Tanv[i]  = f32_Angle_T;
        path.arf32_Norv[i]  = f32_Angle_N;
        path.arf32_Angle[i] = f32_Angle;
    }

    path.arf32_Tanv[s32_Size - 1] = path.arf32_Tanv[s32_Size - 2];
    path.arf32_Norv[s32_Size - 1] = path.arf32_Norv[s32_Size - 2];
    path.arf32_Angle[s32_Size - 1] = path.arf32_Angle[s32_Size - 2];
}

void Get_S_Ref(Path &path)
{
    int32 n = path.s32_Size;

    for (int32 i = 1; i < n; ++i)
    {
        double dx = static_cast<double>(path.arf32_E[i]) - static_cast<double>(path.arf32_E[i - 1]);
        double dy = static_cast<double>(path.arf32_N[i]) - static_cast<double>(path.arf32_N[i - 1]);
        path.s_ref[i] = path.s_ref[i - 1] + sqrt(dx * dx + dy * dy);
    }
}

//경로 기준 왼쪽 : 음수 d. 오른쪽 : 양수 d
// Point2D ENU2Frenet(Planning& st_Planning, float32 f32_E, float32 f32_N, string PathType)
// {
//     int idx = FindClostIdx(st_Planning, f32_E, f32_N);

//     Path st_TmpPath = st_Planning.s32_PathType == c_LOCALPATH ? st_Planning.st_LocalPath : st_Planning.st_GlobalPath;
//     if(PathType == "GlobalPath") 
//     {
//         st_TmpPath = st_Planning.st_GlobalPath;
//         idx = FindClostIdx(st_Planning,f32_E,f32_N,c_GLOBALPATH);
//     }


//     // s 값 계산 (누적 거리)
//     float32 s = 0.0;
//     for (int i = 1; i <= idx; i++)
//     {
//         s += Normalize2D(st_TmpPath.arf32_E, st_TmpPath.arf32_N, i - 1, i);
//     }

//     float32 f32_deg = st_TmpPath.arf32_Norv[idx];
//     float32 f32_rad = Deg2Rad(f32_deg);
//     float32 f32_nx = cosf(f32_rad);
//     float32 f32_ny = sinf(f32_rad);

//     // 방향성을 고려한 d 계산
//     float32 f32_de = f32_E - st_TmpPath.arf32_E[idx];
//     float32 f32_dn = f32_N - st_TmpPath.arf32_N[idx];
   
//     // 법선 벡터와 내적을 이용해 부호 결정
//     float32 d = (f32_de * f32_nx + f32_dn * f32_ny);

//     Point2D st_Frenet = {s,d};

//     return st_Frenet;
// }

static inline float32 clamp01(float32 x){ return x<0.f?0.f:(x>1.f?1.f:x); }

// i번째 점까지의 누적거리 S[i]를 반환. 미보유 시 빠르게 계산(필요시 캐싱 권장)
static float32 accumulateS(const Path& path, int i)
{
    float32 s = 0.f;
    for(int k=1; k<=i; ++k){
        const float32 dx = path.arf32_E[k]-path.arf32_E[k-1];
        const float32 dy = path.arf32_N[k]-path.arf32_N[k-1];
        s += std::sqrt(dx*dx + dy*dy);
    }
    return s;
}

Point2D ENU2Frenet(Planning& st_Planning, float32 f32_E, float32 f32_N, std::string PathType)
{
    // 1) 사용할 레퍼런스 경로 선택
    const Path& path = (PathType == "GlobalPath")
        ? st_Planning.st_GlobalPath
        : (st_Planning.s32_PathType == c_LOCALPATH ? st_Planning.st_LocalPath : st_Planning.st_GlobalPath);

    const int N = path.s32_Size; // 점 개수 (없다면 arf32_E/N의 길이 사용)
    if (N < 2) return {0.f, 0.f};

    // 2) 가장 가까운 '점' idx를 찾고, 그 주변 두 세그먼트 중 더 잘 투영되는 세그먼트를 선택
    int idxPt = (PathType == "GlobalPath")
              ? FindClostIdx(st_Planning, f32_E, f32_N, c_GLOBALPATH)
              : FindClostIdx(st_Planning, f32_E, f32_N);

    // 후보 세그먼트: [idxPt-1, idxPt], [idxPt, idxPt+1]
    int i0 = std::max(0, idxPt - 1);
    int i1 = std::min(N - 2, idxPt); // 세그먼트의 시작 인덱스(끝은 i+1)

    auto project_on_seg = [&](int i, float32& t_out, float32& d_signed_out)->float32 {
        // 세그 i: Pi -> Pj
        const float32 x1 = path.arf32_E[i],     y1 = path.arf32_N[i];
        const float32 x2 = path.arf32_E[i+1],   y2 = path.arf32_N[i+1];
        const float32 vx = x2 - x1,             vy = y2 - y1;
        const float32 seg_len2 = vx*vx + vy*vy;
        if (seg_len2 < 1e-6f) { t_out = 0.f; d_signed_out = 0.f; return 1e9f; } // 퇴화 세그

        // t*: (P1->P 점) · (P1->P2) / |seg|^2
        const float32 wx = f32_E - x1, wy = f32_N - y1;
        float32 t = (wx*vx + wy*vy) / seg_len2;
        t = clamp01(t); // 세그먼트 내부로 클램프
        t_out = t;

        // 단위 접선벡터
        const float32 seg_len = std::sqrt(seg_len2);
        const float32 tx = vx / seg_len, ty = vy / seg_len;

        // 투영점
        const float32 px = x1 + t*vx, py = y1 + t*vy;

        // 부호 포함 수직거리: cross(접선, P1->P)
        // cross = tx*wy - ty*wx. 진행 기준 좌측(수학적 +)이 양수
        float32 cross = tx*wy - ty*wx;
        // 요구사항: "왼쪽 음수, 오른쪽 양수" → 부호 뒤집기
        d_signed_out = -cross;

        // 실제 거리(절댓값) 반환(선택 기준)
        const float32 dxp = f32_E - px, dyp = f32_N - py;
        return std::sqrt(dxp*dxp + dyp*dyp);
    };

    float32 t0, d0, t1, d1;
    float32 dist0 = project_on_seg(i0, t0, d0);
    float32 dist1 = project_on_seg(i1, t1, d1);

    int i_best; float32 t_best, d_best;
    if (dist0 <= dist1) { i_best = i0; t_best = t0; d_best = d0; }
    else                { i_best = i1; t_best = t1; d_best = d1; }

    // 3) s 계산: S[i_best] + t_best * |seg|
    float32 s_prefix;
    if constexpr (/* 경로에 누적거리 테이블이 있다면 조건 바꿔줘 */ false) {
        // 예: path.arf32_S[i]
        // s_prefix = path.arf32_S[i_best];
        s_prefix = accumulateS(path, i_best); // 위 줄처럼 테이블 있으면 이 한줄로 교체
    } else {
        s_prefix = accumulateS(path, i_best);  // 테이블 없으면 누적(성능 필요 시 캐싱 권장)
    }
    const float32 vx = path.arf32_E[i_best+1] - path.arf32_E[i_best];
    const float32 vy = path.arf32_N[i_best+1] - path.arf32_N[i_best];
    const float32 seg_len = std::sqrt(vx*vx + vy*vy);

    const float32 s = s_prefix + t_best * seg_len;
    const float32 d = d_best; // 이미 좌/우 부호 규칙 반영

    return {s, d};
}

Point2D Frenet2ENU(Planning &st_Planning, float32 f32_s, float32 f32_d)
{
    // s에 가장 가까운 점 찾기
    float32 accumulated_s = 0.0;
    int idx = 0;

    Path& st_TmpPath = st_Planning.s32_PathType == c_LOCALPATH ? st_Planning.st_LocalPath : st_Planning.st_GlobalPath;
    for (size_t i = 1; i <= st_TmpPath.s32_Size; i++)
    {
        accumulated_s += Normalize2D(st_TmpPath.arf32_E, st_TmpPath.arf32_N, i - 1, i);
        if (accumulated_s >= f32_s)
        {
            idx = i;
            break;
        }
    }

    float32 f32_deg = st_TmpPath.arf32_Norv[idx - 1];
    float32 f32_rad = Deg2Rad(f32_deg);
    float32 f32_nx = cosf(f32_rad);
    float32 f32_ny = sinf(f32_rad);

    // 새로운 ENU 좌표 계산 (d만큼 이동)
    Point2D st_EN;
    st_EN.f32_X = st_TmpPath.arf32_E[idx] + f32_d * f32_nx;
    st_EN.f32_Y = st_TmpPath.arf32_N[idx] + f32_d * f32_ny;

    return st_EN;
}

void Body2ENU(VehicleSpec& st_VehicleSpec, const GPS_IMU& st_GPSIMU, Object& st_Object)
{
    float32 f32_Xb = st_Object.f32_CoreX + st_VehicleSpec.st_LiDARPos.f32_X - st_VehicleSpec.st_GPSPos.f32_X;
    float32 f32_Yb = st_Object.f32_CoreY + st_VehicleSpec.st_LiDARPos.f32_Y - st_VehicleSpec.st_GPSPos.f32_Y;
    float32 f32_Zb = st_Object.f32_CoreZ + st_VehicleSpec.st_LiDARPos.f32_Z - st_VehicleSpec.st_GPSPos.f32_Z;
    float32 f32_Roll  = Deg2Rad(st_GPSIMU.st_IMU.f32_Roll_deg);
    float32 f32_Pitch = Deg2Rad(st_GPSIMU.st_IMU.f32_Pitch_deg);
    float32 f32_Yaw   = Deg2Rad(st_GPSIMU.st_IMU.f32_Yaw_deg);

    float32 f32_C_Roll = cos(f32_Roll);  
    float32 f32_S_Roll = sin(f32_Roll);
    
    float32 f32_C_Pitch = cos(f32_Pitch); 
    float32 f32_S_Pitch = sin(f32_Pitch);
    
    float32 f32_C_Yaw = cos(f32_Yaw);   
    float32 f32_S_Yaw = sin(f32_Yaw);

    float32 f32_R11 = f32_C_Yaw * f32_C_Pitch;
    float32 f32_R12 = f32_C_Yaw * f32_S_Pitch * f32_S_Roll - f32_S_Yaw * f32_C_Roll;
    float32 f32_R13 = f32_C_Yaw * f32_S_Pitch * f32_C_Roll + f32_S_Yaw * f32_S_Roll;

    float32 f32_R21 = f32_S_Yaw * f32_C_Pitch;
    float32 f32_R22 = f32_S_Yaw * f32_S_Pitch * f32_S_Roll + f32_C_Yaw * f32_C_Roll;
    float32 f32_R23 = f32_S_Yaw * f32_S_Pitch * f32_C_Roll - f32_C_Yaw * f32_S_Roll;

    float32 f32_R31 = -f32_S_Pitch;
    float32 f32_R32 = f32_C_Pitch * f32_S_Roll;
    float32 f32_R33 = f32_C_Pitch * f32_C_Roll;

    float32 f32_E = f32_R11 * f32_Xb + f32_R12 * f32_Yb + f32_R13 * f32_Zb + st_GPSIMU.st_GPS.f64_E;
    float32 f32_N = f32_R21 * f32_Xb + f32_R22 * f32_Yb + f32_R23 * f32_Zb + st_GPSIMU.st_GPS.f64_N;
    float32 f32_U = f32_R31 * f32_Xb + f32_R32 * f32_Yb + f32_R33 * f32_Zb + st_GPSIMU.st_GPS.f64_U;

    // float32 f32_E =  st_Object.f32_CoreX ;
    // float32 f32_N =  st_Object.f32_CoreY;
    // // float32 f32_U = f32_R31 * f32_Xb + f32_R32 * f32_Yb + f32_R33 * f32_Zb + st_GPSIMU.st_GPS.f32_U;
    st_Object.f32_CoreE= f32_E;
    st_Object.f32_CoreN= f32_N;
    // st_Object.f32_CoreU = f32_U;
}

Point2D Body2ENU_Point(const VehicleSpec& st_VehicleSpec, const GPS_IMU& st_GPSIMU, float32 x, float32 y)
{
    // Body 좌표계의 점 (Xb, Yb)
    float32 f32_Xb = x + st_VehicleSpec.st_LiDARPos.f32_X - st_VehicleSpec.st_GPSPos.f32_X;
    float32 f32_Yb = y + st_VehicleSpec.st_LiDARPos.f32_Y - st_VehicleSpec.st_GPSPos.f32_Y;
    
    // GPS IMU의 Roll, Pitch, Yaw
    float32 f32_Roll  = Deg2Rad(st_GPSIMU.st_IMU.f32_Roll_deg);
    float32 f32_Pitch = Deg2Rad(st_GPSIMU.st_IMU.f32_Pitch_deg);
    float32 f32_Yaw   = Deg2Rad(st_GPSIMU.st_IMU.f32_Yaw_deg);

    // 회전 행렬 계산을 위한 cos, sin 값들
    float32 f32_C_Roll = cos(f32_Roll);  
    float32 f32_S_Roll = sin(f32_Roll);
    
    float32 f32_C_Pitch = cos(f32_Pitch); 
    float32 f32_S_Pitch = sin(f32_Pitch);
    
    float32 f32_C_Yaw = cos(f32_Yaw);   
    float32 f32_S_Yaw = sin(f32_Yaw);

    // 회전 행렬
    float32 f32_R11 = f32_C_Yaw * f32_C_Pitch;
    float32 f32_R12 = f32_C_Yaw * f32_S_Pitch * f32_S_Roll - f32_S_Yaw * f32_C_Roll;
    float32 f32_R13 = f32_C_Yaw * f32_S_Pitch * f32_C_Roll + f32_S_Yaw * f32_S_Roll;

    float32 f32_R21 = f32_S_Yaw * f32_C_Pitch;
    float32 f32_R22 = f32_S_Yaw * f32_S_Pitch * f32_S_Roll + f32_C_Yaw * f32_C_Roll;
    float32 f32_R23 = f32_S_Yaw * f32_S_Pitch * f32_C_Roll - f32_C_Yaw * f32_S_Roll;

    float32 f32_R31 = -f32_S_Pitch;
    float32 f32_R32 = f32_C_Pitch * f32_S_Roll;
    float32 f32_R33 = f32_C_Pitch * f32_C_Roll;

    // ENU 좌표계로 변환
    float32 f32_E = f32_R11 * f32_Xb + f32_R12 * f32_Yb + st_GPSIMU.st_GPS.f64_E;
    float32 f32_N = f32_R21 * f32_Xb + f32_R22 * f32_Yb + st_GPSIMU.st_GPS.f64_N;

    // ENU 좌표를 반환 (z는 고려하지 않음)
    Point2D enu_point;
    enu_point.f32_X = f32_E;
    enu_point.f32_Y = f32_N;

    return enu_point;
}

void MinMax2ENU(VehicleSpec& st_VehicleSpec, const GPS_IMU& st_GPSIMU, Object& st_Object)
{
    float32 f32_MINX = st_Object.f32_MinX + st_VehicleSpec.st_LiDARPos.f32_X - st_VehicleSpec.st_GPSPos.f32_X;
    float32 f32_MINY = st_Object.f32_MinY + st_VehicleSpec.st_LiDARPos.f32_Y - st_VehicleSpec.st_GPSPos.f32_Y;
    float32 f32_MINZ = st_Object.f32_MinZ + st_VehicleSpec.st_LiDARPos.f32_Z - st_VehicleSpec.st_GPSPos.f32_Z;
    float32 f32_MAXX = st_Object.f32_MaxX + st_VehicleSpec.st_LiDARPos.f32_X - st_VehicleSpec.st_GPSPos.f32_X;
    float32 f32_MAXY = st_Object.f32_MaxY + st_VehicleSpec.st_LiDARPos.f32_Y - st_VehicleSpec.st_GPSPos.f32_Y;
    float32 f32_MAXZ = st_Object.f32_MaxZ + st_VehicleSpec.st_LiDARPos.f32_Z - st_VehicleSpec.st_GPSPos.f32_Z;
    
    float32 f32_Roll  = Deg2Rad(st_GPSIMU.st_IMU.f32_Roll_deg);
    float32 f32_Pitch = Deg2Rad(st_GPSIMU.st_IMU.f32_Pitch_deg);
    float32 f32_Yaw   = Deg2Rad(st_GPSIMU.st_IMU.f32_Yaw_deg);

    float32 f32_C_Roll = cos(f32_Roll);  
    float32 f32_S_Roll = sin(f32_Roll);
    
    float32 f32_C_Pitch = cos(f32_Pitch); 
    float32 f32_S_Pitch = sin(f32_Pitch);
    
    float32 f32_C_Yaw = cos(f32_Yaw);   
    float32 f32_S_Yaw = sin(f32_Yaw);

    float32 f32_R11 = f32_C_Yaw * f32_C_Pitch;
    float32 f32_R12 = f32_C_Yaw * f32_S_Pitch * f32_S_Roll - f32_S_Yaw * f32_C_Roll;
    float32 f32_R13 = f32_C_Yaw * f32_S_Pitch * f32_C_Roll + f32_S_Yaw * f32_S_Roll;

    float32 f32_R21 = f32_S_Yaw * f32_C_Pitch;
    float32 f32_R22 = f32_S_Yaw * f32_S_Pitch * f32_S_Roll + f32_C_Yaw * f32_C_Roll;
    float32 f32_R23 = f32_S_Yaw * f32_S_Pitch * f32_C_Roll - f32_C_Yaw * f32_S_Roll;

    float32 f32_R31 = -f32_S_Pitch;
    float32 f32_R32 = f32_C_Pitch * f32_S_Roll;
    float32 f32_R33 = f32_C_Pitch * f32_C_Roll;

    float32 f32_MinE = f32_R11 * f32_MINX + f32_R12 * f32_MINY + f32_R13 * f32_MINZ + st_GPSIMU.st_GPS.f64_E;
    float32 f32_MinN = f32_R21 * f32_MINX + f32_R22 * f32_MINY + f32_R23 * f32_MINZ + st_GPSIMU.st_GPS.f64_N;
    float32 f32_MinU = f32_R31 * f32_MINX + f32_R32 * f32_MINY + f32_R33 * f32_MINZ + st_GPSIMU.st_GPS.f64_U;

    float32 f32_MaxE = f32_R11 * f32_MAXX + f32_R12 * f32_MAXY + f32_R13 * f32_MAXZ + st_GPSIMU.st_GPS.f64_E;
    float32 f32_MaxN = f32_R21 * f32_MAXX + f32_R22 * f32_MAXY + f32_R23 * f32_MAXZ + st_GPSIMU.st_GPS.f64_N;
    float32 f32_MaxU = f32_R31 * f32_MAXX + f32_R32 * f32_MAXY + f32_R33 * f32_MAXZ + st_GPSIMU.st_GPS.f64_U;

    // float32 f32_E =  st_Object.f32_CoreX ;
    // float32 f32_N =  st_Object.f32_CoreY;
    // // float32 f32_U = f32_R31 * f32_MX + f32_R32 * f32_MY + f32_R33 * f32_MZ + st_GPSIMU.st_GPS.f32_U;


    st_Object.f32_MinE_X= f32_MinE;
    st_Object.f32_MinE_Y= f32_MinN;
    st_Object.f32_MinE_Z = f32_MinU;
    st_Object.f32_MaxE_X= f32_MaxE;
    st_Object.f32_MaxE_Y= f32_MaxN;
    st_Object.f32_MaxE_Z = f32_MaxU;

}

uint64_t GetMillisecond()
{
    auto now = std::chrono::steady_clock::now();
    auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return static_cast<uint64_t>(ms_since_epoch.count());
}

void CalcVertex(float32 f32_X_global, float32 f32_Y_global, float32 f32_MaxX, float32 f32_MaxY, float32 f32_MinX, float32 f32_MinY, float32 f32_heading, float32 (&f32_Vertex)[4][2], int32 s32_Mode)
{
    float32 headingRad = f32_heading;
    float32 cosHeading = cos(headingRad);
    float32 sinHeading = sin(headingRad);

    if(s32_Mode == -1) // Object
    {
        f32_MaxX = fmaxf(f32_MaxX , 3.f);
        f32_MaxY = fmaxf(f32_MaxY , 0.9f);
        f32_MinX = fminf(f32_MinX , 0.f);
        f32_MinY = fminf(f32_MinY , -0.9f);

        f32_Vertex[0][0] = f32_X_global + (cosHeading * f32_MaxX - sinHeading * f32_MaxY);
        f32_Vertex[0][1] = f32_Y_global + (sinHeading * f32_MaxX + cosHeading * f32_MaxY);

        f32_Vertex[1][0] = f32_X_global + (cosHeading * f32_MaxX + sinHeading * f32_MaxY);
        f32_Vertex[1][1] = f32_Y_global + (sinHeading * f32_MaxX - cosHeading * f32_MaxY);

        f32_Vertex[2][0] = f32_X_global + (cosHeading * f32_MinX + sinHeading * f32_MaxY);
        f32_Vertex[2][1] = f32_Y_global + (sinHeading * f32_MinX - cosHeading * f32_MaxY);

        f32_Vertex[3][0] = f32_X_global + (cosHeading * f32_MinX - sinHeading * f32_MaxY);
        f32_Vertex[3][1] = f32_Y_global + (sinHeading * f32_MinX + cosHeading * f32_MaxY);
    }
    else if(s32_Mode == 0) // Ego
{
    // === GPS → 차량 중심 보정 ===
    float32 offset = -1.5f;  // 뒤쪽으로 이동
    // float32 X_center = f32_X_global + cosHeading * offset;
    // float32 Y_center = f32_Y_global + sinHeading * offset;
    float32 X_center = f32_X_global;
    float32 Y_center = f32_Y_global;

    // === Ego OBB 계산 ===
    f32_Vertex[0][0] = X_center + (cosHeading * f32_MaxX - sinHeading * f32_MaxY);
    f32_Vertex[0][1] = Y_center + (sinHeading * f32_MaxX + cosHeading * f32_MaxY);

    f32_Vertex[1][0] = X_center + (cosHeading * f32_MaxX + sinHeading * f32_MaxY);
    f32_Vertex[1][1] = Y_center + (sinHeading * f32_MaxX - cosHeading * f32_MaxY);

    f32_Vertex[2][0] = X_center + (cosHeading * f32_MinX + sinHeading * f32_MaxY);
    f32_Vertex[2][1] = Y_center + (sinHeading * f32_MinX - cosHeading * f32_MaxY);

    f32_Vertex[3][0] = X_center + (cosHeading * f32_MinX - sinHeading * f32_MaxY);
    f32_Vertex[3][1] = Y_center + (sinHeading * f32_MinX + cosHeading * f32_MaxY);
}

}

Point2D CalcTangentVector(Point2D p1, Point2D p2)
{
    Point2D edge = {p2.f32_X - p1.f32_X, p2.f32_Y - p1.f32_Y};
    return edge;
}

Point2D CalcNormalVector(Point2D edge)
{
    Point2D normal = {-edge.f32_Y, edge.f32_X};
    return normal;
}

float32 CalcDotProduct(Point2D v1, Point2D v2)
{
    return v1.f32_X * v2.f32_X + v1.f32_Y * v2.f32_Y;
}

void PolygonProjection(Point2D axis, Point2D vertices[], int numVertices, float32 *min, float32 *max)
{
    *min = CalcDotProduct(axis, vertices[0]);
    *max = *min;
    for (int i = 1; i < numVertices; i++) {
        float32 projection = CalcDotProduct(axis, vertices[i]);
        if (projection < *min) {
            *min = projection;
        }
        if (projection > *max) {
            *max = projection;
        }
    }
}

bool OverlappingTest(float32 minA, float32 maxA, float32 minB, float32 maxB)
{
    return !(maxA < minB || maxB < minA);
}

bool CalcSAT(float32 vertices1[4][2], float32 vertices2[4][2])
{
    Point2D poly1[4], poly2[4];
    for (int i = 0; i < 4; i++) {
        poly1[i].f32_X = vertices1[i][0];
        poly1[i].f32_Y = vertices1[i][1];
        poly2[i].f32_X = vertices2[i][0];
        poly2[i].f32_Y = vertices2[i][1];
    }

    for (int i = 0; i < 4; i++) {
        Point2D edge1 = CalcTangentVector(poly1[i], poly1[(i + 1) % 4]);
        Point2D axis1 = CalcNormalVector(edge1);

        float32 minA, maxA, minB, maxB;
        PolygonProjection(axis1, poly1, 4, &minA, &maxA);
        PolygonProjection(axis1, poly2, 4, &minB, &maxB);

        if (!OverlappingTest(minA, maxA, minB, maxB)) {
            return false;
        }
    }

    for (int i = 0; i < 4; i++) {
        Point2D edge2 = CalcTangentVector(poly2[i], poly2[(i + 1) % 4]);
        Point2D axis2 = CalcNormalVector(edge2);

        float32 minA, maxA, minB, maxB;
        PolygonProjection(axis2, poly1, 4, &minA, &maxA);
        PolygonProjection(axis2, poly2, 4, &minB, &maxB);

        if (!OverlappingTest(minA, maxA, minB, maxB)) {
            return false;
        }
    }

    return true;
}

// ==================================== Lidar ==================================== //

LiDARTracking st_Tracking = {
    0,        // s32_new_track_id (처음엔 0)
    1.0f,     // f32_COST_W_DIST
    1.0f,     // f32_COST_W_IOU
    0.5f,     // f32_COST_W_DIR 얘는 사실상 안쓰는 중(헤딩에 대한 가중치)
    3.0f,      // f32_COST_THRESHOLD (10 m was permissive enough to swap nearby tracks)
    9999.0f   // f32_COST_INF
};

ros::Time ts_RawTime(0,0);
ros::Time ts_PrevTime(0,0);
ros::Time ts_CurrTime(0,0);
