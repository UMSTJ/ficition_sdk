#include <string>

// 将一组十六进制数组合成一个int32类型
enum ControlStatus
{
    EMERGENCY_STOP,  // 急停状态
    PROGRAM_CONTROL, // 程序控制状态
    REMOTE_CONTROL   // 遥控控制状态
};

enum class SysStatus : int
{
    SYS_STANDBY,
    SYS_RUNNING_URT,
    SYS_RUNNING_JOY,
    SYS_EMG_PSB,
    SYS_EMG_APT
};

struct ParamsData
{
    float KP;
    float KI;
    float KD;
    float LA;
    float LB;
    float MPE;
    float MPC;
    int32_t KMTT;
    float IMU_ZOFS;
    float IMU_YOFS;
    float IMU_XOFS;
    int32_t MTDIR; //电机编码器方向配置
    int32_t MSNUM; //驱动器速度和角度输出配置
    int32_t ROSID; //microROS ID配置
    int32_t UPLHZ; // UART上传频率配置
    int32_t FUNC1; //功能组件1 
    int32_t FUNC2; //功能组件2
    int32_t FUNC3; //功能组件3
    bool sysStatusFrame;     // 状态帧
    SysStatus sysStatusData; // 系统运作状态

     /**
     * @brief 重载等于操作符 (==)
     * @param other 另一个要比较的 ParamsData 对象
     * @return 如果所有成员都相等，则返回 true
     */
    bool operator==(const ParamsData& other) const
    {
        // 将所有成员逐一比较。
        // 对于 enum class，可以直接使用 == 进行比较。
        return KP == other.KP &&
               KI == other.KI &&
               KD == other.KD &&
               LA == other.LA &&
               LB == other.LB &&
               MPE == other.MPE &&
               MPC == other.MPC &&
               KMTT == other.KMTT &&
               IMU_ZOFS == other.IMU_ZOFS &&
               IMU_YOFS == other.IMU_YOFS &&
               IMU_XOFS == other.IMU_XOFS &&
               MTDIR == other.MTDIR &&
               MSNUM == other.MSNUM &&
               ROSID == other.ROSID &&
               UPLHZ == other.UPLHZ &&
               FUNC1 == other.FUNC1 &&
               FUNC2 == other.FUNC2 &&
               FUNC3 == other.FUNC3 &&
               sysStatusFrame == other.sysStatusFrame &&
               sysStatusData == other.sysStatusData; // enum class可以直接比较
    }

    /**
     * @brief 重载不等于操作符 (!=)
     * @param other 另一个要比较的 ParamsData 对象
     * @return 如果它们不相等，则返回 true
     */
    bool operator!=(const ParamsData& other) const
    {
        return !(*this == other); // 复用 operator== 的结果
    }
};

struct OdomInfo
{
    /* data */
    double vx;
    double vy;
    double vth;
};

struct PowerInfo
{
    double bus;
    double output5v;
    double input;
    double output19v;
};

struct ImuInfo
{
    double axaxis;
    double ayaxis;
    double azaxis;
    double gxaxis;
    double gyaxis;
    double gzaxis;
    double q0;
    double q1;
    double q2;
    double q3;
    double roll;
    double yaw;
    double pitch;
};
struct ICDRemote
{
    double az;
    double vx;
    double vy;
};

struct RCSBUSRemote
{
    int len;
    uint16_t axes[8];
    uint16_t buttons[8];
};

enum class AgreementVersion : int
{
    V1,
    V2
};

struct FictionData
{
    ImuInfo imuStructural;
    OdomInfo odomData;
    ParamsData paramsData;
    std::string magneticData;
    std::string rfidData;
    PowerInfo powerData;
    float ultrasonic;
    float temperature;
    ICDRemote icdData;
    RCSBUSRemote rcsBusData;
};

struct TwistCustom
{
    float linear_x;
    float linear_y;
    float angular_z;
};
