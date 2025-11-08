/**
 * =====================================================================
 * 互补滤波算法 - C语言实现
 * =====================================================================
 *
 * 功能：融合陀螺仪和加速度计数据，估计姿态角度
 * 适用：无人机、平衡车、云台等姿态控制系统
 * 特点：计算简单、实时性好、资源占用低
 *
 * 作者：嵌入式系统开发示例
 * 日期：2025
 * =====================================================================
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>

// =====================================================================
// 常量定义
// =====================================================================

#define PI 3.14159265358979323846
#define RAD_TO_DEG (180.0 / PI)  // 弧度转角度
#define DEG_TO_RAD (PI / 180.0)  // 角度转弧度

// =====================================================================
// 互补滤波器结构体
// =====================================================================

/**
 * @brief 互补滤波器数据结构
 */
typedef struct {
    float roll;          // 横滚角（绕X轴旋转），单位：度
    float pitch;         // 俯仰角（绕Y轴旋转），单位：度
    float yaw;           // 航向角（绕Z轴旋转），单位：度
    float alpha;         // 互补滤波系数（0~1），推荐0.95~0.98
    uint32_t last_time;  // 上次更新时间（毫秒）
} ComplementaryFilter_t;

// =====================================================================
// 辅助函数
// =====================================================================

/**
 * @brief 限制角度范围在 -180° ~ +180°
 *
 * @param angle 输入角度
 * @return 归一化后的角度
 */
float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 安全的反正切函数（避免除零）
 *
 * @param y 分子
 * @param x 分母
 * @return 角度（度）
 */
float safe_atan2(float y, float x) {
    return atan2f(y, x) * RAD_TO_DEG;
}

// =====================================================================
// 互补滤波器核心函数
// =====================================================================

/**
 * @brief 初始化互补滤波器
 *
 * @param filter 滤波器结构体指针
 * @param alpha 互补滤波系数（推荐0.96）
 * @param current_time 当前时间（毫秒）
 *
 * 说明：
 * - alpha接近1：更相信陀螺仪（响应快，但会漂移）
 * - alpha接近0：更相信加速度计（稳定，但易受干扰）
 */
void complementary_filter_init(ComplementaryFilter_t *filter,
                               float alpha,
                               uint32_t current_time) {
    filter->roll = 0.0f;
    filter->pitch = 0.0f;
    filter->yaw = 0.0f;
    filter->alpha = alpha;
    filter->last_time = current_time;
}

/**
 * @brief 从加速度计数据计算俯仰角和横滚角
 *
 * @param ax 加速度计X轴原始数据（m/s²或g）
 * @param ay 加速度计Y轴原始数据
 * @param az 加速度计Z轴原始数据
 * @param roll_out 输出横滚角（度）
 * @param pitch_out 输出俯仰角（度）
 *
 * 原理：
 * 静止时，加速度计只测量重力加速度
 * 重力在各轴的分量可以计算出倾斜角度
 *
 * 注意：
 * - 运动时会受到运动加速度干扰
 * - 只能测量roll和pitch，无法测量yaw
 */
void calculate_accel_angles(float ax, float ay, float az,
                           float *roll_out, float *pitch_out) {
    // 横滚角：绕X轴旋转
    // atan2(ay, az) 计算Y轴和Z轴加速度的比值
    *roll_out = safe_atan2(ay, az);

    // 俯仰角：绕Y轴旋转
    // atan2(-ax, sqrt(ay^2 + az^2)) 考虑所有轴的加速度
    *pitch_out = safe_atan2(-ax, sqrtf(ay * ay + az * az));
}

/**
 * @brief 互补滤波器更新（核心算法）
 *
 * @param filter 滤波器结构体指针
 * @param gx 陀螺仪X轴角速度（度/秒）
 * @param gy 陀螺仪Y轴角速度（度/秒）
 * @param gz 陀螺仪Z轴角速度（度/秒）
 * @param ax 加速度计X轴数据（m/s²或g）
 * @param ay 加速度计Y轴数据
 * @param az 加速度计Z轴数据
 * @param current_time 当前时间（毫秒）
 *
 * 核心公式：
 * angle = α × (angle + gyro × dt) + (1-α) × accel
 *
 * 说明：
 * 1. 高通部分：α × (angle + gyro × dt) - 陀螺仪积分
 * 2. 低通部分：(1-α) × accel - 加速度计角度
 * 3. 互补融合：两部分相加得到最优估计
 */
void complementary_filter_update(ComplementaryFilter_t *filter,
                                 float gx, float gy, float gz,
                                 float ax, float ay, float az,
                                 uint32_t current_time) {
    // 1. 计算时间间隔（秒）
    float dt = (current_time - filter->last_time) / 1000.0f;
    filter->last_time = current_time;

    // 防止异常：时间间隔过大或过小
    if (dt > 1.0f || dt <= 0.0f) {
        dt = 0.01f;  // 默认100Hz
    }

    // 2. 陀螺仪积分：计算角度变化
    // dt很小时，可以用一阶积分近似
    float gyro_roll = filter->roll + gx * dt;   // 角度 += 角速度 × 时间
    float gyro_pitch = filter->pitch + gy * dt;
    float gyro_yaw = filter->yaw + gz * dt;

    // 3. 加速度计计算角度
    float accel_roll, accel_pitch;
    calculate_accel_angles(ax, ay, az, &accel_roll, &accel_pitch);

    // 4. 互补滤波融合
    // roll和pitch：结合陀螺仪和加速度计
    filter->roll = filter->alpha * gyro_roll + (1.0f - filter->alpha) * accel_roll;
    filter->pitch = filter->alpha * gyro_pitch + (1.0f - filter->alpha) * accel_pitch;

    // yaw：只能用陀螺仪积分（加速度计无法测量yaw）
    filter->yaw = gyro_yaw;

    // 5. 角度归一化（防止累积超出范围）
    filter->roll = normalize_angle(filter->roll);
    filter->pitch = normalize_angle(filter->pitch);
    filter->yaw = normalize_angle(filter->yaw);
}

/**
 * @brief 获取当前姿态角度
 *
 * @param filter 滤波器结构体指针
 * @param roll 输出横滚角
 * @param pitch 输出俯仰角
 * @param yaw 输出航向角
 */
void complementary_filter_get_angles(ComplementaryFilter_t *filter,
                                    float *roll, float *pitch, float *yaw) {
    *roll = filter->roll;
    *pitch = filter->pitch;
    *yaw = filter->yaw;
}

// =====================================================================
// 高级功能：自适应互补滤波
// =====================================================================

/**
 * @brief 自适应互补滤波器（根据运动状态调整alpha）
 *
 * @param filter 滤波器结构体指针
 * @param gx, gy, gz 陀螺仪数据
 * @param ax, ay, az 加速度计数据
 * @param current_time 当前时间
 *
 * 策略：
 * - 快速运动时：增大alpha（相信陀螺仪）
 * - 静止时：减小alpha（相信加速度计）
 * - 中速运动：使用默认值
 */
void adaptive_complementary_filter_update(ComplementaryFilter_t *filter,
                                          float gx, float gy, float gz,
                                          float ax, float ay, float az,
                                          uint32_t current_time) {
    // 计算陀螺仪的总角速度（衡量运动剧烈程度）
    float gyro_magnitude = sqrtf(gx*gx + gy*gy + gz*gz);

    // 计算加速度幅值（检测是否有额外加速度）
    float accel_magnitude = sqrtf(ax*ax + ay*ay + az*az);
    float gravity = 9.81f;  // 标准重力加速度
    float accel_deviation = fabsf(accel_magnitude - gravity);

    // 保存原始alpha
    float original_alpha = filter->alpha;

    // 自适应调整策略
    if (gyro_magnitude > 100.0f) {
        // 快速旋转：增大alpha（陀螺仪更可靠）
        filter->alpha = 0.98f;
    } else if (gyro_magnitude < 5.0f && accel_deviation < 1.0f) {
        // 静止或慢速且无额外加速度：减小alpha（加速度计更可靠）
        filter->alpha = 0.92f;
    } else if (accel_deviation > 3.0f) {
        // 有明显加速度干扰：增大alpha（忽略加速度计）
        filter->alpha = 0.97f;
    } else {
        // 正常运动：使用默认值
        filter->alpha = 0.96f;
    }

    // 调用标准更新函数
    complementary_filter_update(filter, gx, gy, gz, ax, ay, az, current_time);

    // 恢复原始alpha（避免影响下次调用）
    filter->alpha = original_alpha;
}

// =====================================================================
// 示例代码：模拟姿态估计
// =====================================================================

/**
 * @brief 模拟获取系统时间（实际使用时需要替换）
 */
uint32_t get_system_time_ms(void) {
    // 实际嵌入式系统中使用：
    // - STM32: HAL_GetTick()
    // - Arduino: millis()
    // - FreeRTOS: xTaskGetTickCount()
    static uint32_t time = 0;
    time += 10;  // 模拟10ms间隔
    return time;
}

/**
 * @brief 模拟读取IMU传感器数据（实际使用时需要替换）
 */
void read_imu_sensor(float *gx, float *gy, float *gz,
                    float *ax, float *ay, float *az) {
    // 实际嵌入式系统中调用：
    // - MPU6050_ReadData()
    // - ICM20602_GetData()
    // - BMI088_ReadAccelGyro()

    // 模拟数据：设备向前倾斜10度
    *gx = 0.0f;    // 陀螺仪X轴：0度/秒（静止）
    *gy = 0.0f;    // 陀螺仪Y轴：0度/秒
    *gz = 0.0f;    // 陀螺仪Z轴：0度/秒

    *ax = 1.7f;    // 加速度计X轴：倾斜时X轴有分量
    *ay = 0.0f;    // 加速度计Y轴：0（无横滚）
    *az = 9.65f;   // 加速度计Z轴：接近重力加速度
}

/**
 * @brief 主程序示例
 */
int main(void) {
    printf("==========================================\n");
    printf("互补滤波算法示例 - 姿态估计\n");
    printf("==========================================\n\n");

    // 1. 初始化互补滤波器
    ComplementaryFilter_t filter;
    float alpha = 0.96f;  // 互补滤波系数
    uint32_t current_time = get_system_time_ms();
    complementary_filter_init(&filter, alpha, current_time);

    printf("滤波器初始化完成\n");
    printf("Alpha系数: %.2f (陀螺仪权重: %.0f%%, 加速度计权重: %.0f%%)\n\n",
           alpha, alpha * 100, (1 - alpha) * 100);

    // 2. 模拟传感器数据读取和滤波（循环100次）
    printf("开始姿态估计...\n");
    printf("-------------------------------------------\n");
    printf("次数 | Roll(度) | Pitch(度) | Yaw(度)\n");
    printf("-------------------------------------------\n");

    for (int i = 0; i < 100; i++) {
        // 读取传感器数据
        float gx, gy, gz;  // 陀螺仪
        float ax, ay, az;  // 加速度计
        read_imu_sensor(&gx, &gy, &gz, &ax, &ay, &az);

        // 更新滤波器
        current_time = get_system_time_ms();
        complementary_filter_update(&filter, gx, gy, gz, ax, ay, az, current_time);

        // 获取姿态角度
        float roll, pitch, yaw;
        complementary_filter_get_angles(&filter, &roll, &pitch, &yaw);

        // 每10次打印一次结果
        if (i % 10 == 0) {
            printf("%4d | %8.2f | %9.2f | %7.2f\n", i + 1, roll, pitch, yaw);
        }
    }

    printf("-------------------------------------------\n\n");

    // 3. 最终结果
    float final_roll, final_pitch, final_yaw;
    complementary_filter_get_angles(&filter, &final_roll, &final_pitch, &final_yaw);

    printf("最终姿态估计结果：\n");
    printf("  横滚角 (Roll):  %8.2f 度\n", final_roll);
    printf("  俯仰角 (Pitch): %8.2f 度\n", final_pitch);
    printf("  航向角 (Yaw):   %8.2f 度\n\n", final_yaw);

    // 4. 演示不同alpha值的效果
    printf("==========================================\n");
    printf("不同Alpha值的效果对比\n");
    printf("==========================================\n\n");

    float test_alphas[] = {0.90f, 0.95f, 0.98f, 0.99f};

    for (int j = 0; j < 4; j++) {
        ComplementaryFilter_t test_filter;
        complementary_filter_init(&test_filter, test_alphas[j], 0);

        // 模拟10次更新
        for (int i = 0; i < 10; i++) {
            float gx, gy, gz, ax, ay, az;
            read_imu_sensor(&gx, &gy, &gz, &ax, &ay, &az);
            complementary_filter_update(&test_filter, gx, gy, gz, ax, ay, az, i * 10);
        }

        float test_roll, test_pitch, test_yaw;
        complementary_filter_get_angles(&test_filter, &test_roll, &test_pitch, &test_yaw);

        printf("Alpha = %.2f: Roll = %6.2f°, Pitch = %6.2f°\n",
               test_alphas[j], test_roll, test_pitch);
    }

    printf("\n==========================================\n");
    printf("程序运行完成\n");
    printf("==========================================\n");

    return 0;
}

// =====================================================================
// 实际嵌入式系统集成示例
// =====================================================================

/**
 * @brief 嵌入式系统中的典型使用方法
 *
 * 以STM32为例，在定时器中断中周期性调用
 */
#if 0  // 示例代码，实际使用时取消注释

// 全局滤波器
ComplementaryFilter_t g_filter;

// 初始化函数（在main中调用）
void system_init(void) {
    // ... 其他初始化 ...

    // 初始化互补滤波器
    complementary_filter_init(&g_filter, 0.96f, HAL_GetTick());

    // 启动定时器中断（如100Hz）
    HAL_TIM_Base_Start_IT(&htim2);
}

// 定时器中断回调（100Hz = 10ms间隔）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        float gx, gy, gz, ax, ay, az;

        // 读取MPU6050数据
        MPU6050_ReadGyro(&gx, &gy, &gz);
        MPU6050_ReadAccel(&ax, &ay, &az);

        // 更新滤波器
        complementary_filter_update(&g_filter, gx, gy, gz, ax, ay, az, HAL_GetTick());

        // 获取姿态角度用于控制
        float roll, pitch, yaw;
        complementary_filter_get_angles(&g_filter, &roll, &pitch, &yaw);

        // 使用角度进行控制
        pid_control(roll, pitch);
    }
}

#endif

// =====================================================================
// 性能优化技巧
// =====================================================================

/**
 * @brief 快速平方根倒数（用于向量归一化）
 *
 * 著名的Quake III算法，比标准sqrt()快约4倍
 * 适用于资源极度受限的单片机
 */
float fast_inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;           // 浮点数的二进制表示
    i = 0x5f3759df - (i >> 1);     // 魔术常数
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));  // 牛顿迭代一次
    return y;
}

/**
 * @brief 优化版互补滤波（减少三角函数调用）
 *
 * 适用于CPU没有FPU的单片机（如STM32F1）
 */
void complementary_filter_update_fast(ComplementaryFilter_t *filter,
                                     float gx, float gy, float gz,
                                     float ax, float ay, float az,
                                     uint32_t current_time) {
    float dt = (current_time - filter->last_time) / 1000.0f;
    filter->last_time = current_time;

    if (dt > 1.0f || dt <= 0.0f) dt = 0.01f;

    // 陀螺仪积分
    float gyro_roll = filter->roll + gx * dt;
    float gyro_pitch = filter->pitch + gy * dt;

    // 加速度计角度（使用查找表或近似计算替代atan2）
    // 这里仍使用atan2，但实际可以用更快的近似方法
    float accel_roll = safe_atan2(ay, az);
    float accel_pitch = safe_atan2(-ax, sqrtf(ay * ay + az * az));

    // 互补滤波
    filter->roll = filter->alpha * gyro_roll + (1.0f - filter->alpha) * accel_roll;
    filter->pitch = filter->alpha * gyro_pitch + (1.0f - filter->alpha) * accel_pitch;
    filter->yaw += gz * dt;

    // 归一化
    filter->roll = normalize_angle(filter->roll);
    filter->pitch = normalize_angle(filter->pitch);
    filter->yaw = normalize_angle(filter->yaw);
}

// =====================================================================
// 调试辅助函数
// =====================================================================

/**
 * @brief 打印滤波器状态（用于调试）
 */
void complementary_filter_print_status(ComplementaryFilter_t *filter) {
    printf("=== 互补滤波器状态 ===\n");
    printf("Roll:  %8.2f 度\n", filter->roll);
    printf("Pitch: %8.2f 度\n", filter->pitch);
    printf("Yaw:   %8.2f 度\n", filter->yaw);
    printf("Alpha: %8.4f\n", filter->alpha);
    printf("========================\n");
}

/**
 * @brief 计算滤波器性能指标
 */
typedef struct {
    float response_time;  // 响应时间（秒）
    float steady_error;   // 稳态误差（度）
    float noise_level;    // 噪声水平（度）
} FilterPerformance_t;

void evaluate_filter_performance(float alpha, FilterPerformance_t *perf) {
    // 简化的性能估计
    float tau = (1.0f - alpha) / alpha * 0.01f;  // 时间常数
    perf->response_time = 3 * tau;               // 3倍时间常数达到95%
    perf->steady_error = 0.5f * (1.0f - alpha);  // 稳态误差与alpha相关
    perf->noise_level = 2.0f * (1.0f - alpha);   // 噪声与加速度计权重相关

    printf("=== 滤波器性能评估 (Alpha=%.2f) ===\n", alpha);
    printf("响应时间:   %.3f 秒\n", perf->response_time);
    printf("稳态误差:   %.2f 度\n", perf->steady_error);
    printf("噪声水平:   %.2f 度\n", perf->noise_level);
    printf("=====================================\n");
}
