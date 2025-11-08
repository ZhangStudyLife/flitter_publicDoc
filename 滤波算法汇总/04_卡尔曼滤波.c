/**
 * @file    04_卡尔曼滤波.c
 * @brief   卡尔曼滤波算法 - 简化的一维实现
 * @date    2024
 * @author  嵌入式滤波算法教程
 *
 * @note    这是一个教学用的简化版本，适合初学者理解卡尔曼滤波原理
 *          实际应用中可以扩展为多维卡尔曼滤波或扩展卡尔曼滤波(EKF)
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

/* ======================== 数据结构定义 ======================== */

/**
 * @brief   一维卡尔曼滤波器结构体
 * @details 包含所有必要的状态变量和参数
 */
typedef struct {
    float x;      /**< 状态估计值 (posteriori estimate) */
    float P;      /**< 估计误差协方差 (posteriori error covariance) */
    float Q;      /**< 过程噪声协方差 (process noise covariance) - 对系统模型的不信任度 */
    float R;      /**< 测量噪声协方差 (measurement noise covariance) - 对传感器的不信任度 */
    float K;      /**< 卡尔曼增益 (Kalman gain) - 预测和测量的权重系数 */
} KalmanFilter_t;

/* ======================== 函数声明 ======================== */

void Kalman_Init(KalmanFilter_t *kf, float init_x, float init_P,
                 float process_noise, float measure_noise);
float Kalman_Filter(KalmanFilter_t *kf, float measurement);
void Kalman_GetState(KalmanFilter_t *kf, float *estimate, float *uncertainty);

/* ======================== 辅助函数（用于测试） ======================== */

float generate_true_value(float time);
float add_noise(float value, float noise_std);
void print_filter_state(KalmanFilter_t *kf, float true_val, float measured, float filtered);

/* ======================== 核心算法实现 ======================== */

/**
 * @brief   初始化卡尔曼滤波器
 * @param   kf              卡尔曼滤波器指针
 * @param   init_x          初始状态估计值（可以设为第一次测量值）
 * @param   init_P          初始估计误差协方差（表示初始不确定性，一般设为1~10）
 * @param   process_noise   过程噪声协方差 Q（系统模型的不确定性）
 * @param   measure_noise   测量噪声协方差 R（传感器的不确定性）
 *
 * @note    参数选择建议：
 *          - Q 通常远小于 R，约为 R 的 1/10 ~ 1/100
 *          - Q 越大，越信任测量值，响应越快但噪声越大
 *          - R 越大，越信任预测值，滤波越平滑但响应越慢
 */
void Kalman_Init(KalmanFilter_t *kf, float init_x, float init_P,
                 float process_noise, float measure_noise)
{
    kf->x = init_x;           // 初始状态估计
    kf->P = init_P;           // 初始误差协方差
    kf->Q = process_noise;    // 过程噪声
    kf->R = measure_noise;    // 测量噪声
    kf->K = 0.0f;             // 卡尔曼增益初始化为0
}

/**
 * @brief   卡尔曼滤波处理函数（核心算法）
 * @param   kf              卡尔曼滤波器指针
 * @param   measurement     当前测量值
 * @return  float           滤波后的最优估计值
 *
 * @details 实现标准的"预测-更新"两阶段卡尔曼滤波
 *
 *          预测阶段（时间更新）：
 *          1. 状态预测：x̂(k|k-1) = x̂(k-1|k-1)
 *          2. 协方差预测：P(k|k-1) = P(k-1|k-1) + Q
 *
 *          更新阶段（测量更新）：
 *          3. 计算卡尔曼增益：K(k) = P(k|k-1) / (P(k|k-1) + R)
 *          4. 状态更新：x̂(k|k) = x̂(k|k-1) + K(k) × [z(k) - x̂(k|k-1)]
 *          5. 协方差更新：P(k|k) = (1 - K(k)) × P(k|k-1)
 */
float Kalman_Filter(KalmanFilter_t *kf, float measurement)
{
    /* ========== 第一阶段：预测（Predict） ========== */

    // 1. 状态预测（先验估计）
    // 对于恒定值跟踪，预测值等于上一次的估计值
    // 如果系统有运动模型，这里需要加入状态转移方程
    float x_predict = kf->x;

    // 2. 协方差预测（先验误差协方差）
    // 预测的不确定性 = 上次的不确定性 + 过程噪声
    float P_predict = kf->P + kf->Q;

    /* ========== 第二阶段：更新（Update） ========== */

    // 3. 计算卡尔曼增益
    // K 的范围在 0~1 之间
    // - K 接近 0：更相信预测值（R很大，传感器不可靠）
    // - K 接近 1：更相信测量值（R很小，传感器很可靠）
    kf->K = P_predict / (P_predict + kf->R);

    // 4. 状态更新（后验估计）
    // 新估计 = 预测值 + 增益 × (测量值 - 预测值)
    //        = 预测值 + 增益 × 测量残差
    kf->x = x_predict + kf->K * (measurement - x_predict);

    // 5. 协方差更新（后验误差协方差）
    // 经过测量更新后，不确定性降低
    kf->P = (1.0f - kf->K) * P_predict;

    return kf->x;  // 返回最优估计值
}

/**
 * @brief   获取当前滤波器状态
 * @param   kf              卡尔曼滤波器指针
 * @param   estimate        输出参数：当前状态估计值
 * @param   uncertainty     输出参数：当前估计的不确定性（标准差）
 */
void Kalman_GetState(KalmanFilter_t *kf, float *estimate, float *uncertainty)
{
    if (estimate != NULL) {
        *estimate = kf->x;
    }
    if (uncertainty != NULL) {
        *uncertainty = sqrtf(kf->P);  // 协方差的平方根 = 标准差
    }
}

/* ======================== 测试辅助函数 ======================== */

/**
 * @brief   生成真实值（模拟真实的物理量）
 * @param   time    时间（秒）
 * @return  float   真实值
 * @note    这里模拟一个恒定值（例如电池电压），实际可以是任意变化的信号
 */
float generate_true_value(float time)
{
    // 模拟一个恒定的电压值，带有微小的随机波动
    const float VOLTAGE = 3.7f;  // 标准电压 3.7V
    return VOLTAGE + 0.01f * sinf(time * 0.5f);  // 加入微小的正弦波动
}

/**
 * @brief   添加高斯白噪声
 * @param   value       原始值
 * @param   noise_std   噪声标准差
 * @return  float       加噪后的值
 */
float add_noise(float value, float noise_std)
{
    // Box-Muller变换生成高斯分布随机数
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    float gauss = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265f * u2);
    return value + noise_std * gauss;
}

/**
 * @brief   打印滤波器状态（调试用）
 */
void print_filter_state(KalmanFilter_t *kf, float true_val, float measured, float filtered)
{
    printf("真实值: %.4f | 测量值: %.4f | 滤波值: %.4f | ",
           true_val, measured, filtered);
    printf("增益K: %.4f | 误差P: %.4f\n", kf->K, kf->P);
}

/* ======================== 示例程序 ======================== */

/**
 * @brief   示例1：恒定电压测量
 * @details 模拟使用ADC测量一个恒定的电压值，传感器有噪声
 */
void example_voltage_measurement(void)
{
    printf("\n========== 示例1：恒定电压测量（卡尔曼滤波） ==========\n");

    // 1. 创建滤波器实例
    KalmanFilter_t voltage_filter;

    // 2. 初始化参数
    float init_x = 3.7f;      // 初始猜测：3.7V（也可以用第一次测量值）
    float init_P = 1.0f;      // 初始不确定性：中等
    float Q = 0.001f;         // 过程噪声：很小（电压基本恒定）
    float R = 0.1f;           // 测量噪声：0.1V 标准差（传感器精度）

    Kalman_Init(&voltage_filter, init_x, init_P, Q, R);

    // 3. 模拟100次测量
    printf("\n前10次测量结果：\n");
    printf("-------------------------------------------------------------\n");

    for (int i = 0; i < 100; i++) {
        // 生成真实值（实际中这是未知的）
        float true_voltage = generate_true_value(i * 0.1f);

        // 模拟传感器测量（真实值 + 噪声）
        float measured_voltage = add_noise(true_voltage, 0.1f);

        // 卡尔曼滤波处理
        float filtered_voltage = Kalman_Filter(&voltage_filter, measured_voltage);

        // 打印前10次结果
        if (i < 10) {
            printf("第%2d次 | ", i + 1);
            print_filter_state(&voltage_filter, true_voltage, measured_voltage, filtered_voltage);
        }
    }

    // 4. 显示最终收敛状态
    printf("\n最终收敛状态：\n");
    float final_estimate, final_uncertainty;
    Kalman_GetState(&voltage_filter, &final_estimate, &final_uncertainty);
    printf("估计电压: %.4f V ± %.4f V\n", final_estimate, final_uncertainty);
    printf("卡尔曼增益: %.4f (越小表示越信任预测)\n", voltage_filter.K);
}

/**
 * @brief   示例2：不同参数对比
 * @details 对比不同 Q/R 比值对滤波效果的影响
 */
void example_parameter_comparison(void)
{
    printf("\n========== 示例2：参数调节对比 ==========\n");

    // 准备三组参数
    struct {
        float Q;
        float R;
        const char *description;
    } configs[] = {
        {0.001f, 0.1f, "平滑优先（Q小）"},
        {0.01f,  0.1f, "平衡模式"},
        {0.1f,   0.1f, "响应优先（Q大）"}
    };

    for (int cfg = 0; cfg < 3; cfg++) {
        printf("\n--- 配置%d: %s (Q=%.3f, R=%.3f) ---\n",
               cfg + 1, configs[cfg].description, configs[cfg].Q, configs[cfg].R);

        KalmanFilter_t kf;
        Kalman_Init(&kf, 3.7f, 1.0f, configs[cfg].Q, configs[cfg].R);

        // 测试5次
        printf("次数 | 测量值  | 滤波值  | 增益K\n");
        printf("-----|---------|---------|-------\n");

        for (int i = 0; i < 5; i++) {
            float true_val = 3.7f;
            float measured = add_noise(true_val, 0.1f);
            float filtered = Kalman_Filter(&kf, measured);

            printf("%4d | %.4f | %.4f | %.4f\n",
                   i + 1, measured, filtered, kf.K);
        }
    }
}

/**
 * @brief   示例3：阶跃响应测试
 * @details 测试卡尔曼滤波器对突变信号的响应速度
 */
void example_step_response(void)
{
    printf("\n========== 示例3：阶跃响应测试 ==========\n");
    printf("模拟电压从 3.0V 突变到 5.0V\n\n");

    KalmanFilter_t kf;
    Kalman_Init(&kf, 3.0f, 1.0f, 0.01f, 0.1f);

    printf("时刻 | 真实值 | 测量值 | 滤波值 | 误差\n");
    printf("-----|--------|--------|--------|---------\n");

    for (int i = 0; i < 20; i++) {
        // 前10次测量 3.0V，后10次测量 5.0V
        float true_val = (i < 10) ? 3.0f : 5.0f;
        float measured = add_noise(true_val, 0.1f);
        float filtered = Kalman_Filter(&kf, measured);
        float error = fabsf(filtered - true_val);

        printf("%4d | %.4f | %.4f | %.4f | %.4f\n",
               i + 1, true_val, measured, filtered, error);
    }
}

/**
 * @brief   示例4：实际嵌入式应用伪代码
 * @details 展示在实际项目中如何使用
 */
void example_embedded_usage(void)
{
    printf("\n========== 示例4：嵌入式应用伪代码 ==========\n");
    printf("/*\n");
    printf(" * 实际项目中的使用示例（伪代码）\n");
    printf(" */\n\n");

    printf("// 1. 在头文件中定义全局滤波器\n");
    printf("KalmanFilter_t g_voltage_kf;\n\n");

    printf("// 2. 在初始化函数中设置参数\n");
    printf("void System_Init(void) {\n");
    printf("    // 读取第一次ADC值作为初始估计\n");
    printf("    float first_reading = ADC_ReadVoltage();\n");
    printf("    Kalman_Init(&g_voltage_kf, first_reading, 1.0f, 0.001f, 0.1f);\n");
    printf("}\n\n");

    printf("// 3. 在主循环或中断中调用滤波\n");
    printf("void ADC_SampleCallback(void) {\n");
    printf("    float raw_voltage = ADC_ReadVoltage();\n");
    printf("    float filtered_voltage = Kalman_Filter(&g_voltage_kf, raw_voltage);\n");
    printf("    \n");
    printf("    // 使用滤波后的值进行控制决策\n");
    printf("    if (filtered_voltage < LOW_BATTERY_THRESHOLD) {\n");
    printf("        Trigger_LowBattery_Warning();\n");
    printf("    }\n");
    printf("}\n");
}

/* ======================== 主函数 ======================== */

/**
 * @brief   主函数 - 运行所有示例
 */
int main(void)
{
    // 初始化随机数种子
    srand((unsigned int)time(NULL));

    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║        卡尔曼滤波算法 - C语言实现示例程序           ║\n");
    printf("║        Kalman Filter - One Dimensional Version        ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n");

    // 运行示例
    example_voltage_measurement();    // 示例1：基本使用
    example_parameter_comparison();   // 示例2：参数影响
    example_step_response();          // 示例3：阶跃响应
    example_embedded_usage();         // 示例4：实际应用

    printf("\n========== 程序运行完毕 ==========\n");
    printf("\n调参建议：\n");
    printf("1. 先测量传感器静态噪声，确定 R 值\n");
    printf("2. 设置 Q = R / 10 作为起点\n");
    printf("3. 如果响应太慢，增大 Q；如果噪声太大，减小 Q\n");
    printf("4. 观察卡尔曼增益 K，理想值在 0.3~0.7 之间\n");

    return 0;
}

/* ======================== 扩展功能（可选） ======================== */

/**
 * @brief   带控制输入的卡尔曼滤波（扩展版本）
 * @param   kf              卡尔曼滤波器指针
 * @param   measurement     测量值
 * @param   control_input   控制输入（已知的状态变化量）
 * @return  float           滤波后的估计值
 *
 * @note    适用于有主动控制的系统，例如：
 *          - 电机速度控制：control_input = 加速度指令
 *          - 温度控制：control_input = 加热功率
 */
float Kalman_Filter_WithControl(KalmanFilter_t *kf, float measurement, float control_input)
{
    // 预测阶段：考虑控制输入
    float x_predict = kf->x + control_input;  // 状态转移方程
    float P_predict = kf->P + kf->Q;

    // 更新阶段：与标准版本相同
    kf->K = P_predict / (P_predict + kf->R);
    kf->x = x_predict + kf->K * (measurement - x_predict);
    kf->P = (1.0f - kf->K) * P_predict;

    return kf->x;
}

/**
 * @brief   自适应卡尔曼滤波（动态调整 R 值）
 * @details 根据测量残差自动调整测量噪声协方差
 * @note    当传感器可靠性动态变化时使用
 */
float Kalman_Filter_Adaptive(KalmanFilter_t *kf, float measurement)
{
    // 计算测量残差
    float residual = fabsf(measurement - kf->x);

    // 残差大说明测量不可靠，增大 R
    if (residual > 0.5f) {
        kf->R = kf->R * 1.2f;  // 增大20%
        if (kf->R > 10.0f) kf->R = 10.0f;  // 限制上限
    } else {
        kf->R = kf->R * 0.95f;  // 减小5%
        if (kf->R < 0.01f) kf->R = 0.01f;  // 限制下限
    }

    // 执行标准卡尔曼滤波
    return Kalman_Filter(kf, measurement);
}

/* ======================== 文件结束 ======================== */

/**
 * 编译命令：
 *   gcc 04_卡尔曼滤波.c -o kalman_filter -lm
 *
 * 运行：
 *   ./kalman_filter
 *
 * 预期输出：
 *   - 示例1：展示滤波效果，观察卡尔曼增益逐渐收敛
 *   - 示例2：对比不同参数的效果
 *   - 示例3：测试对阶跃信号的响应
 *   - 示例4：实际应用伪代码
 *
 * 学习要点：
 *   1. 理解"预测-更新"两阶段过程
 *   2. 观察卡尔曼增益 K 的变化规律
 *   3. 实验不同 Q/R 比值的影响
 *   4. 对比滤波前后的噪声抑制效果
 */
