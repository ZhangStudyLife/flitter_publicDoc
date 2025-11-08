/**
 * ============================================================================
 * 文件名  : 01_一阶低通滤波.c
 * 描述    : 一阶低通滤波算法（RC滤波）的完整C语言实现
 * 作者    : AI Assistant
 * 日期    : 2025-01-09
 * 版本    : v1.0
 * ============================================================================
 *
 * 算法原理：
 *     Y(n) = α × X(n) + (1 - α) × Y(n-1)
 *
 * 参数说明：
 *     α : 滤波系数，范围 (0, 1]
 *         - α 越大，跟踪速度越快，但滤波效果越弱
 *         - α 越小，滤波效果越好，但响应越慢
 *
 * 适用场景：
 *     - 温度传感器数据平滑
 *     - 电压监测
 *     - ADC采样数据去噪
 *     - 缓慢变化的信号处理
 * ============================================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

// ============================================================================
// 方法1：基础版本 - 单通道滤波器
// ============================================================================

/**
 * @brief  一阶低通滤波器（基础版）
 * @param  input : 本次采样的原始值
 * @param  alpha : 滤波系数 (0 < alpha <= 1)
 * @return 滤波后的输出值
 * @note   使用静态变量保存上次输出，仅适用于单个传感器
 */
float FirstOrderLowPassFilter(float input, float alpha)
{
    static float output = 0.0f;      // 静态变量：保存上一次的输出值
    static uint8_t first_run = 1;    // 首次运行标志

    // 第一次运行时，直接用输入值初始化
    if (first_run) {
        output = input;
        first_run = 0;
        return output;
    }

    // 一阶低通滤波核心公式
    output = alpha * input + (1.0f - alpha) * output;

    return output;
}

// ============================================================================
// 方法2：结构体版本 - 多通道滤波器（推荐）
// ============================================================================

/**
 * @brief 低通滤波器结构体定义
 */
typedef struct {
    float alpha;        // 滤波系数
    float output;       // 上次输出值（历史值）
    uint8_t is_init;    // 初始化标志 (0:未初始化, 1:已初始化)
} LowPassFilter_t;

/**
 * @brief  初始化低通滤波器
 * @param  filter     : 滤波器结构体指针
 * @param  alpha      : 滤波系数 (0 < alpha <= 1)
 * @param  init_value : 初始值（建议设为第一次测量值或期望的初始状态）
 * @retval None
 */
void LPF_Init(LowPassFilter_t *filter, float alpha, float init_value)
{
    filter->alpha = alpha;
    filter->output = init_value;
    filter->is_init = 1;
}

/**
 * @brief  更新低通滤波器（执行一次滤波计算）
 * @param  filter : 滤波器结构体指针
 * @param  input  : 本次采样的原始值
 * @return 滤波后的输出值
 */
float LPF_Update(LowPassFilter_t *filter, float input)
{
    // 如果未初始化，自动用第一次输入值初始化
    if (!filter->is_init) {
        filter->output = input;
        filter->is_init = 1;
        return input;
    }

    // 一阶低通滤波公式：新值 = α×本次测量 + (1-α)×上次输出
    filter->output = filter->alpha * input + (1.0f - filter->alpha) * filter->output;

    return filter->output;
}

/**
 * @brief  重置滤波器
 * @param  filter : 滤波器结构体指针
 * @retval None
 */
void LPF_Reset(LowPassFilter_t *filter)
{
    filter->output = 0.0f;
    filter->is_init = 0;
}

/**
 * @brief  动态修改滤波系数
 * @param  filter : 滤波器结构体指针
 * @param  alpha  : 新的滤波系数
 * @retval None
 */
void LPF_SetAlpha(LowPassFilter_t *filter, float alpha)
{
    if (alpha > 0.0f && alpha <= 1.0f) {
        filter->alpha = alpha;
    }
}

// ============================================================================
// 方法3：进阶版本 - 自适应α值
// ============================================================================

/**
 * @brief  自适应低通滤波器（α值根据信号变化自动调整）
 * @param  filter : 滤波器结构体指针
 * @param  input  : 本次采样值
 * @return 滤波后的输出值
 * @note   当信号剧烈变化时增大α（快速跟踪），平稳时减小α（强滤波）
 */
float LPF_Adaptive_Update(LowPassFilter_t *filter, float input)
{
    // 如果未初始化，自动初始化
    if (!filter->is_init) {
        filter->output = input;
        filter->is_init = 1;
        return input;
    }

    // 计算本次输入与上次输出的差值（变化幅度）
    float diff = fabs(input - filter->output);

    // 根据变化幅度自适应调整α值
    float adaptive_alpha;
    if (diff < 1.0f) {
        // 变化很小，使用小α，强滤波（去噪）
        adaptive_alpha = 0.1f;
    } else if (diff < 5.0f) {
        // 中等变化，使用中等α
        adaptive_alpha = 0.3f;
    } else {
        // 剧烈变化，使用大α，快速跟踪真实信号
        adaptive_alpha = 0.7f;
    }

    // 执行滤波
    filter->output = adaptive_alpha * input + (1.0f - adaptive_alpha) * filter->output;

    return filter->output;
}

// ============================================================================
// 方法4：整数版本（定点数实现，节省计算资源）
// ============================================================================

/**
 * @brief  整数版一阶低通滤波（定点数实现）
 * @param  input : 本次采样值（整数）
 * @param  alpha_percent : α百分比值 (0~100)，例如20表示α=0.2
 * @return 滤波后的输出值（整数）
 * @note   适用于低端单片机，避免浮点运算
 */
int32_t LPF_Integer(int32_t input, uint8_t alpha_percent)
{
    static int32_t output = 0;
    static uint8_t first_run = 1;

    if (first_run) {
        output = input;
        first_run = 0;
        return output;
    }

    // 使用整数运算实现：output = (alpha% × input + (100-alpha%) × output) / 100
    output = (alpha_percent * input + (100 - alpha_percent) * output) / 100;

    return output;
}

// ============================================================================
// 辅助函数：根据时间常数计算α值
// ============================================================================

/**
 * @brief  根据采样周期和时间常数计算α值
 * @param  sample_period : 采样周期 (单位: 秒)
 * @param  time_constant : 时间常数 τ (单位: 秒)
 * @return 计算得到的α值
 * @note   公式: α = T / (T + τ)
 *         τ越大，滤波效果越强（但延迟也越大）
 *
 * @example
 *     float alpha = Calculate_Alpha(0.01, 0.1);  // 采样周期10ms，时间常数100ms
 */
float Calculate_Alpha(float sample_period, float time_constant)
{
    return sample_period / (sample_period + time_constant);
}

// ============================================================================
// 测试示例
// ============================================================================

/**
 * @brief  模拟带噪声的传感器读数
 * @param  true_value : 真实值
 * @param  noise_amplitude : 噪声幅度
 * @return 带噪声的模拟读数
 */
float Simulate_Sensor_Reading(float true_value, float noise_amplitude)
{
    // 生成 [-1, 1] 范围的随机数
    float random_noise = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
    return true_value + random_noise * noise_amplitude;
}

/**
 * @brief  示例1：温度传感器滤波
 */
void Example_Temperature_Sensor(void)
{
    printf("\n========== 示例1：温度传感器滤波 ==========\n");

    LowPassFilter_t temp_filter;
    LPF_Init(&temp_filter, 0.15f, 25.0f);  // α=0.15，初始温度25℃

    printf("真实温度: 25.0℃ (模拟噪声幅度: ±2℃)\n\n");
    printf("采样次数  原始读数  滤波输出\n");
    printf("-------------------------------\n");

    for (int i = 1; i <= 10; i++) {
        // 模拟带噪声的温度读数（真实值25℃，噪声±2℃）
        float raw_temp = Simulate_Sensor_Reading(25.0f, 2.0f);

        // 滤波处理
        float filtered_temp = LPF_Update(&temp_filter, raw_temp);

        printf("  %2d      %6.2f℃   %6.2f℃\n", i, raw_temp, filtered_temp);
    }
}

/**
 * @brief  示例2：三轴加速度计滤波
 */
void Example_Accelerometer(void)
{
    printf("\n========== 示例2：三轴加速度计滤波 ==========\n");

    // 为X、Y、Z三个轴分别创建滤波器
    LowPassFilter_t acc_x, acc_y, acc_z;

    // 初始化（静止状态：X=0, Y=0, Z=9.8m/s²）
    LPF_Init(&acc_x, 0.3f, 0.0f);
    LPF_Init(&acc_y, 0.3f, 0.0f);
    LPF_Init(&acc_z, 0.3f, 9.8f);

    printf("静止状态理想值: X=0.0, Y=0.0, Z=9.8 m/s² (噪声幅度: ±0.5)\n\n");
    printf("次数   原始X   滤波X   原始Y   滤波Y   原始Z   滤波Z\n");
    printf("------------------------------------------------------------\n");

    for (int i = 1; i <= 8; i++) {
        // 模拟带噪声的加速度读数
        float raw_x = Simulate_Sensor_Reading(0.0f, 0.5f);
        float raw_y = Simulate_Sensor_Reading(0.0f, 0.5f);
        float raw_z = Simulate_Sensor_Reading(9.8f, 0.5f);

        // 滤波处理
        float filt_x = LPF_Update(&acc_x, raw_x);
        float filt_y = LPF_Update(&acc_y, raw_y);
        float filt_z = LPF_Update(&acc_z, raw_z);

        printf(" %d    %6.2f  %6.2f  %6.2f  %6.2f  %6.2f  %6.2f\n",
               i, raw_x, filt_x, raw_y, filt_y, raw_z, filt_z);
    }
}

/**
 * @brief  示例3：不同α值的对比测试
 */
void Example_Alpha_Comparison(void)
{
    printf("\n========== 示例3：不同α值的效果对比 ==========\n");

    LowPassFilter_t filter_small, filter_medium, filter_large;

    LPF_Init(&filter_small, 0.1f, 50.0f);   // 小α：强滤波
    LPF_Init(&filter_medium, 0.3f, 50.0f);  // 中α：平衡
    LPF_Init(&filter_large, 0.7f, 50.0f);   // 大α：快速响应

    printf("信号：从50突变到100 (噪声幅度: ±5)\n\n");
    printf("次数  原始值   α=0.1   α=0.3   α=0.7\n");
    printf("------------------------------------------\n");

    for (int i = 1; i <= 12; i++) {
        // 第5次开始，真实值从50跳变到100
        float true_value = (i >= 5) ? 100.0f : 50.0f;
        float raw_value = Simulate_Sensor_Reading(true_value, 5.0f);

        float out_small = LPF_Update(&filter_small, raw_value);
        float out_medium = LPF_Update(&filter_medium, raw_value);
        float out_large = LPF_Update(&filter_large, raw_value);

        printf(" %2d   %6.1f   %6.1f  %6.1f  %6.1f\n",
               i, raw_value, out_small, out_medium, out_large);
    }

    printf("\n观察：α越小，输出越平滑但跟踪越慢；α越大，跟踪越快但抗噪性越差\n");
}

/**
 * @brief  示例4：自适应滤波器
 */
void Example_Adaptive_Filter(void)
{
    printf("\n========== 示例4：自适应滤波器 ==========\n");

    LowPassFilter_t adaptive_filter;
    LPF_Init(&adaptive_filter, 0.2f, 50.0f);  // 初始α=0.2（实际会自动调整）

    printf("场景：信号从50缓慢上升，然后突然跳到100\n\n");
    printf("次数  原始值   自适应输出\n");
    printf("---------------------------\n");

    for (int i = 1; i <= 15; i++) {
        float true_value;
        if (i <= 5) {
            true_value = 50.0f;  // 初始稳定在50
        } else if (i <= 10) {
            true_value = 50.0f + (i - 5) * 2.0f;  // 缓慢上升
        } else {
            true_value = 100.0f;  // 突然跳到100
        }

        float raw_value = Simulate_Sensor_Reading(true_value, 3.0f);
        float adaptive_out = LPF_Adaptive_Update(&adaptive_filter, raw_value);

        printf(" %2d   %6.1f     %6.1f\n", i, raw_value, adaptive_out);
    }

    printf("\n说明：自适应滤波器在信号剧烈变化时会自动加快响应速度\n");
}

/**
 * @brief  示例5：整数版滤波器（适用于低端MCU）
 */
void Example_Integer_Filter(void)
{
    printf("\n========== 示例5：整数版滤波器 ==========\n");

    printf("场景：ADC读取电池电压 (0~4095对应0~3.3V)\n\n");
    printf("次数  原始ADC值  滤波后ADC值\n");
    printf("--------------------------------\n");

    for (int i = 1; i <= 8; i++) {
        // 模拟ADC读数（真实值约2048，噪声±50）
        int32_t raw_adc = 2048 + (rand() % 101 - 50);

        // 整数滤波（α=20%）
        int32_t filtered_adc = LPF_Integer(raw_adc, 20);

        printf(" %2d      %4d         %4d\n", i, raw_adc, filtered_adc);
    }
}

// ============================================================================
// 主函数
// ============================================================================

int main(void)
{
    // 初始化随机数种子
    srand((unsigned int)time(NULL));

    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║      一阶低通滤波算法 - 完整测试程序                  ║\n");
    printf("║      First-Order Low-Pass Filter Demo                ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n");

    // 运行所有示例
    Example_Temperature_Sensor();
    Example_Accelerometer();
    Example_Alpha_Comparison();
    Example_Adaptive_Filter();
    Example_Integer_Filter();

    printf("\n========== 参数选择建议 ==========\n");
    printf("应用场景              建议α值\n");
    printf("温度传感器            0.10 ~ 0.20\n");
    printf("电池电压监测          0.10 ~ 0.15\n");
    printf("距离传感器            0.20 ~ 0.30\n");
    printf("加速度计/陀螺仪       0.30 ~ 0.50\n");
    printf("ADC采样               0.30 ~ 0.50\n");
    printf("摇杆/旋钮             0.40 ~ 0.60\n");
    printf("===================================\n");

    printf("\n程序运行完成！\n");

    return 0;
}

/* ============================================================================
 * 使用说明：
 *
 * 1. 编译命令：
 *    gcc -o filter 01_一阶低通滤波.c -lm
 *
 * 2. 运行程序：
 *    ./filter (Linux/Mac)
 *    filter.exe (Windows)
 *
 * 3. 移植到单片机：
 *    - 保留 LowPassFilter_t 结构体定义
 *    - 保留 LPF_Init() 和 LPF_Update() 函数
 *    - 删除 main() 函数中的测试代码
 *    - 在定时器中断或主循环中调用 LPF_Update()
 *
 * 4. 性能优化：
 *    - 如果MCU不支持浮点，使用整数版 LPF_Integer()
 *    - 如果需要处理多个传感器，为每个传感器创建独立的滤波器实例
 *    - 可以将α值预先计算为宏定义，减少运行时计算
 *
 * 5. 常见问题：
 *    Q: 滤波后数据仍然跳动？
 *    A: 减小α值（如从0.3改为0.1）
 *
 *    Q: 滤波后响应太慢？
 *    A: 增大α值（如从0.2改为0.5）
 *
 *    Q: 如何选择合适的α值？
 *    A: 先用0.3测试，然后根据实际效果调整
 *
 * ============================================================================
 */
