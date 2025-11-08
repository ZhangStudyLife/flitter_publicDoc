/**
 ******************************************************************************
 * @file    07_滞环滤波.c
 * @author  滤波算法系列
 * @version V1.0
 * @date    2025-11-09
 * @brief   滞环滤波算法实现与应用示例
 ******************************************************************************
 * @description
 * 滞环滤波（Hysteresis Filter）是一种基于阈值的数字滤波算法。
 *
 * 核心原理：
 * - 只有当新采样值与当前输出值的差值超过设定阈值时，才更新输出
 * - 小于阈值的变化会被忽略，输出保持不变
 *
 * 特点：
 * - 计算量极小（一次减法+一次比较）
 * - 抗小幅噪声能力极强
 * - 特别适合开关量信号和状态切换
 * - 输出呈阶跃状，不平滑
 *
 * 适用场景：
 * - 按键消抖
 * - 温度/液位等开关控制
 * - 状态机的状态切换
 * - 传感器数据稳定化
 ******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ============================================================================
 * 基础版本：使用静态变量
 * ============================================================================ */

/**
 * @brief  基础滞环滤波器
 * @param  new_value: 当前采样值
 * @param  threshold: 滞环阈值（死区宽度）
 * @return 滤波后的输出值
 * @note   使用静态变量保存状态，只能用于单个滤波器
 */
float hysteresis_filter_basic(float new_value, float threshold)
{
    static float output = 0.0f;        // 保存上次输出值
    static int first_run = 1;          // 首次运行标志

    // 首次运行时，直接用新值初始化
    if (first_run) {
        output = new_value;
        first_run = 0;
        return output;
    }

    // 计算新值与当前输出的差值绝对值
    float diff = new_value - output;
    if (diff < 0) {
        diff = -diff;  // 取绝对值（避免使用fabs节省资源）
    }

    // 判断差值是否超过阈值
    if (diff > threshold) {
        output = new_value;  // 超过阈值，更新输出
    }
    // 否则保持原值不变（output不变）

    return output;
}


/* ============================================================================
 * 结构体版本：支持多个滤波器实例
 * ============================================================================ */

/**
 * @brief 滞环滤波器结构体定义
 */
typedef struct {
    float output;           // 当前输出值
    float threshold;        // 滞环阈值
    int initialized;        // 初始化标志（0=未初始化，1=已初始化）
} HysteresisFilter;


/**
 * @brief  初始化滞环滤波器
 * @param  filter: 滤波器指针
 * @param  threshold: 滞环阈值
 * @retval None
 */
void hysteresis_init(HysteresisFilter *filter, float threshold)
{
    filter->output = 0.0f;
    filter->threshold = threshold;
    filter->initialized = 0;
}


/**
 * @brief  滞环滤波更新（核心算法）
 * @param  filter: 滤波器指针
 * @param  new_value: 新采样值
 * @return 滤波后的输出值
 */
float hysteresis_update(HysteresisFilter *filter, float new_value)
{
    // 首次运行，直接使用新值初始化输出
    if (!filter->initialized) {
        filter->output = new_value;
        filter->initialized = 1;
        return filter->output;
    }

    // 计算差值的绝对值
    float diff = new_value - filter->output;
    if (diff < 0) {
        diff = -diff;
    }

    // 判断是否超过阈值
    if (diff > filter->threshold) {
        filter->output = new_value;  // 更新输出
    }
    // 否则保持原值

    return filter->output;
}


/**
 * @brief  重置滤波器
 * @param  filter: 滤波器指针
 * @retval None
 */
void hysteresis_reset(HysteresisFilter *filter)
{
    filter->output = 0.0f;
    filter->initialized = 0;
}


/**
 * @brief  获取当前输出值（不更新）
 * @param  filter: 滤波器指针
 * @return 当前输出值
 */
float hysteresis_get_output(HysteresisFilter *filter)
{
    return filter->output;
}


/* ============================================================================
 * 增强版本：双阈值滞环
 * ============================================================================ */

/**
 * @brief 双阈值滞环滤波器结构体
 */
typedef struct {
    float output;
    float upper_threshold;  // 上限阈值
    float lower_threshold;  // 下限阈值
    int initialized;
} DualHysteresisFilter;


/**
 * @brief  初始化双阈值滞环滤波器
 * @param  filter: 滤波器指针
 * @param  upper_threshold: 上限阈值
 * @param  lower_threshold: 下限阈值
 * @retval None
 */
void dual_hysteresis_init(DualHysteresisFilter *filter,
                          float upper_threshold,
                          float lower_threshold)
{
    filter->output = 0.0f;
    filter->upper_threshold = upper_threshold;
    filter->lower_threshold = lower_threshold;
    filter->initialized = 0;
}


/**
 * @brief  双阈值滞环滤波更新
 * @param  filter: 滤波器指针
 * @param  new_value: 新采样值
 * @return 滤波后的输出值
 * @note   向上变化和向下变化可以使用不同的阈值
 */
float dual_hysteresis_update(DualHysteresisFilter *filter, float new_value)
{
    if (!filter->initialized) {
        filter->output = new_value;
        filter->initialized = 1;
        return filter->output;
    }

    float diff = new_value - filter->output;

    // 向上变化（新值 > 旧值）
    if (diff > filter->upper_threshold) {
        filter->output = new_value;
    }
    // 向下变化（新值 < 旧值）
    else if (diff < -filter->lower_threshold) {
        filter->output = new_value;
    }
    // 在阈值范围内，保持不变

    return filter->output;
}


/* ============================================================================
 * 实用工具函数
 * ============================================================================ */

/**
 * @brief  生成带噪声的模拟信号
 * @param  base_value: 基准值
 * @param  noise_amplitude: 噪声幅度
 * @return 带噪声的信号值
 */
float generate_noisy_signal(float base_value, float noise_amplitude)
{
    // 生成 [-1, 1] 范围的随机数
    float random_noise = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
    return base_value + random_noise * noise_amplitude;
}


/* ============================================================================
 * 应用示例
 * ============================================================================ */

/**
 * @brief  示例1：基础滤波演示
 */
void example_basic_filtering(void)
{
    printf("\n========== 示例1：基础滞环滤波 ==========\n");
    printf("模拟一个在50附近波动的信号，噪声幅度为±2\n");
    printf("设置滞环阈值为5.0\n\n");

    HysteresisFilter filter;
    hysteresis_init(&filter, 5.0f);  // 阈值设为5

    printf("  采样值  |  滤波输出  |  说明\n");
    printf("----------|------------|------------------------\n");

    // 模拟10个带噪声的采样点
    float test_values[] = {50.5, 51.2, 49.8, 52.0, 48.5, 56.0, 55.5, 54.8, 60.0, 59.5};

    for (int i = 0; i < 10; i++) {
        float filtered = hysteresis_update(&filter, test_values[i]);

        printf("  %6.2f  |   %6.2f   | ", test_values[i], filtered);

        // 说明是否更新了输出
        if (i == 0) {
            printf("首次初始化\n");
        } else {
            float diff = fabs(test_values[i] - test_values[i-1]);
            if (fabs(test_values[i] - filtered) < 0.01) {
                printf("差值%.2f > 阈值，更新\n", diff);
            } else {
                printf("差值%.2f ≤ 阈值，保持\n", diff);
            }
        }
    }
}


/**
 * @brief  示例2：按键消抖
 */
void example_key_debounce(void)
{
    printf("\n========== 示例2：按键消抖 ==========\n");
    printf("模拟按键信号：0=释放，1=按下\n");
    printf("机械抖动会导致在切换时出现短暂的错误状态\n\n");

    HysteresisFilter key_filter;
    hysteresis_init(&key_filter, 0.5f);  // 阈值0.5（对于0/1信号）

    // 模拟按键序列：按下时有抖动
    float key_sequence[] = {
        0, 0, 0,           // 初始释放状态
        1, 0, 1, 1, 1,     // 按下，有抖动
        1, 1, 1,           // 稳定按下
        0, 1, 0, 0, 0,     // 释放，有抖动
        0, 0, 0            // 稳定释放
    };

    printf("时间 | 原始信号 | 滤波输出 | 状态\n");
    printf("-----|----------|----------|----------\n");

    for (int i = 0; i < sizeof(key_sequence)/sizeof(float); i++) {
        float filtered = hysteresis_update(&key_filter, key_sequence[i]);
        printf(" %2d  |    %d     |    %d     | %s\n",
               i,
               (int)key_sequence[i],
               (int)filtered,
               (int)filtered ? "按下" : "释放");
    }

    printf("\n可以看到，滤波后的输出消除了抖动，状态切换清晰稳定！\n");
}


/**
 * @brief  示例3：温度控制系统
 */
void example_temperature_control(void)
{
    printf("\n========== 示例3：温度控制系统 ==========\n");
    printf("目标温度：26℃\n");
    printf("滞环阈值：1.0℃\n");
    printf("控制策略：温度超过27℃开启制冷，低于25℃关闭制冷\n\n");

    HysteresisFilter temp_filter;
    hysteresis_init(&temp_filter, 1.0f);

    // 模拟温度变化
    float temperatures[] = {
        26.0, 26.3, 26.7, 27.2,  // 温度上升
        27.0, 26.5, 26.0, 25.5,  // 制冷后下降
        25.0, 24.8, 25.2, 25.8   // 继续变化
    };

    printf("时间 | 实际温度 | 稳定温度 | 空调状态\n");
    printf("-----|----------|----------|----------\n");

    for (int i = 0; i < sizeof(temperatures)/sizeof(float); i++) {
        float stable_temp = hysteresis_update(&temp_filter, temperatures[i]);

        // 控制逻辑
        const char* ac_status;
        if (stable_temp > 27.0f) {
            ac_status = "制冷中";
        } else if (stable_temp < 25.0f) {
            ac_status = "已关闭";
        } else {
            ac_status = "保持";
        }

        printf(" %2d  |  %5.1f℃  |  %5.1f℃  | %s\n",
               i, temperatures[i], stable_temp, ac_status);
    }

    printf("\n滞环控制避免了在26℃附近频繁开关空调！\n");
}


/**
 * @brief  示例4：双阈值滞环应用
 */
void example_dual_threshold(void)
{
    printf("\n========== 示例4：双阈值滞环 ==========\n");
    printf("应用场景：电池电压监测\n");
    printf("上升阈值：0.2V（电压上升时较敏感）\n");
    printf("下降阈值：0.5V（电压下降时较迟钝，避免误报）\n\n");

    DualHysteresisFilter battery_filter;
    dual_hysteresis_init(&battery_filter, 0.2f, 0.5f);

    float voltages[] = {
        3.7, 3.8, 3.9, 4.0,  // 充电中
        4.0, 3.95, 3.92,     // 小幅下降（不告警）
        3.5, 3.3, 3.1        // 明显下降（告警）
    };

    printf("时间 | 实际电压 | 稳定电压 | 说明\n");
    printf("-----|----------|----------|------------------\n");

    for (int i = 0; i < sizeof(voltages)/sizeof(float); i++) {
        float stable = dual_hysteresis_update(&battery_filter, voltages[i]);

        const char* note;
        if (i == 0) {
            note = "初始化";
        } else if (fabs(stable - voltages[i]) < 0.01) {
            note = "更新";
        } else {
            note = "保持（在死区内）";
        }

        printf(" %2d  |  %4.2fV   |  %4.2fV   | %s\n",
               i, voltages[i], stable, note);
    }
}


/**
 * @brief  示例5：性能对比测试
 */
void example_performance_comparison(void)
{
    printf("\n========== 示例5：不同阈值的效果对比 ==========\n");

    // 创建三个不同阈值的滤波器
    HysteresisFilter filter_small, filter_medium, filter_large;
    hysteresis_init(&filter_small, 0.5f);   // 小阈值
    hysteresis_init(&filter_medium, 2.0f);  // 中阈值
    hysteresis_init(&filter_large, 5.0f);   // 大阈值

    printf("原始信号 | 阈值0.5 | 阈值2.0 | 阈值5.0\n");
    printf("---------|---------|---------|--------\n");

    // 生成带噪声的测试信号（基准值50，噪声±1）
    for (int i = 0; i < 15; i++) {
        float signal = generate_noisy_signal(50.0f, 1.0f);

        float out_small = hysteresis_update(&filter_small, signal);
        float out_medium = hysteresis_update(&filter_medium, signal);
        float out_large = hysteresis_update(&filter_large, signal);

        printf(" %6.2f  | %6.2f  | %6.2f  | %6.2f\n",
               signal, out_small, out_medium, out_large);
    }

    printf("\n观察：\n");
    printf("- 阈值0.5：输出变化频繁，抗噪能力弱\n");
    printf("- 阈值2.0：输出稳定，能滤除大部分噪声\n");
    printf("- 阈值5.0：输出几乎不变，可能丢失真实变化\n");
}


/* ============================================================================
 * 主函数：运行所有示例
 * ============================================================================ */

int main(void)
{
    printf("\n");
    printf("╔════════════════════════════════════════════════════════╗\n");
    printf("║           滞环滤波算法 - 完整示例程序                  ║\n");
    printf("║        Hysteresis Filter Implementation                ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");

    // 运行所有示例
    example_basic_filtering();
    example_key_debounce();
    example_temperature_control();
    example_dual_threshold();
    example_performance_comparison();

    printf("\n");
    printf("╔════════════════════════════════════════════════════════╗\n");
    printf("║                    总结与建议                          ║\n");
    printf("╠════════════════════════════════════════════════════════╣\n");
    printf("║ 1. 阈值建议：2~3倍噪声幅度                            ║\n");
    printf("║ 2. 最佳应用：按键消抖、开关控制                       ║\n");
    printf("║ 3. 注意事项：会丢失小于阈值的真实变化                 ║\n");
    printf("║ 4. 计算成本：极低（仅一次减法+一次比较）              ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");
    printf("\n");

    return 0;
}


/* ============================================================================
 * 补充说明
 * ============================================================================
 *
 * 1. 编译命令：
 *    gcc 07_滞环滤波.c -o hysteresis_filter -lm
 *
 * 2. 运行：
 *    ./hysteresis_filter
 *
 * 3. 移植到嵌入式系统：
 *    - 去掉所有printf语句
 *    - 只保留核心的滤波函数和结构体定义
 *    - 如果内存紧张，可以使用基础版本（静态变量）
 *
 * 4. 参数调节技巧：
 *    a) 先测量噪声幅度（在稳定状态下采集100个样本）
 *    b) 设置阈值为噪声幅度的2~3倍
 *    c) 实际测试并根据效果微调
 *
 * 5. 常见问题：
 *    Q: 输出长时间不变？
 *    A: 阈值设置过大，减小阈值
 *
 *    Q: 输出仍然抖动？
 *    A: 阈值设置过小或噪声超预期，增大阈值
 *
 *    Q: 响应太慢？
 *    A: 滞环滤波不会产生延迟，可能是采样频率太低
 *
 * 6. 高级技巧：
 *    - 可以与均值滤波组合使用
 *    - 可以实现自适应阈值（根据噪声水平动态调整）
 *    - 可以为不同方向的变化设置不同阈值（双阈值版本）
 *
 * ============================================================================
 */
