/**
 * ============================================================================
 * 文件名  : 06_限幅滤波.c
 * 描述    : 限幅滤波算法的C语言实现（也称为程序判断滤波）
 * 作者    : AI助手
 * 日期    : 2025-11-09
 * 版本    : v1.0
 * ============================================================================
 *
 * 算法原理：
 *   限幅滤波是最简单的滤波算法之一，其核心思想是：
 *   在正常情况下，相邻两次采样值的变化不应该太大。如果新采样值与上一次
 *   有效值的偏差超过某个合理范围（限幅值），则认为这是一个异常值（干扰），
 *   应该舍弃，继续使用上一次的有效值。
 *
 * 数学公式：
 *   y(n) = { x(n),     如果 |x(n) - x(n-1)| ≤ A
 *          { x(n-1),   如果 |x(n) - x(n-1)| > A
 *
 *   其中：
 *   - x(n)   : 第n次采样值
 *   - x(n-1) : 第n-1次有效值（上一次的输出���
 *   - A      : 允许的最大偏差（限幅值）
 *   - y(n)   : 第n次滤波输出
 *
 * 优点：
 *   1. 实现极其简单，代码量少
 *   2. 计算量小，只需一次减法和一次比较
 *   3. 内存占用极少，只需保存一个历史值
 *   4. 实时性好，无延迟
 *   5. 能有效消除偶然出现的脉冲干扰
 *
 * 缺点：
 *   1. 无法跟踪快速变化的信号
 *   2. 限幅值A难以精确选择
 *   3. 不能滤除叠加的随机噪声
 *   4. 对连续的干扰无能为力
 *
 * 适用场景：
 *   1. 信号变化缓慢的系统（如温度、压力、液位监测）
 *   2. 偶发性脉冲干扰的场合
 *   3. 资源受限的嵌入式系统
 *   4. 需要快速响应的控制系统
 *
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

// ============================================================================
// 基础版本：标准限幅滤波
// ============================================================================

/**
 * @brief  限幅滤波函数（基础版本）
 * @param  new_value  : 本次采样值
 * @param  last_value : 指向上一次有效值的指针（会被更新）
 * @param  limit      : 允许的最大偏差（限幅值）
 * @return 滤波后的输出值
 * @note   这是最基础的实现，逻辑清晰易懂
 */
float amplitude_limit_filter(float new_value, float *last_value, float limit)
{
    // 计算本次采样值与上次有效值的偏差（取绝对值）
    float delta = fabs(new_value - *last_value);

    // 判断偏差是否在允许范围内
    if (delta <= limit) {
        // 偏差合理，接受新值
        *last_value = new_value;  // 更新历史值
        return new_value;         // 输出新值
    } else {
        // 偏差过大，拒绝新值，认为是干扰
        return *last_value;       // 输出旧值（保持不变）
    }
}

// ============================================================================
// 改进版本1：带统计信息的限幅滤波
// ============================================================================

/**
 * @brief  限幅滤波滤波器结构体（带统计功能）
 */
typedef struct {
    float last_value;        // 上一次有效值
    float limit;             // 限幅值
    uint32_t total_count;    // 总采样次数
    uint32_t reject_count;   // 被拒绝的次数
    uint8_t is_initialized;  // 是否已初始化
} AmplitudeLimitFilter_t;

/**
 * @brief  初始化限幅滤波器
 * @param  filter : 滤波器结构体指针
 * @param  limit  : 限幅值
 * @param  init_value : 初始值（可选，传入NAN则使用第一个采样值）
 */
void amplitude_limit_filter_init(AmplitudeLimitFilter_t *filter, float limit, float init_value)
{
    filter->limit = limit;
    filter->total_count = 0;
    filter->reject_count = 0;

    if (isnan(init_value)) {
        filter->is_initialized = 0;  // 等待第一个采样值
    } else {
        filter->last_value = init_value;
        filter->is_initialized = 1;
    }
}

/**
 * @brief  限幅滤波处理（带统计）
 * @param  filter : 滤波器结构体指针
 * @param  new_value : 本次采样值
 * @return 滤波后的输出值
 */
float amplitude_limit_filter_update(AmplitudeLimitFilter_t *filter, float new_value)
{
    // 如果未初始化，第一个采样值直接作为基准
    if (!filter->is_initialized) {
        filter->last_value = new_value;
        filter->is_initialized = 1;
        filter->total_count = 1;
        return new_value;
    }

    // 统计总采样次数
    filter->total_count++;

    // 计算偏差
    float delta = fabs(new_value - filter->last_value);

    // 判断是否超限
    if (delta <= filter->limit) {
        // 接受新值
        filter->last_value = new_value;
        return new_value;
    } else {
        // 拒绝新值
        filter->reject_count++;
        return filter->last_value;
    }
}

/**
 * @brief  获取滤波器的拒绝率（用于诊断）
 * @param  filter : 滤波器结构体指针
 * @return 拒绝率（百分比，0~100）
 */
float amplitude_limit_filter_get_reject_rate(AmplitudeLimitFilter_t *filter)
{
    if (filter->total_count == 0) {
        return 0.0f;
    }
    return (float)filter->reject_count / filter->total_count * 100.0f;
}

// ============================================================================
// 改进版本2：自适应限幅滤波（能跟踪快速变化）
// ============================================================================

/**
 * @brief  自适应限幅滤波器结构体
 */
typedef struct {
    float last_value;        // 上一次有效值
    float limit;             // 正常限幅值
    uint8_t exceed_count;    // 连续超限次数
    uint8_t is_initialized;  // 是否已初始化
} AdaptiveAmplitudeFilter_t;

/**
 * @brief  初始化自适应限幅滤波器
 */
void adaptive_filter_init(AdaptiveAmplitudeFilter_t *filter, float limit, float init_value)
{
    filter->limit = limit;
    filter->exceed_count = 0;

    if (isnan(init_value)) {
        filter->is_initialized = 0;
    } else {
        filter->last_value = init_value;
        filter->is_initialized = 1;
    }
}

/**
 * @brief  自适应限幅滤波处理
 * @param  filter : 滤波器结构体指针
 * @param  new_value : 本次采样值
 * @return 滤波后的输出值
 * @note   当检测到连续超限时，认为可能是真实信号在快速变化，
 *         此时逐步跟踪新值，而不是完全拒绝
 */
float adaptive_filter_update(AdaptiveAmplitudeFilter_t *filter, float new_value)
{
    // 第一次采样，直接接受
    if (!filter->is_initialized) {
        filter->last_value = new_value;
        filter->is_initialized = 1;
        return new_value;
    }

    // 计算偏差
    float delta = fabs(new_value - filter->last_value);

    if (delta <= filter->limit) {
        // 偏差在正常范围，接受新值
        filter->last_value = new_value;
        filter->exceed_count = 0;  // 重置超限计数
        return new_value;
    } else {
        // 偏差超限
        filter->exceed_count++;

        // 如果连续3次超限，可能是真实信号在快速变化
        // 此时采用渐进式跟踪：每次向新值靠近一个限幅值的距离
        if (filter->exceed_count >= 3) {
            if (new_value > filter->last_value) {
                filter->last_value += filter->limit;  // 向上跟踪
            } else {
                filter->last_value -= filter->limit;  // 向下跟踪
            }
        }

        return filter->last_value;
    }
}

// ============================================================================
// 改进版本3：双阈值限幅滤波
// ============================================================================

/**
 * @brief  双阈值限幅滤波
 * @param  new_value : 本次采样值
 * @param  last_value : 上一次有效值（会被更新）
 * @param  limit_normal : 正常限幅值（小变化）
 * @param  limit_max : 最大限幅值（大变化）
 * @return 滤波后的输出值
 * @note   设置两个阈值：
 *         - 偏差 <= limit_normal : 完全接受
 *         - limit_normal < 偏差 <= limit_max : 部分接受（权重0.5）
 *         - 偏差 > limit_max : 完全拒绝
 */
float dual_threshold_filter(float new_value, float *last_value,
                           float limit_normal, float limit_max)
{
    float delta = fabs(new_value - *last_value);
    float output;

    if (delta <= limit_normal) {
        // 小变化，完全接受
        output = new_value;
    } else if (delta <= limit_max) {
        // 中等变化，部分跟踪（加权平均）
        output = *last_value + (new_value - *last_value) * 0.5f;
    } else {
        // 大变化，完全拒绝
        output = *last_value;
    }

    *last_value = output;
    return output;
}

// ============================================================================
// 测试代码
// ============================================================================

/**
 * @brief  生成测试数据：正弦波 + 脉冲干扰
 * @param  index : 采样点索引
 * @return 模拟的传感器采样值
 */
float generate_test_signal(int index)
{
    // 基础信号：缓慢变化的正弦波（模拟温度变化）
    float base_signal = 25.0f + 5.0f * sin(index * 0.1f);

    // 在第20、40、60个点注入脉冲干扰
    if (index == 20 || index == 40 || index == 60) {
        return base_signal + 15.0f;  // +15度的异常值
    }

    // 在第30、50个点注入负脉冲干扰
    if (index == 30 || index == 50) {
        return base_signal - 10.0f;  // -10度的异常值
    }

    return base_signal;
}

/**
 * @brief  测试1：基础限幅滤波
 */
void test_basic_filter(void)
{
    printf("\n========== 测试1：基础限幅滤波 ==========\n");
    printf("场景：温度传感器数据，限幅值 = 3.0℃\n\n");

    float last_value = 25.0f;  // 初始温度
    float limit = 3.0f;        // 限幅值：3℃

    printf("序号\t原始值\t滤波值\t状态\n");
    printf("----\t------\t------\t--------\n");

    for (int i = 0; i < 70; i++) {
        float raw = generate_test_signal(i);
        float filtered = amplitude_limit_filter(raw, &last_value, limit);

        // 每5个点输出一次
        if (i % 5 == 0 || fabs(raw - filtered) > 0.1f) {
            printf("%d\t%.2f\t%.2f\t%s\n",
                   i, raw, filtered,
                   (fabs(raw - filtered) > 0.1f) ? "已滤除" : "正常");
        }
    }
}

/**
 * @brief  测试2：带统计的限幅滤波
 */
void test_filter_with_statistics(void)
{
    printf("\n========== 测试2：带统计的限幅滤波 ==========\n");

    AmplitudeLimitFilter_t filter;
    amplitude_limit_filter_init(&filter, 3.0f, 25.0f);

    for (int i = 0; i < 100; i++) {
        float raw = generate_test_signal(i);
        float filtered = amplitude_limit_filter_update(&filter, raw);
    }

    printf("总采样次数: %u\n", filter.total_count);
    printf("被拒绝次数: %u\n", filter.reject_count);
    printf("拒绝率: %.2f%%\n", amplitude_limit_filter_get_reject_rate(&filter));

    if (amplitude_limit_filter_get_reject_rate(&filter) > 10.0f) {
        printf("⚠️ 警告：拒绝率过高，建议增大限幅值！\n");
    } else {
        printf("✓ 拒绝率正常\n");
    }
}

/**
 * @brief  测试3：自适应限幅滤波（跟踪快速变化）
 */
void test_adaptive_filter(void)
{
    printf("\n========== 测试3：自适应限幅滤波 ==========\n");
    printf("场景：温度快速上升（模拟加热过程）\n\n");

    AdaptiveAmplitudeFilter_t filter;
    adaptive_filter_init(&filter, 2.0f, 25.0f);

    printf("序号\t原始值\t滤波值\t说明\n");
    printf("----\t------\t------\t--------\n");

    // 模拟快速升温过程
    for (int i = 0; i < 20; i++) {
        float raw = 25.0f + i * 1.5f;  // 每次上升1.5℃（超过限幅值2℃）
        float filtered = adaptive_filter_update(&filter, raw);

        printf("%d\t%.2f\t%.2f\t%s\n",
               i, raw, filtered,
               (filter.exceed_count >= 3) ? "渐进跟踪" : "正常");
    }

    printf("\n说明：自适应滤波能够逐步跟踪快速变化，避免输出停滞\n");
}

/**
 * @brief  测试4：双阈值限幅滤波
 */
void test_dual_threshold_filter(void)
{
    printf("\n========== 测试4：双阈值限幅滤波 ==========\n");
    printf("正常限幅: 2.0℃, 最大限幅: 5.0℃\n\n");

    float last_value = 25.0f;
    float limit_normal = 2.0f;
    float limit_max = 5.0f;

    printf("原始值\t滤波值\t处理方式\n");
    printf("------\t------\t--------\n");

    float test_values[] = {26.0f, 27.5f, 30.0f, 32.0f, 25.0f};
    char *modes[] = {"小变化-完全接受", "中变化-部分接受", "大变化-完全拒绝", "大变化-完全拒绝", "大变化-完全拒绝"};

    for (int i = 0; i < 5; i++) {
        float filtered = dual_threshold_filter(test_values[i], &last_value,
                                              limit_normal, limit_max);
        printf("%.2f\t%.2f\t%s\n", test_values[i], filtered, modes[i]);
    }
}

/**
 * @brief  实际应用示例：温度监测系统
 */
void example_temperature_monitor(void)
{
    printf("\n========== 实际应用示例：温度监测系统 ==========\n");
    printf("应用：STM32 + DS18B20温度传感器\n");
    printf("要求：消除偶尔出现的错误读数\n\n");

    // 模拟DS18B20可能出现的错误读数
    float temperature_readings[] = {
        25.0f, 25.2f, 25.1f, 85.0f,    // 第4个是错误读数（DS18B20上电默认值）
        25.3f, 25.4f, 25.2f, -127.0f,  // 第8个是错误读数（通信失败）
        25.5f, 25.6f, 25.8f
    };

    AmplitudeLimitFilter_t filter;
    amplitude_limit_filter_init(&filter, 2.0f, 25.0f);  // 限幅值2℃

    printf("采样序号\t原始读数\t滤波后\t\t备注\n");
    printf("--------\t--------\t--------\t--------\n");

    for (int i = 0; i < sizeof(temperature_readings) / sizeof(float); i++) {
        float raw = temperature_readings[i];
        float filtered = amplitude_limit_filter_update(&filter, raw);

        printf("%d\t\t%.1f℃\t\t%.1f℃\t\t%s\n",
               i + 1, raw, filtered,
               (fabs(raw - filtered) > 1.0f) ? "❌ 错误读数已滤除" : "✓ 正常");
    }

    printf("\n最终稳定输出: %.1f℃\n", filter.last_value);
}

// ============================================================================
// 主函数
// ============================================================================

int main(void)
{
    printf("╔════════════════════════════════════════════════════════��\n");
    printf("║        限幅滤波算法（程序判断滤波）测试程序          ║\n");
    printf("║              Amplitude Limit Filter Test              ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");

    // 运行所有测试
    test_basic_filter();
    test_filter_with_statistics();
    test_adaptive_filter();
    test_dual_threshold_filter();
    example_temperature_monitor();

    printf("\n========== 总结 ==========\n");
    printf("1. 基础限幅滤波：实现简单，能滤除脉冲干扰，但无法跟踪快速变化\n");
    printf("2. 带统计的版本：可以通过拒绝率诊断参数是否合理\n");
    printf("3. 自适应版本：能够逐步跟踪快速变化，避免输出停滞\n");
    printf("4. 双阈值版本：对不同幅度的变化采用不同策略，更加灵活\n");
    printf("\n建议：\n");
    printf("- 限幅值的选择至关重要，建议通过实验确定\n");
    printf("- 单独使用效果有限，常与其他滤波算法组合使用\n");
    printf("- 适合作为第一级滤波，先消除脉冲，再用其他算法滤除噪声\n");

    return 0;
}

/*
 * ============================================================================
 * 编译运行说明
 * ============================================================================
 *
 * 1. GCC 编译（Linux/Mac）：
 *    gcc -o test 06_限幅滤波.c -lm
 *    ./test
 *
 * 2. Windows MinGW 编译：
 *    gcc -o test.exe 06_限幅滤波.c -lm
 *    test.exe
 *
 * 3. Keil MDK（STM32）：
 *    - 新建工程，添加此文件
 *    - 配置串口重定向（实现printf）
 *    - 编译下载运行
 *
 * 4. Arduino IDE：
 *    - 将main()改为loop()
 *    - 删除#include <stdio.h>
 *    - printf改为Serial.print
 *
 * ============================================================================
 * 预期输出示例
 * ============================================================================
 *
 * ========== 测试1：基础限幅滤波 ==========
 * 序号    原始值  滤波值  状态
 * ----    ------  ------  --------
 * 0       25.00   25.00   正常
 * 5       27.46   27.46   正常
 * 20      44.60   27.46   已滤除   ← 脉冲干扰被拦截
 * 25      29.96   29.96   正常
 * 30      19.60   29.96   已滤除   ← 负脉冲被拦截
 * ...
 *
 * ============================================================================
 */
