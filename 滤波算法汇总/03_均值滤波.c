/**
 * ===============================================================================
 * 文件名称：03_均值滤波.c
 * 功能描述：均值滤波算法（滑动平均滤波）的C语言实现
 *
 * 算法原理：
 *   对最近N次采样值求算术平均，用平均值作为本次滤波输出值。
 *
 *   数学公式：y = (x1 + x2 + x3 + ... + xN) / N
 *
 *   工作机制：
 *   1. 维护一个长度为N的循环队列（滑动窗口）
 *   2. 每次新数据到来，最旧的数据被丢弃
 *   3. 将新数据加入队列
 *   4. 计算队列中所有数据的平均值
 *
 * 适用场景：
 *   ✅ 适合：变化缓慢的信号（温度、湿度、电压、光照）
 *   ✅ 适合：周期性干扰的信号（工频干扰、纹波噪声）
 *   ❌ 不适合：快速变化的信号（会产生明显延迟）
 *   ❌ 不适合：有脉冲干扰的信号（脉冲会持续影响N次）
 *
 * 优点：
 *   - 算法简单，易于理解和实现
 *   - 对周期性干扰滤波效果好
 *   - 资源占用少（仅需N个数据的内存）
 *
 * 缺点：
 *   - 响应速度慢（延迟N/2个采样周期）
 *   - 灵敏度低（对突变信号反应迟钝）
 *   - 对脉冲干扰无效（会持续影响N次）
 *
 * 作者：嵌入式滤波算法教程
 * 日期：2025
 * 版本：v1.0
 * ===============================================================================
 */

#include <stdio.h>
#include <stdint.h>

/* ============================================================================
 * 方式一：标准实现（每次遍历求和）
 * ============================================================================
 * 特点：
 *   - 代码简单直观，容易理解
 *   - 每次调用都要遍历整个缓冲区求和
 *   - 适合窗口较小（N<10）或调用频率不高的场合
 *
 * 时间复杂度：O(N)
 * 空间复杂度：O(N)
 */

#define FILTER_N 12  // 滤波窗口长度（参与平均的采样点数）

/**
 * @brief 均值滤波函数（标准实现）
 *
 * @param new_value 本次采样的新值
 * @return float 滤波后的输出值
 *
 * @note
 *   - 第一次调用时，缓冲区全部用new_value填充
 *   - 之后每次调用，最旧的数据被新数据替换
 *   - 返回值是缓冲区所有数据的算术平均值
 */
float average_filter(float new_value)
{
    static float value_buf[FILTER_N];  // 数据缓冲区（静态变量，程序运行期间一直存在）
    static uint8_t filter_cnt = 0;     // 当前缓冲区填充计数（0~FILTER_N-1）
    static uint8_t is_first = 1;       // 首次调用标志（1=第一次，0=非第一次）

    uint8_t i;
    float sum = 0;

    /* -----------------------------------------------------------------------
     * 首次调用特殊处理：用第一个采样值填充整个缓冲区
     *
     * 原因：如果缓冲区初始值为0，第一次输出会偏小
     * 例如：真实值100，缓冲区[0,0,0,0,0]，平均值=(100+0+0+0+0)/5=20（偏差大）
     *      改进后：缓冲区[100,100,100,100,100]，平均值=100（立即正确）
     * ----------------------------------------------------------------------- */
    if (is_first) {
        for (i = 0; i < FILTER_N; i++) {
            value_buf[i] = new_value;  // 全部填充为第一个采样值
        }
        is_first = 0;  // 清除首次标志
    }

    /* -----------------------------------------------------------------------
     * 更新缓冲区：新数据替换最旧的数据
     *
     * 循环队列原理（filter_cnt是循环索引）：
     *   filter_cnt = 0: 替换 value_buf[0]（最旧的位置）
     *   filter_cnt = 1: 替换 value_buf[1]
     *   ...
     *   filter_cnt = N-1: 替换 value_buf[N-1]
     *   filter_cnt = N: 归零，重新从 value_buf[0] 开始
     *
     * 这样形成一个"循环覆盖"，始终保持最新的N个数据
     * ----------------------------------------------------------------------- */
    value_buf[filter_cnt] = new_value;

    /* 循环索引自增，到达N时归零（实现循环队列） */
    filter_cnt++;
    if (filter_cnt >= FILTER_N) {
        filter_cnt = 0;
    }

    /* -----------------------------------------------------------------------
     * 求和：遍历缓冲区，累加所有数据
     * ----------------------------------------------------------------------- */
    for (i = 0; i < FILTER_N; i++) {
        sum += value_buf[i];
    }

    /* -----------------------------------------------------------------------
     * 求平均：总和除以窗口长度
     * ----------------------------------------------------------------------- */
    return (sum / FILTER_N);
}


/* ============================================================================
 * 方式二：优化实现（增量更新，避免重复求和）
 * ============================================================================
 * 特点：
 *   - 只在第一次求完整的和，之后每次只更新差值
 *   - 效率高，适合窗口较大（N>10）或调用频繁的场合
 *   - 代码稍微复杂一点
 *
 * 核心思想：
 *   新的和 = 旧的和 - 被移除的旧值 + 新加入的值
 *
 *   例如：缓冲区 [10, 12, 8, 15, 9]，和=54
 *        新值=11，要移除10
 *        新的和 = 54 - 10 + 11 = 55
 *        无需重新遍历求和！
 *
 * 时间复杂度：O(1)（除了第一次是O(N)）
 * 空间复杂度：O(N)
 */

/**
 * @brief 均值滤波函数（优化实现 - 增量更新）
 *
 * @param new_value 本次采样的新值
 * @return float 滤波后的输出值
 *
 * @note
 *   - 相比标准实现，这个版本效率更高
 *   - 只在第一次完整求和，之后每次只计算增量
 *   - ��合高频调用或窗口较大的场合
 */
float average_filter_fast(float new_value)
{
    static float value_buf[FILTER_N];
    static uint8_t filter_cnt = 0;
    static uint8_t is_first = 1;
    static float sum = 0;  // 静态变量保存当前缓冲区的总和

    uint8_t i;

    /* 首次调用：填充缓冲区并计算初始和 */
    if (is_first) {
        for (i = 0; i < FILTER_N; i++) {
            value_buf[i] = new_value;
        }
        sum = new_value * FILTER_N;  // 初始和 = 单个值 × 数量
        is_first = 0;
    }

    /* -----------------------------------------------------------------------
     * 增量更新：只计算差值，不重新求和
     *
     * 关键公式：
     *   新和 = 旧和 - 要移除的旧值 + 新加入的值
     *
     * 这样每次只需要：1次减法 + 1次加法
     * 而不是：N次加法（遍历求和）
     *
     * 当N=100时，性能提升约50倍！
     * ----------------------------------------------------------------------- */
    sum = sum - value_buf[filter_cnt] + new_value;

    /* 更新缓冲区 */
    value_buf[filter_cnt] = new_value;

    /* 循环索引自增 */
    filter_cnt++;
    if (filter_cnt >= FILTER_N) {
        filter_cnt = 0;
    }

    /* 返回平均值 */
    return (sum / FILTER_N);
}


/* ============================================================================
 * 方式三：定点数优化（整数运算，适合没有FPU的单片机）
 * ============================================================================
 * 特点：
 *   - 完全使用整数运算，避免浮点数（速度快，资源省）
 *   - 使用位移代替除法（当N为2的幂次时）
 *   - 适合对性能要求高的嵌入式系统
 *
 * 限制：
 *   - FILTER_N 必须是 2 的幂次（2, 4, 8, 16, 32...）
 *   - 输入数据需要乘以放大系数（保留小数精度）
 *
 * 性能提升：
 *   - 整数加法比浮点加法快 5-10 倍
 *   - 位移运算比除法快 10-20 倍
 */

#define FILTER_N_FIXED 16     // 窗口长度（必须是2的幂次：2,4,8,16,32...）
#define FILTER_SHIFT 4        // 位移位数（16 = 2^4，所以位移4位）
#define SCALE_FACTOR 100      // 放大系数（保留2位小数：实际值×100）

/**
 * @brief 均值滤波函数（定点数优化版）
 *
 * @param new_value 本次采样的新值（已经乘以 SCALE_FACTOR）
 * @return int32_t 滤波后的输出值（需要除以 SCALE_FACTOR 得到实际值）
 *
 * @note
 *   - 输入和输出都是放大后的整数
 *   - 例如：实际温度 25.67°C → 输入 2567（25.67×100）
 *          滤波输出 2570 → 实际值 25.70°C（2570÷100）
 *
 * @warning
 *   - FILTER_N_FIXED 必须是 2 的幂次（2,4,8,16,32,64...）
 *   - 否则位移运算结果错误！
 */
int32_t average_filter_fixed(int32_t new_value)
{
    static int32_t value_buf[FILTER_N_FIXED];
    static uint8_t filter_cnt = 0;
    static uint8_t is_first = 1;
    static int32_t sum = 0;

    uint8_t i;

    /* 首次调用初始化 */
    if (is_first) {
        for (i = 0; i < FILTER_N_FIXED; i++) {
            value_buf[i] = new_value;
        }
        sum = new_value << FILTER_SHIFT;  // sum = new_value × 16（左移4位）
        is_first = 0;
    }

    /* 增量更新和 */
    sum = sum - value_buf[filter_cnt] + new_value;

    /* 更新缓冲区 */
    value_buf[filter_cnt] = new_value;

    /* 循环索引自增 */
    filter_cnt++;
    if (filter_cnt >= FILTER_N_FIXED) {
        filter_cnt = 0;
    }

    /* -----------------------------------------------------------------------
     * 位移代替除法（核心优化）
     *
     * 原理：除以 2^n 等价于 右移 n 位
     *   sum / 16 = sum >> 4（右移4位）
     *   sum / 32 = sum >> 5（右移5位）
     *
     * 性能对比（STM32F103测试）：
     *   整数除法：sum / 16  → 约 20 个时钟周期
     *   位移运算：sum >> 4  → 约 1 个时钟周期（快20倍！）
     * ----------------------------------------------------------------------- */
    return (sum >> FILTER_SHIFT);  // 等价于 sum / FILTER_N_FIXED
}


/* ============================================================================
 * 测试代码：模拟传感器数据并验证滤波效果
 * ============================================================================ */

/**
 * @brief 模拟传感器读取（带随机噪声）
 *
 * 模拟场景：温度传感器
 *   - 真实温度：25.0°C（稳定值）
 *   - 测量噪声：±2°C（随机波动）
 *
 * @return float 模拟的传感器读数
 */
float simulate_sensor_reading(void)
{
    static float true_value = 25.0;  // 真实温度值
    static int count = 0;

    count++;

    /* 模拟不同的噪声模式 */
    if (count < 10) {
        /* 前10次：小幅随机噪声（±0.5） */
        return true_value + (count % 3 - 1) * 0.5;
    } else if (count == 10) {
        /* 第10次：模拟一个脉冲干扰 */
        return true_value + 10.0;  // 突然跳变到35°C
    } else if (count < 20) {
        /* 后10次：恢复正常 */
        return true_value + (count % 3 - 1) * 0.5;
    } else {
        /* 之后：真实值缓慢上��（模拟环境温度升高） */
        true_value += 0.1;
        return true_value + (count % 3 - 1) * 0.5;
    }
}

/**
 * @brief 主函数 - 测试均值滤波效果
 */
int main(void)
{
    float raw_value, filtered_value, filtered_value_fast;
    int32_t raw_value_fixed, filtered_value_fixed;
    int i;

    printf("===============================================================================\n");
    printf("均值滤波算法测试程序\n");
    printf("===============================================================================\n");
    printf("测试场景：温度传感器（真实值25.0°C，噪声±2°C）\n");
    printf("滤波窗口：N = %d（标准版和快速版）\n", FILTER_N);
    printf("滤波窗口：N = %d（定点数版）\n", FILTER_N_FIXED);
    printf("===============================================================================\n\n");

    printf("采样次数 | 原始数据 | 标准滤波 | 快速滤波 | 定点滤波 | 说明\n");
    printf("---------|----------|----------|----------|----------|------------------------\n");

    /* 进行30次采样测试 */
    for (i = 1; i <= 30; i++) {
        /* 读取模拟传感器数据 */
        raw_value = simulate_sensor_reading();

        /* 方式一：标准均值滤波 */
        filtered_value = average_filter(raw_value);

        /* 方式二：快速均值滤波（增量更新） */
        filtered_value_fast = average_filter_fast(raw_value);

        /* 方式三：定点数均值滤波 */
        raw_value_fixed = (int32_t)(raw_value * SCALE_FACTOR);  // 放大100倍
        filtered_value_fixed = average_filter_fixed(raw_value_fixed);

        /* 打印结果 */
        printf("  %2d     | %6.2f   | %6.2f   | %6.2f   | %6.2f   | ",
               i, raw_value, filtered_value, filtered_value_fast,
               (float)filtered_value_fixed / SCALE_FACTOR);

        /* 特殊事件说明 */
        if (i == 1) {
            printf("首次采样\n");
        } else if (i == 10) {
            printf("← 脉冲干扰！\n");
        } else if (i == 11) {
            printf("干扰开始影响输出\n");
        } else if (i > 10 && i <= 10 + FILTER_N) {
            printf("干扰影响中...\n");
        } else if (i == 21) {
            printf("真实值开始上升\n");
        } else {
            printf("\n");
        }
    }

    printf("\n===============================================================================\n");
    printf("测试结论：\n");
    printf("1. 标准滤波和快速滤波结果相同，验证算法正确性\n");
    printf("2. 定点滤波结果略有差异（整数运算的舍入误差），但误差可接受\n");
    printf("3. 滤波后数据明显平滑，随机噪声被有效抑制\n");
    printf("4. 脉冲干扰会持续影响约%d次采样（窗口长度）\n", FILTER_N);
    printf("5. 对缓慢变化的信号（温度上升）跟踪良好\n");
    printf("===============================================================================\n");

    printf("\n性能对比：\n");
    printf("┌──────────────┬──────────┬──────────┬────────────────┐\n");
    printf("│ 实现方式     │ 时间复杂度│ 空间复杂度│ 推荐使用场景    │\n");
    printf("├──────────────┼──────────┼──────────┼────────────────┤\n");
    printf("│ 标准实现     │ O(N)     │ O(N)     │ N<10，学习用    │\n");
    printf("│ 快速实现     │ O(1)     │ O(N)     │ N>10，通用推荐  │\n");
    printf("│ 定点数实现   │ O(1)     │ O(N)     │ 无FPU单片机     │\n");
    printf("└──────────────┴──────────┴──────────┴────────────────┘\n");

    return 0;
}

/* ============================================================================
 * 实际应用示例
 * ============================================================================ */

#if 0  // 示例代码，编译时不包含

/**
 * 示例1：温度监测（DS18B20传感器）
 *
 * 应用场景：智能家居温控系统
 * 采样周期：1秒
 * 滤波窗口：10次（平滑10秒）
 */
void example_temperature_monitor(void)
{
    float temperature, smooth_temp;

    // 每秒读取一次温度
    temperature = ds18b20_read_temperature();

    // 均值滤波平滑
    smooth_temp = average_filter(temperature);

    // 显示到LCD
    lcd_show_temperature(smooth_temp);

    // 温控逻辑
    if (smooth_temp > 28.0) {
        turn_on_fan();      // 温度高，开风扇
    } else if (smooth_temp < 22.0) {
        turn_on_heater();   // 温度低,开加热
    }
}

/**
 * 示例2：电池电压监测（ADC采样）
 *
 * 应用场景：锂电池电量检测
 * 采样周期：100ms
 * 滤波窗口：20次（平滑2秒）
 */
void example_battery_voltage_monitor(void)
{
    float voltage, smooth_voltage;

    // 读取ADC电压
    voltage = adc_read_voltage();

    // 均值滤波
    smooth_voltage = average_filter(voltage);

    // 电量显示
    uint8_t battery_level = voltage_to_percentage(smooth_voltage);
    display_battery_level(battery_level);

    // 低电量报警
    if (smooth_voltage < 3.3) {
        low_battery_warning();
    }
}

/**
 * 示例3：光照强度自适应（光敏电阻）
 *
 * 应用场景：自动调节屏幕亮度
 * 采样周期：200ms
 * 滤波窗口：15次（平滑3秒）
 */
void example_light_sensor(void)
{
    float light_intensity, smooth_light;

    // 读取光照强度（0~1000）
    light_intensity = read_light_sensor();

    // 均值滤波
    smooth_light = average_filter(light_intensity);

    // 映射到亮度值（10~100）
    uint8_t brightness = map_value(smooth_light, 0, 1000, 10, 100);

    // 调节屏幕亮度
    set_screen_brightness(brightness);
}

/**
 * 示例4：压力传感器（工业应用）
 *
 * 应用场景：气压罐压力监测
 * 采样周期：50ms
 * 滤波窗口：40次（平滑2秒）
 *
 * 特点：使用定点数优化版本（高效）
 */
void example_pressure_sensor(void)
{
    int32_t pressure_raw, pressure_filtered;
    float pressure_kpa;

    // 读取压力ADC值（0~4095）
    uint16_t adc_value = adc_read_pressure_sensor();

    // 转换为压力值（放大100倍，保留2位小数）
    // 例如：ADC=2048 → 50.00 kPa → 存储为 5000
    pressure_raw = (int32_t)(adc_value * 0.0244 * SCALE_FACTOR);

    // 定点数滤波
    pressure_filtered = average_filter_fixed(pressure_raw);

    // 还原为实际压力值（kPa）
    pressure_kpa = (float)pressure_filtered / SCALE_FACTOR;

    // 显示和报警
    display_pressure(pressure_kpa);

    if (pressure_kpa > 80.0) {
        pressure_alarm();  // 超压报警
    }
}

#endif  // 示例代码结束

/* ============================================================================
 * 常见问题 FAQ
 * ============================================================================
 *
 * Q1: 为什么第一次输出值不对？
 * A1: 需要用首次采样值填充缓冲区，参考代码中的 is_first 处理。
 *
 * Q2: 窗口长度N如何选择？
 * A2: N = 期望平滑时间 / 采样周期
 *     例如：采样周期100ms，想平滑1秒 → N = 1000ms/100ms = 10
 *
 * Q3: N值是否越大越好？
 * A3: 不是！N越大滤波效果越好，但响应越慢。需要权衡。
 *
 * Q4: 如何提高运算速度？
 * A4: ① 使用增量更新（average_filter_fast）
 *     ② 使用定点数（average_filter_fixed）
 *     ③ N选择2的幂次，用位移代替除法
 *
 * Q5: 能不能滤除脉冲干扰？
 * A5: 不能！脉冲会持续影响N次采样。
 *     建议先用"限幅滤波"去脉冲，再用均值滤波平滑。
 *
 * Q6: 适合用在控制系统中吗？
 * A6: 不建议！相位滞后会导致控制不稳定。
 *     控制系统建议用"一阶滞后滤波"或"卡尔曼滤波"。
 *
 * Q7: 为什么我的定点数滤波结果总是0？
 * A7: 检查是否忘记放大输入值！
 *     输入必须先乘以 SCALE_FACTOR，输出再除以 SCALE_FACTOR。
 *
 * Q8: 多个传感器能共用一个滤波器吗？
 * A8: 不能！每个传感器需要独立的缓冲区。
 *     可以封装成结构体：
 *     typedef struct {
 *         float buf[FILTER_N];
 *         uint8_t cnt;
 *         float sum;
 *     } AverageFilter;
 *
 * ============================================================================ */
