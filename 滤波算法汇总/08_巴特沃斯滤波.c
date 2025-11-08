/**
 * =====================================================================
 * 二阶巴特沃斯低通滤波器（Butterworth Low-Pass Filter）
 * =====================================================================
 *
 * 功能说明：
 *   实现二阶巴特沃斯低通滤波器，用于去除高频噪声，保留低频有用信号。
 *   特点是通带内频率响应最平坦，无波纹。
 *
 * 适用场景：
 *   - 传感器信号滤波（温度、压力、加速度等）
 *   - 电源电压滤波
 *   - 音频信号处理
 *   - PID控制器输入滤波
 *
 * 作者：AI助手
 * 日期：2025-11-09
 * =====================================================================
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// 数学常数
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 二阶巴特沃斯滤波器数据结构
 *
 * 存储滤波器的系数和历史数据
 */
typedef struct {
    // 滤波器系数
    float a0, a1, a2;      // 前向系数（输入项）
    float b1, b2;          // 反馈系数（输出项）

    // 历史数据
    float x1, x2;          // 前两次的输入值 x[n-1], x[n-2]
    float y1, y2;          // 前两次的输出值 y[n-1], y[n-2]
} ButterworthFilter;


/**
 * @brief 初始化二阶巴特沃斯低通滤波器
 *
 * 功能：根据截止频率和采样频率计算滤波器系数
 *
 * 数学原理：
 *   使用双线性变换将模拟滤波器转换为数字滤波器
 *   传递函数：H(s) = ωc² / (s² + √2·ωc·s + ωc²)
 *
 * @param filter  滤波器结构体指针
 * @param fc      截止频率（Hz），低于此频率的信号通过，高于此频率的信号衰减
 * @param fs      采样频率（Hz），应满足 fs >= 10*fc（推荐值）
 *
 * @note 截止频率fc的选择：
 *       - 传感器信号：fc = 0.5~10Hz（根据信号变化速度）
 *       - 音频信号：fc = 几百Hz到几kHz
 *       - 原则：fc应介于有用信号频率和噪声频率之间
 */
void Butterworth_Init(ButterworthFilter* filter, float fc, float fs)
{
    // 参数有效性检查
    if (fc <= 0 || fs <= 0 || fc >= fs/2) {
        printf("错误：参数无效！fc必须在(0, fs/2)范围内\n");
        printf("当前：fc=%.2f Hz, fs=%.2f Hz\n", fc, fs);
        return;
    }

    // 推荐fs至少是fc的10倍
    if (fs < 10 * fc) {
        printf("警告：采样率偏低！建议 fs >= 10*fc，当前 fs/fc = %.2f\n", fs/fc);
    }

    // 第一步：计算角频率
    float wc = 2.0f * M_PI * fc;           // 截止角频率（rad/s）
    float T = 1.0f / fs;                    // 采样周期（s）
    float wd = wc * T;                      // 归一化角频率（无量纲）

    // 第二步：双线性变换的预畸变因子
    // 作用：补偿双线性变换引起的频率畸变
    float K = tanf(wd / 2.0f);

    // 第三步：二阶巴特沃斯的品质因数（固定值）
    // Q = 1/√2 ≈ 0.7071，这是巴特沃斯滤波器的特征参数
    float Q = 0.70710678118f;  // 1/sqrt(2)

    // 第四步：计算归一化系数
    // 用于将系数归一化，使得滤波器稳定
    float norm = 1.0f / (1.0f + K / Q + K * K);

    // 第五步：计算前向系数（输入项系数）
    // 差分方程：y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
    filter->a0 = K * K * norm;             // x[n]的系数
    filter->a1 = 2.0f * filter->a0;        // x[n-1]的系数
    filter->a2 = filter->a0;               // x[n-2]的系数

    // 第六步：计算反馈系数（输出项系数）
    filter->b1 = 2.0f * (K * K - 1.0f) * norm;      // y[n-1]的系数
    filter->b2 = (1.0f - K / Q + K * K) * norm;     // y[n-2]的系数

    // 第七步：清零历史数据（避免启动瞬态）
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;

    // 调试信息（可选）
    printf("巴特沃斯滤波器初始化完成\n");
    printf("  截止频率：%.2f Hz\n", fc);
    printf("  采样频率：%.2f Hz\n", fs);
    printf("  系数：a0=%.6f, a1=%.6f, a2=%.6f\n",
           filter->a0, filter->a1, filter->a2);
    printf("       b1=%.6f, b2=%.6f\n", filter->b1, filter->b2);
}


/**
 * @brief 巴特沃斯滤波器更新函数（处理一个新样本）
 *
 * 功能：输入一个新的采样值，输出滤波后的值
 *
 * 差分方程：
 *   y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
 *
 * 工作流程：
 *   1. 使用当前输入和历史数据计算输出
 *   2. 更新历史数据（为下次计算做准备）
 *
 * @param filter  滤波器结构体指针
 * @param input   当前输入值 x[n]
 * @return        滤波后的输出值 y[n]
 *
 * @note 此函数应在固定的时间间隔调用，间隔 = 1/fs
 */
float Butterworth_Update(ButterworthFilter* filter, float input)
{
    // 第一步：根据差分方程计算输出
    // y[n] = 输入项 - 反馈项
    float output = filter->a0 * input           // 当前输入
                 + filter->a1 * filter->x1      // 上一次输入
                 + filter->a2 * filter->x2      // 上上次输入
                 - filter->b1 * filter->y1      // 上一次输出（反馈）
                 - filter->b2 * filter->y2;     // 上上次输出（反馈）

    // 第二步：更新历史输入数据（移位操作）
    filter->x2 = filter->x1;    // x[n-2] = x[n-1]
    filter->x1 = input;         // x[n-1] = x[n]

    // 第三步：更新历史输出数据（移位操作）
    filter->y2 = filter->y1;    // y[n-2] = y[n-1]
    filter->y1 = output;        // y[n-1] = y[n]

    return output;
}


/**
 * @brief 重置滤波器（清零历史数据）
 *
 * 适用场景：
 *   - 重新开始滤波
 *   - 检测到信号异常需要重置
 *
 * @param filter  滤波器结构体指针
 */
void Butterworth_Reset(ButterworthFilter* filter)
{
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    printf("滤波器已重置\n");
}


/**
 * @brief 生成测试信号（有用信号 + 噪声）
 *
 * 信号组成：
 *   - 1Hz正弦波（幅值1.0，代表有用信号）
 *   - 50Hz正弦波（幅值0.3，代表高频噪声）
 *
 * @param t  时间（秒）
 * @return   混合信号值
 */
float generate_test_signal(float t)
{
    float signal = sinf(2.0f * M_PI * 1.0f * t);      // 1Hz有用信号
    float noise = 0.3f * sinf(2.0f * M_PI * 50.0f * t);  // 50Hz噪声
    return signal + noise;
}


/**
 * =====================================================================
 * 主函数 - 演示巴特沃斯滤波器的使用
 * =====================================================================
 */
int main(void)
{
    printf("========================================\n");
    printf("  二阶巴特沃斯滤波器演示程序\n");
    printf("========================================\n\n");

    // ========================
    // 示例1：传感器信号滤波
    // ========================
    printf("【示例1：传感器信号滤波】\n");
    printf("----------------------------------------\n");

    // 创建滤波器实例
    ButterworthFilter sensor_filter;

    // 参数设置
    float fc_sensor = 5.0f;      // 截止频率5Hz（传感器信号一般低于5Hz）
    float fs_sensor = 100.0f;    // 采样频率100Hz

    // 初始化滤波器
    Butterworth_Init(&sensor_filter, fc_sensor, fs_sensor);

    // 模拟传感器数据处理
    printf("\n模拟10个采样点的滤波过程：\n");
    printf("时间(s)  原始值    滤波后    差值\n");
    printf("---------------------------------------\n");

    float dt = 1.0f / fs_sensor;  // 采样间隔
    for (int i = 0; i < 10; i++) {
        float t = i * dt;
        float raw_data = generate_test_signal(t);  // 原始信号（含噪声）
        float filtered_data = Butterworth_Update(&sensor_filter, raw_data);  // 滤波后信号
        float diff = raw_data - filtered_data;

        printf("%.3f    %7.4f   %7.4f   %7.4f\n", t, raw_data, filtered_data, diff);
    }


    // ========================
    // 示例2：对比不同截止频率
    // ========================
    printf("\n\n【示例2：对比不同截止频率的效果】\n");
    printf("----------------------------------------\n");

    ButterworthFilter filter_fc2, filter_fc5, filter_fc10;
    float fs = 100.0f;

    // 初始化三个不同截止频率的滤波器
    printf("\n初始化三个滤波器：\n");
    Butterworth_Init(&filter_fc2, 2.0f, fs);
    Butterworth_Init(&filter_fc5, 5.0f, fs);
    Butterworth_Init(&filter_fc10, 10.0f, fs);

    printf("\n在t=0.5s时刻的滤波效果对比：\n");
    printf("截止频率  滤波输出\n");
    printf("--------------------\n");

    // 运行到0.5秒
    float target_time = 0.5f;
    int num_samples = (int)(target_time * fs);

    for (int i = 0; i <= num_samples; i++) {
        float t = i / fs;
        float input = generate_test_signal(t);

        float out2 = Butterworth_Update(&filter_fc2, input);
        float out5 = Butterworth_Update(&filter_fc5, input);
        float out10 = Butterworth_Update(&filter_fc10, input);

        // 只打印最后一个点
        if (i == num_samples) {
            printf("fc=2Hz    %.4f\n", out2);
            printf("fc=5Hz    %.4f\n", out5);
            printf("fc=10Hz   %.4f\n", out10);
            printf("原始值    %.4f\n", input);
            printf("\n说明：fc越小，滤波效果越强，但响应也越慢\n");
        }
    }


    // ========================
    // 示例3：实际应用场景
    // ========================
    printf("\n\n【示例3：实际应用场景配置参考】\n");
    printf("----------------------------------------\n");
    printf("应用场景              截止频率  采样频率\n");
    printf("--------------------------------------------\n");
    printf("温度传感器            1 Hz      10~100 Hz\n");
    printf("加速度计（姿态）      5 Hz      200 Hz\n");
    printf("陀螺仪（姿态）        10 Hz     1000 Hz\n");
    printf("电池电压监测          1 Hz      100 Hz\n");
    printf("心率传感器            5 Hz      100 Hz\n");
    printf("音频低通              3400 Hz   16000 Hz\n");
    printf("--------------------------------------------\n");


    // ========================
    // 示例4：性能分析
    // ========================
    printf("\n\n【示例4：性能分析】\n");
    printf("----------------------------------------\n");

    // 计算一次滤波的运算量
    printf("每次滤波运算量：\n");
    printf("  - 浮点乘法：5次\n");
    printf("  - 浮点加法：4次\n");
    printf("  - 总计：约9次浮点运算\n\n");

    // 内存占用
    printf("内存占用：\n");
    printf("  - 滤波器结构体：%zu字节\n", sizeof(ButterworthFilter));
    printf("  - 系数：5个float = 20字节\n");
    printf("  - 历史数据：4个float = 16字节\n\n");

    // 实时性分析
    printf("实时性分析（假设单次浮点运算=10个时钟周期）：\n");
    printf("  - STM32F4(168MHz)：约0.5微秒/样本\n");
    printf("  - STM32F1(72MHz)：约1.2微秒/样本\n");
    printf("  - Arduino Uno(16MHz)：约5.6微秒/样本\n");


    printf("\n========================================\n");
    printf("  演示完成\n");
    printf("========================================\n");

    return 0;
}


/**
 * =====================================================================
 * 使用说明
 * =====================================================================
 *
 * 1. 基本使用流程：
 *    a) 创建滤波器实例：ButterworthFilter filter;
 *    b) 初始化：Butterworth_Init(&filter, fc, fs);
 *    c) 循环调用：output = Butterworth_Update(&filter, input);
 *
 * 2. 参数选择建议：
 *    - 截止频率fc：介于信号频率和噪声频率之间
 *    - 采样频率fs：建议 fs >= 10 * fc
 *    - 原则：fc越低滤波越强，但响应越慢
 *
 * 3. 常见应用配置：
 *    - 温度传感器：fc=1Hz, fs=10Hz
 *    - 加速度计：fc=5Hz, fs=200Hz
 *    - 陀螺仪：fc=10Hz, fs=1000Hz
 *
 * 4. 优点：
 *    - 通带平坦，信号失真小
 *    - 计算量适中
 *    - 数学特性明确
 *
 * 5. 缺点：
 *    - 有相位延迟
 *    - 需要浮点运算
 *    - 过渡带较宽
 *
 * 6. 注意事项：
 *    - 每次调用Update的时间间隔必须是1/fs
 *    - 系统重启或异常时需要Reset
 *    - fc不能超过fs/2（奈奎斯特频率）
 *
 * 7. 编译运行：
 *    gcc 08_巴特沃斯滤波.c -o butterworth -lm
 *    ./butterworth
 *
 * =====================================================================
 */
