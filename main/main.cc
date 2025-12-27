/**
 * @file main.cc
 * @brief ESP32-S3 智能语音助手 - 舵机控制主程序
 *
 * 本程序实现了命令词识别和舵机控制功能，包括：
 * 1. 命令词识别 - 支持"帮我开灯"、"帮我关灯"等语音指令
 * 2. 舵机控制 - 根据语音指令控制舵机旋转
 *
 * 硬件配置：
 * - ESP32-S3-DevKitC-1开发板（需要PSRAM版本）
 * - INMP441数字麦克风（音频输入）
 *   连接方式：VDD->3.3V, GND->GND, SD->GPIO6, WS->GPIO4, SCK->GPIO5
 * - 舵机（GPIO1控制）
 *   连接方式：红线->5V/3.3V, 棕线->GND, 橙线->GPIO1
 *
 * 音频参数：
 * - 采样率：16kHz
 * - 声道：单声道(Mono)
 * - 位深度：16位
 *
 * 使用的AI模型：
 * - 命令词识别：MultiNet7中文命令词识别模型
 */

extern "C"
{
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mn_iface.h"           // 命令词识别接口
#include "esp_mn_models.h"          // 命令词模型管理
#include "esp_mn_speech_commands.h" // 命令词配置
#include "esp_process_sdkconfig.h"  // sdkconfig处理函数
#include "model_path.h"             // 模型路径定义
#include "bsp_board.h"              // 板级支持包，INMP441麦克风驱动
#include "esp_log.h"                // ESP日志系统
#include "driver/gpio.h"            // GPIO驱动
#include "driver/ledc.h"            // LEDC PWM驱动，用于舵机控制
}

#include "servo_controller.h" // 舵机控制器类

static const char *TAG = "舵机控制"; // 日志标签

// 命令词ID定义（对应commands_cn.txt中的ID）
#define COMMAND_ZHEN_BANG 310      // "真棒"
#define COMMAND_ZHI_JIN 311        // "纸巾"
#define COMMAND_LUO_BO 312         // "萝卜"

// 命令词配置结构体
typedef struct
{
    int command_id;
    const char *pinyin;
    const char *description;
} command_config_t;

// 自定义命令词列表
static const command_config_t custom_commands[] = {
    {COMMAND_ZHEN_BANG, "zhen bang", "真棒"},
    {COMMAND_ZHI_JIN, "zhi jin", "纸巾"},
    {COMMAND_LUO_BO, "luo bo", "萝卜"},
};

#define CUSTOM_COMMANDS_COUNT (sizeof(custom_commands) / sizeof(custom_commands[0]))

// 全局变量
static esp_mn_iface_t *multinet = NULL;
static model_iface_data_t *mn_model_data = NULL;

// 舵机控制器实例
static ServoController servo_controller;

/**
 * @brief 配置自定义命令词
 *
 * 该函数会清除现有命令词，然后添加自定义命令词列表中的所有命令
 *
 * @param multinet 命令词识别接口指针
 * @param mn_model_data 命令词模型数据指针
 * @return esp_err_t
 *         - ESP_OK: 配置成功
 *         - ESP_FAIL: 配置失败
 */
static esp_err_t configure_custom_commands(esp_mn_iface_t *multinet, model_iface_data_t *mn_model_data)
{
    ESP_LOGI(TAG, "开始配置自定义命令词...");

    // 首先尝试从sdkconfig加载默认命令词配置
    esp_mn_commands_update_from_sdkconfig(multinet, mn_model_data);

    // 清除现有命令词，重新开始
    esp_mn_commands_clear();

    // 分配命令词管理结构
    esp_err_t ret = esp_mn_commands_alloc(multinet, mn_model_data);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "命令词管理结构分配失败: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    // 添加自定义命令词
    int success_count = 0;
    int fail_count = 0;

    for (int i = 0; i < CUSTOM_COMMANDS_COUNT; i++)
    {
        const command_config_t *cmd = &custom_commands[i];

        ESP_LOGI(TAG, "添加命令词 [%d]: %s (%s)",
                 cmd->command_id, cmd->description, cmd->pinyin);

        // 添加命令词
        esp_err_t ret_cmd = esp_mn_commands_add(cmd->command_id, cmd->pinyin);
        if (ret_cmd == ESP_OK)
        {
            success_count++;
            ESP_LOGI(TAG, "✓ 命令词 [%d] 添加成功", cmd->command_id);
        }
        else
        {
            fail_count++;
            ESP_LOGE(TAG, "✗ 命令词 [%d] 添加失败: %s",
                     cmd->command_id, esp_err_to_name(ret_cmd));
        }
    }

    // 更新命令词到模型
    ESP_LOGI(TAG, "更新命令词到模型...");
    esp_mn_error_t *error_phrases = esp_mn_commands_update();
    if (error_phrases != NULL && error_phrases->num > 0)
    {
        ESP_LOGW(TAG, "有 %d 个命令词更新失败:", error_phrases->num);
        for (int i = 0; i < error_phrases->num; i++)
        {
            ESP_LOGW(TAG, "  失败命令 %d: %s",
                     error_phrases->phrases[i]->command_id,
                     error_phrases->phrases[i]->string);
        }
    }

    // 打印配置结果
    ESP_LOGI(TAG, "命令词配置完成: 成功 %d 个, 失败 %d 个", success_count, fail_count);

    // 打印激活的命令词
    ESP_LOGI(TAG, "当前激活的命令词列表:");
    multinet->print_active_speech_commands(mn_model_data);

    // 打印支持的命令列表
    ESP_LOGI(TAG, "支持的语音命令:");
    for (int i = 0; i < CUSTOM_COMMANDS_COUNT; i++)
    {
        const command_config_t *cmd = &custom_commands[i];
        ESP_LOGI(TAG, "  ID=%d: '%s'", cmd->command_id, cmd->description);
    }

    return (fail_count == 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief 获取命令词的中文描述
 *
 * @param command_id 命令ID
 * @return const char* 命令的中文描述，如果未找到返回"未知命令"
 */
static const char *get_command_description(int command_id)
{
    for (int i = 0; i < CUSTOM_COMMANDS_COUNT; i++)
    {
        if (custom_commands[i].command_id == command_id)
        {
            return custom_commands[i].description;
        }
    }
    return "未知命令";
}

/**
 * @brief 应用程序主入口函数
 *
 * 初始化INMP441麦克风硬件，加载命令词识别模型，
 * 然后进入主循环进行实时音频采集和命令词识别。
 */
extern "C" void app_main(void)
{
    // ========== 第一步：初始化舵机 ==========
    esp_err_t servo_ret = servo_controller.init();
    if (servo_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "舵机初始化失败: %s", esp_err_to_name(servo_ret));
        return;
    }

    // ========== 第二步：初始化INMP441麦克风硬件 ==========
    ESP_LOGI(TAG, "正在初始化INMP441数字麦克风...");
    ESP_LOGI(TAG, "音频参数: 采样率16kHz, 单声道, 16位深度");

    esp_err_t ret = bsp_board_init(16000, 1, 16); // 16kHz, 单声道, 16位
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "INMP441麦克风初始化失败: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "请检查硬件连接: VDD->3.3V, GND->GND, SD->GPIO6, WS->GPIO4, SCK->GPIO5");
        return;
    }
    ESP_LOGI(TAG, "✓ INMP441麦克风初始化成功");

    // ========== 第三步：初始化语音识别模型 ==========
    ESP_LOGI(TAG, "正在初始化命令词识别模型...");

    // 检查内存状态
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    ESP_LOGI(TAG, "内存状态检查:");
    ESP_LOGI(TAG, "  - 总可用内存: %zu KB", free_heap / 1024);
    ESP_LOGI(TAG, "  - 内部RAM: %zu KB", free_internal / 1024);
    ESP_LOGI(TAG, "  - PSRAM: %zu KB", free_spiram / 1024);

    if (free_heap < 100 * 1024)
    {
        ESP_LOGE(TAG, "可用内存不足，需要至少100KB");
        return;
    }

    // 从模型目录加载所有可用的语音识别模型
    ESP_LOGI(TAG, "开始加载模型文件...");

    // 临时添加错误处理和重试机制
    srmodel_list_t *models = NULL;
    int retry_count = 0;
    const int max_retries = 3;

    while (models == NULL && retry_count < max_retries)
    {
        ESP_LOGI(TAG, "尝试加载模型 (第%d次)...", retry_count + 1);

        // 在每次重试前等待一下
        if (retry_count > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        models = esp_srmodel_init("model");

        if (models == NULL)
        {
            ESP_LOGW(TAG, "模型加载失败，准备重试...");
            retry_count++;
        }
    }
    if (models == NULL)
    {
        ESP_LOGE(TAG, "语音识别模型初始化失败");
        ESP_LOGE(TAG, "请检查模型文件是否正确烧录到Flash分区");
        return;
    }

    // ========== 第四步：初始化命令词识别模型 ==========

    // 获取中文命令词识别模型（MultiNet7）
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    if (mn_name == NULL)
    {
        ESP_LOGE(TAG, "未找到中文命令词识别模型！");
        ESP_LOGE(TAG, "请确保已正确配置并烧录MultiNet7中文模型");
        return;
    }

    ESP_LOGI(TAG, "✓ 选择命令词模型: %s", mn_name);

    // 获取命令词识别接口
    multinet = esp_mn_handle_from_name(mn_name);
    if (multinet == NULL)
    {
        ESP_LOGE(TAG, "获取命令词识别接口失败，模型: %s", mn_name);
        return;
    }

    // 创建命令词模型数据实例
    mn_model_data = multinet->create(mn_name, 6000);
    if (mn_model_data == NULL)
    {
        ESP_LOGE(TAG, "创建命令词模型数据失败");
        return;
    }

    // 配置自定义命令词
    ESP_LOGI(TAG, "正在配置命令词...");
    esp_err_t cmd_config_ret = configure_custom_commands(multinet, mn_model_data);
    if (cmd_config_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "命令词配置失败");
        return;
    }
    ESP_LOGI(TAG, "✓ 命令词配置完成");

    // ========== 第五步：准备音频缓冲区 ==========
    // 获取模型要求的音频数据块大小（样本数 × 每样本字节数）
    int audio_chunksize = multinet->get_samp_chunksize(mn_model_data) * sizeof(int16_t);

    // 分配音频数据缓冲区内存
    int16_t *buffer = (int16_t *)malloc(audio_chunksize);
    if (buffer == NULL)
    {
        ESP_LOGE(TAG, "音频缓冲区内存分配失败，需要 %d 字节", audio_chunksize);
        ESP_LOGE(TAG, "请检查系统可用内存");
        return;
    }

    // 显示系统配置信息
    ESP_LOGI(TAG, "✓ 智能语音助手系统配置完成:");
    ESP_LOGI(TAG, "  - 命令词模型: %s", mn_name);
    ESP_LOGI(TAG, "  - 音频块大小: %d 字节", audio_chunksize);
    ESP_LOGI(TAG, "系统已就绪，持续监听命令词...");
    ESP_LOGI(TAG, "支持的指令:");
    ESP_LOGI(TAG, "  - '真棒': 左右45度快速摇摆3次后复位");
    ESP_LOGI(TAG, "  - '纸巾': 随机旋转45度或-90度");
    ESP_LOGI(TAG, "  - '萝卜': 随机旋转45度或-90度");

    // ========== 第六步：主循环 - 实时音频采集与命令词识别 ==========
    while (1)
    {
        // 从INMP441麦克风获取一帧音频数据
        // false参数表示获取处理后的音频数据（非原始通道数据）
        esp_err_t ret = bsp_get_feed_data(false, buffer, audio_chunksize);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "麦克风音频数据获取失败: %s", esp_err_to_name(ret));
            ESP_LOGE(TAG, "请检查INMP441硬件连接");
            vTaskDelay(pdMS_TO_TICKS(10)); // 等待10ms后重试
            continue;
        }

        // 直接进行命令词识别
        esp_mn_state_t mn_state = multinet->detect(mn_model_data, buffer);

        if (mn_state == ESP_MN_STATE_DETECTED)
        {
            // 获取识别结果
            esp_mn_results_t *mn_result = multinet->get_results(mn_model_data);
            if (mn_result->num > 0)
            {
                int command_id = mn_result->command_id[0];
                float prob = mn_result->prob[0];

                const char *cmd_desc = get_command_description(command_id);
                ESP_LOGI(TAG, "检测到命令词: ID=%d, 置信度=%.2f, 内容=%s, 命令='%s'",
                         command_id, prob, mn_result->string, cmd_desc);

                // 处理命令
                if (command_id == COMMAND_ZHEN_BANG)
                {
                    ESP_LOGI(TAG, "执行真棒命令 - 左右45度快速摇摆");

                    // 左右快速摇摆3次
                    for (int i = 0; i < 3; i++)
                    {
                        servo_controller.rotate(45);   // 向右45度
                        vTaskDelay(pdMS_TO_TICKS(150));
                        servo_controller.rotate(-90);  // 向左90度（相对当前位置）
                        vTaskDelay(pdMS_TO_TICKS(150));
                        servo_controller.rotate(45);   // 回到右45度
                        vTaskDelay(pdMS_TO_TICKS(150));
                    }

                    // 复位到中心位置
                    servo_controller.resetToCenter();
                    ESP_LOGI(TAG, "✓ 真棒命令执行完成，舵机已复位");
                }
                else if (command_id == COMMAND_ZHI_JIN)
                {
                    // 随机选择旋转角度：45度或-90度
                    int random_angle = (rand() % 2 == 0) ? 45 : -90;

                    ESP_LOGI(TAG, "执行纸巾命令 - 随机旋转%d度", random_angle);
                    servo_controller.rotate(random_angle);
                    ESP_LOGI(TAG, "✓ 纸巾命令执行完成，舵机保持在当前位置");
                }
                else if (command_id == COMMAND_LUO_BO)
                {
                    // 随机选择旋转角度：45度或-90度
                    int random_angle = (rand() % 2 == 0) ? 45 : -90;

                    ESP_LOGI(TAG, "执行萝卜命令 - 随机旋转%d度", random_angle);
                    servo_controller.rotate(random_angle);
                    ESP_LOGI(TAG, "✓ 萝卜命令执行完成，舵机保持在当前位置");
                }

                // 清理缓冲区，继续监听下一个命令
                multinet->clean(mn_model_data);
                ESP_LOGI(TAG, "命令执行完成，继续监听...");
            }
        }

        // 短暂延时，避免CPU占用过高，同时保证实时性
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // ========== 资源清理 ==========
    // 注意：由于主循环是无限循环，以下代码正常情况下不会执行
    // 仅在程序异常退出时进行资源清理
    ESP_LOGI(TAG, "正在清理系统资源...");

    // 释放音频缓冲区内存
    if (buffer != NULL)
    {
        free(buffer);
    }

    // 删除当前任务
    vTaskDelete(NULL);
}
