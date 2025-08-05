#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/list.h"
#include "esp_check.h"
#include "controller.h"

#define PRINT(window, fmt, args...) printf("{"#window"}"fmt"\n", ##args)

// ============================================================================
// 电机控制结构体定义
// ============================================================================
typedef struct mc_t {
    bdc_motor_handle_t motor;           /*!< 电机驱动句柄 */
    encoder_handle_t encoder;           /*!< 编码器句柄 */
    pid_ctrl_block_handle_t pid;        /*!< PID控制器句柄 */
    ListItem_t item;                    /*!< 链表节点 */
    float max_rpm;                      /*!< 最大转速限制 */
    float expect_rpm;                   /*!< 期望转速 */
    pid_ctrl_parameter_t pid_params;    /*!< PID参数 */
#ifdef USE_TB6612_DRIVER
    gpio_num_t dir1_gpio;              /*!< TB6612方向控制1 GPIO */
    gpio_num_t dir2_gpio;              /*!< TB6612方向控制2 GPIO */
    bool is_forward;                   /*!< 当前方向状态 (true=正转, false=反转) */
#endif
} mc_t;

#define TAG ""

#define RETURN_ON_FALSE(a, msg) ESP_RETURN_ON_FALSE(a, ESP_ERR_INVALID_STATE, TAG, msg)
#define RETURN_ON_INCREATED(msg) RETURN_ON_FALSE(g_timer_handle == NULL, msg)
#define RETURN_ON_UNCREATED(msg) RETURN_ON_FALSE(g_timer_handle != NULL, msg)
#define RETURN_ON_INACTIVE(msg) RETURN_ON_FALSE(!xTimerIsTimerActive(g_timer_handle), msg)
#define RETURN_ON_UNACTIVE(msg) RETURN_ON_FALSE(xTimerIsTimerActive(g_timer_handle), msg)
#define RETURN_ON_LIST_EMPTY(msg) RETURN_ON_FALSE(!listLIST_IS_EMPTY(&g_list), msg)
#define RETURN_ON_LIST_NOT_EMPTY(msg) RETURN_ON_FALSE(listLIST_IS_EMPTY(&g_list), msg)
#define RETURN_ON_ENCODER_IN_LIST(handle, msg) RETURN_ON_FALSE(!listIS_CONTAINED_WITHIN(&g_list, &handle->item), msg)
#define RETURN_ON_ENCODER_NOT_IN_LIST(handle, msg) RETURN_ON_FALSE(listIS_CONTAINED_WITHIN(&g_list, &handle->item), msg)

static List_t g_list;
static TimerHandle_t g_timer_handle = NULL;

#ifdef USE_TB6612_DRIVER
// ============================================================================
// TB6612驱动控制函数
// ============================================================================

/**
 * @brief TB6612电机方向控制
 *
 * @param handle 电机控制句柄
 * @param forward true=正转, false=反转
 * @return esp_err_t
 */
static esp_err_t tb6612_set_direction(mc_handle_t handle, bool forward) {
    if (handle->dir1_gpio != -1 && handle->dir2_gpio != -1) {
        if (forward) {
            // 正转: DIR1=1, DIR2=0
            ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir1_gpio, 1), TAG, "set dir1 gpio failed");
            ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir2_gpio, 0), TAG, "set dir2 gpio failed");
        } else {
            // 反转: DIR1=0, DIR2=1
            ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir1_gpio, 0), TAG, "set dir1 gpio failed");
            ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir2_gpio, 1), TAG, "set dir2 gpio failed");
        }
        handle->is_forward = forward;
        ESP_LOGD(TAG, "TB6612 direction set to %s", forward ? "forward" : "reverse");
    }
    return ESP_OK;
}

/**
 * @brief TB6612电机刹车
 *
 * @param handle 电机控制句柄
 * @return esp_err_t
 */
static esp_err_t tb6612_brake(mc_handle_t handle) {
    if (handle->dir1_gpio != -1 && handle->dir2_gpio != -1) {
        // 刹车: DIR1=1, DIR2=1
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir1_gpio, 1), TAG, "set dir1 gpio failed");
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir2_gpio, 1), TAG, "set dir2 gpio failed");
        ESP_LOGD(TAG, "TB6612 brake applied");
    }
    return ESP_OK;
}

/**
 * @brief 初始化TB6612方向控制GPIO
 *
 * @param handle 电机控制句柄
 * @param dir1_gpio 方向控制1 GPIO
 * @param dir2_gpio 方向控制2 GPIO
 * @return esp_err_t
 */
static esp_err_t tb6612_init_gpio(mc_handle_t handle, gpio_num_t dir1_gpio, gpio_num_t dir2_gpio) {
    handle->dir1_gpio = dir1_gpio;
    handle->dir2_gpio = dir2_gpio;
    handle->is_forward = true;

    // 初始化DIR1 GPIO
    if (dir1_gpio != -1) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dir1_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        
        // 检查GPIO是否有效
        if (dir1_gpio >= SOC_GPIO_PIN_COUNT) {
            return ESP_ERR_INVALID_ARG;
        }
        
        esp_err_t ret = gpio_config(&io_conf);

        ESP_LOGI(TAG, "DIR1 GPIO configured successfully");
        ESP_RETURN_ON_ERROR(gpio_set_level(dir1_gpio, 0), TAG, "set dir1 gpio level failed");
    }

    // 初始化DIR2 GPIO
    if (dir2_gpio != -1) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dir2_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        
        // 检查GPIO是否有效
        if (dir2_gpio >= SOC_GPIO_PIN_COUNT) {
            return ESP_ERR_INVALID_ARG;
        }
        
        esp_err_t ret = gpio_config(&io_conf);
        
        ESP_LOGI(TAG, "DIR2 GPIO configured successfully");
        ESP_RETURN_ON_ERROR(gpio_set_level(dir2_gpio, 0), TAG, "set dir2 gpio level failed");
    }

    ESP_LOGI(TAG, "TB6612 GPIO initialization completed successfully");
    return ESP_OK;
}
#endif

// ============================================================================
// PID控制循环 - 支持DRV8833和TB6612两种驱动
// ============================================================================
static void pid_loop_tcb(TimerHandle_t xTimer) {
    ListItem_t *item = listGET_HEAD_ENTRY(&g_list);
    while (item != listGET_END_MARKER(&g_list)) {
        mc_handle_t handle = item->pvOwner;

        // 1. 读取当前转速和计算误差
        float current_rpm = encoder_get_rpm(handle->encoder);
        float error = handle->expect_rpm - current_rpm;

        //ESP_LOGI(TAG, "<rpm>:%.2f,%.2f\n", handle->expect_rpm,current_rpm);
        // 2. PID计算
        float new_duty;
        pid_compute(handle->pid, error, &new_duty);

        // 3. 根据驱动类型执行电机控制
#ifdef USE_TB6612_DRIVER
        if (new_duty > 0) {
            // 正转
            tb6612_set_direction(handle, true);
            bdc_motor_set_speed(handle->motor, new_duty);
        } else if (new_duty < 0) {
            // 反转
            tb6612_set_direction(handle, false);
            bdc_motor_set_speed(handle->motor, -new_duty);
        } else {
            // 停止 (PWM=0)
            bdc_motor_set_speed(handle->motor, 0);
        }
#else
        // DRV8833模式：使用双PWM控制
        if (new_duty > 0) {
            bdc_motor_forward(handle->motor);
            bdc_motor_set_speed(handle->motor, new_duty);
        } else {
            bdc_motor_reverse(handle->motor);
            bdc_motor_set_speed(handle->motor, -new_duty);
        }
#endif

        item = listGET_NEXT(item);
    }
}

static esp_err_t mc_start(mc_handle_t handle) {
    ESP_LOGD(TAG, "Enable motor");
    ESP_RETURN_ON_ERROR(bdc_motor_enable(handle->motor), TAG, "enable motor failed");
    
    ESP_LOGD(TAG, "Forward motor");
    ESP_RETURN_ON_ERROR(bdc_motor_forward(handle->motor), TAG, "forward motor failed");

    return ESP_OK;
}

static esp_err_t mc_stop(mc_handle_t handle) {
    ESP_LOGD(TAG, "disable motor unit");
    ESP_RETURN_ON_ERROR(bdc_motor_disable(handle->motor), TAG, "disable motro failed");

    return ESP_OK;
}

esp_err_t mc_timer_loop_create(TickType_t pid_period_tick, TickType_t encoder_period_tick) {
    RETURN_ON_INCREATED("timer loop has been already created");

    esp_err_t ret;

    ESP_RETURN_ON_ERROR(encoder_timer_loop_create(encoder_period_tick), TAG, "");

    g_timer_handle = xTimerCreate("pid_loop",
        pid_period_tick, pdTRUE, NULL, pid_loop_tcb);
    ESP_GOTO_ON_FALSE(g_timer_handle != NULL, ESP_ERR_NO_MEM, err, TAG, "create timer loop failed");

    vListInitialise(&g_list);

    return ESP_OK;

err:
    ESP_RETURN_ON_ERROR(encoder_timer_loop_del(), TAG, "");
    return ret;
}

esp_err_t mc_timer_loop_del(void) {
    RETURN_ON_UNCREATED("timer loop is not created or has been deleted");
    RETURN_ON_INACTIVE("please call `mc_timer_loop_stop` to stop it at first");
    RETURN_ON_LIST_NOT_EMPTY("please delete all encoder handle at first");

    ESP_RETURN_ON_ERROR(encoder_timer_loop_del(), TAG, "");
    ESP_RETURN_ON_FALSE(xTimerDelete(g_timer_handle, 0), ESP_FAIL, TAG, "stop timer loop failed");
    g_timer_handle = NULL;
    return ESP_OK;
}

esp_err_t mc_timer_loop_start(void) {
    RETURN_ON_UNCREATED("please call `mc_timer_loop_init` to create timer loop at first");
    RETURN_ON_INACTIVE("timer loop has been already started");

    ESP_RETURN_ON_ERROR(encoder_timer_loop_start(), TAG, "");

    esp_err_t ret;
    ListItem_t *item = listGET_HEAD_ENTRY(&g_list);
    ListItem_t *_item = item;
    while (item != listGET_END_MARKER(&g_list)) {
        mc_handle_t handle = item->pvOwner;
        ESP_GOTO_ON_ERROR(mc_start(handle), err, TAG, "start failed");
        item = listGET_NEXT(item);
    }

    ESP_GOTO_ON_FALSE(xTimerStart(g_timer_handle, 0), ESP_FAIL, err, TAG, "start timer loop failed");

    return ESP_OK;

err:
    ESP_RETURN_ON_ERROR(encoder_timer_loop_stop(), TAG, "");
    while (_item != item) {
        mc_handle_t handle = item->pvOwner;
        ESP_RETURN_ON_ERROR(mc_stop(handle), TAG, "stop failed");
        item = listGET_NEXT(item);
    }

    return ret;
}

esp_err_t mc_timer_loop_stop(void) {
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_UNACTIVE("timer loop is not started or has been stopped");

    ESP_RETURN_ON_FALSE(xTimerStop(g_timer_handle, 0), ESP_FAIL, TAG,
                        "deactivate timer loop failed");

    ListItem_t *item = listGET_HEAD_ENTRY(&g_list);
    while (item != listGET_END_MARKER(&g_list)) {
        mc_handle_t handle = item->pvOwner;
        ESP_RETURN_ON_ERROR(mc_stop(handle), TAG, "stop failed");
        item = listGET_NEXT(item);
    }

    ESP_RETURN_ON_ERROR(encoder_timer_loop_stop(), TAG, "");

    return ESP_OK;
}

esp_err_t mc_new(const mc_config_t *config, float max_rpm, mc_handle_t *ret_handle) {
    esp_err_t ret;
    ESP_LOGI(TAG, "Creating new motor controller with max RPM: %.2f", max_rpm);
    
    ESP_RETURN_ON_FALSE(config != NULL && ret_handle != NULL, ESP_ERR_NO_MEM,
        TAG, "no mem for motor control handle");
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_INACTIVE("can not create encoder when timer loop is active");

    mc_handle_t handle = malloc(sizeof(mc_t));
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG, "no mem for motor control handle");

#ifdef USE_TB6612_DRIVER
    ESP_LOGI(TAG, "Using TB6612 driver mode");    
    // TB6612模式下，先初始化方向控制GPIO
    if (config->tb6612_config.dir1_gpio != -1 && config->tb6612_config.dir2_gpio != -1) {
        ESP_GOTO_ON_ERROR(tb6612_init_gpio(handle,
                                         config->tb6612_config.dir1_gpio,
                                         config->tb6612_config.dir2_gpio),
                         err, TAG, "TB6612 GPIO init failed");
    }
#endif

    // 然后创建MCPWM设备
    ESP_LOGI(TAG, "Creating MCPWM device...");
    ret = bdc_motor_new_mcpwm_device(&config->motor_config, &config->mcpwm_config, &handle->motor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Create MCPWM device failed: %s", esp_err_to_name(ret));
        goto err;
    }
    ESP_LOGI(TAG, "MCPWM device created successfully");

    // 创建编码器
    ESP_GOTO_ON_ERROR(encoder_new(&config->encoder_config, &handle->encoder), err,
        TAG, "create encoder handle failed");
    ESP_LOGI(TAG, "Encoder created successfully");

    // 创建PID控制器
    ESP_LOGI(TAG, "Creating PID controller with P=%.2f, I=%.2f, D=%.2f", 
             config->pid_config.init_param.kp,
             config->pid_config.init_param.ki,
             config->pid_config.init_param.kd);
    ESP_GOTO_ON_ERROR(pid_new_control_block(&config->pid_config, &handle->pid), err,
        TAG, "create pid control block failed");
    ESP_LOGI(TAG, "PID controller created successfully");

    ESP_LOGI(TAG, "Adding encoder to timer loop");
    ESP_RETURN_ON_ERROR(encoder_add_to_timer_loop(handle->encoder), TAG, "add encoder to timer loop failed");

#ifdef USE_TB6612_DRIVER
    // 保存TB6612配置
    handle->dir1_gpio = config->tb6612_config.dir1_gpio;
    handle->dir2_gpio = config->tb6612_config.dir2_gpio;
    handle->is_forward = true;

#endif

    handle->expect_rpm = 0;
    handle->max_rpm = max_rpm;
    vListInitialiseItem(&handle->item);
    handle->item.pvOwner = handle;

    handle->pid_params = config->pid_config.init_param;

    *ret_handle = handle;
    ESP_LOGI(TAG, "Motor controller created successfully");
    return ESP_OK;

err:
    if (handle) {
        free(handle);
    }
    ESP_LOGE(TAG, "Failed to create motor controller: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t mc_del(mc_handle_t handle) {
    RETURN_ON_ENCODER_IN_LIST(handle, "please remove encoder from timer loop at first");
    
    ESP_RETURN_ON_ERROR(encoder_remove_from_time_loop(handle->encoder), "TAG", "");

    if (handle->motor) {
        ESP_RETURN_ON_ERROR(bdc_motor_del(handle->motor), TAG, "delete bdc motor failed");
        handle->motor = NULL;
    }

    if (handle->encoder) {
        ESP_RETURN_ON_ERROR(encoder_del(handle->encoder), TAG, "delete encoder failed");
        handle->encoder = NULL;
    }

    if (handle->pid) {
        ESP_RETURN_ON_ERROR(pid_del_control_block(handle->pid), TAG, "delete pid control block failed");
        handle->pid = NULL;
    }

    free(handle);

    return ESP_OK;
}

esp_err_t mc_add_to_timer_loop(mc_handle_t handle) {
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_INACTIVE("the function can be call if and only if timer loop is not active");
    RETURN_ON_ENCODER_IN_LIST(handle, "it is already in list");

    vListInsertEnd(&g_list, &handle->item);

    return ESP_OK;
}

esp_err_t mc_remove_from_timer_loop(mc_handle_t handle) {
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_INACTIVE("the function can be call if and only if timer loop is not active");
    RETURN_ON_ENCODER_NOT_IN_LIST(handle, "it is not in list");

    uxListRemove(&handle->item);

    return ESP_OK;
}

void mc_set_pid_params(mc_handle_t handle, float kp, float ki, float kd) {
    handle->pid_params.kp = kp;
    handle->pid_params.ki = ki;
    handle->pid_params.kd = kd;
    pid_update_parameters(handle->pid, &handle->pid_params);
}
void mc_set_expect_rpm(mc_handle_t handle, float expect_rpm) {
    handle->expect_rpm = expect_rpm > handle->max_rpm ? handle->max_rpm
        : expect_rpm < -handle->max_rpm ? -handle->max_rpm : expect_rpm;
}

float mc_get_real_rpm(mc_handle_t handle) {
    return encoder_get_rpm(handle->encoder);
}

void mc_get_pid_params(mc_handle_t handle, float *kp, float *ki, float *kd) {
    if (handle && kp && ki && kd) {
        *kp = handle->pid_params.kp;
        *ki = handle->pid_params.ki;
        *kd = handle->pid_params.kd;
    }
}

float mc_get_expect_rpm(mc_handle_t handle) {
    return handle ? handle->expect_rpm : 0.0f;
}


