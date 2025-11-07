#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"

#define MODE NORMAL_MODE
#define BLACK_MARGIN 4095
#define WHITE_MARGIN 0
#define LSA_MIN 0
#define LSA_MAX 1200
#define BLACK_BOUNDARY 550
#define OPT_DUTY 60
#define LOW_DUTY 55
#define HIGH_DUTY 68
#define IR_PIN 0
#define NOR 30
#define MAX_JUNCTIONS 50

int junction_preference = 0;
const int weights[5] = {-5, -3, 1, 3, 5};
int junction_memory[MAX_JUNCTIONS] = {0};
int junction_index = 0;

float error = 0, prev_error = 0, cum_error = 0, correction = 0;
float left_duty = 0, right_duty = 0;
int left_flag = 0, right_flag = 0, uturn_flag = 0;

int s0_hist[NOR] = {0}, s4_hist[NOR] = {0}, hist_idx = 0;
line_sensor_array ls;

static TickType_t left_turn_start = 0;
static int end_check_pending = 0;

static inline int all_sensors_black()
{
    for (int i = 0; i < 5; ++i)
        if (ls.adc_reading[i] <= BLACK_BOUNDARY)
            return 0;
    return 1;
}

static inline int all_sensors_white()
{
    for (int i = 0; i < 5; ++i)
        if (ls.adc_reading[i] >= BLACK_BOUNDARY)
            return 0;
    return 1;
}

void store_history()
{
    s0_hist[hist_idx] = (ls.adc_reading[0] > BLACK_BOUNDARY) ? 1 : 0;
    s4_hist[hist_idx] = (ls.adc_reading[4] > BLACK_BOUNDARY) ? 1 : 0;
    hist_idx = (hist_idx + 1) % NOR;
}

float hist_avg(int arr[])
{
    int s = 0;
    for (int i = 0; i < NOR; ++i)
        s += arr[i];
    return (float)s / NOR;
}

void compute_error()
{
    int any_black = 0;
    float wsum = 0, sum = 0;
    for (int i = 0; i < 5; ++i)
    {
        int k = (ls.adc_reading[i] > BLACK_BOUNDARY) ? 1 : 0;
        wsum += weights[i] * k;
        sum += k;
        if (k)
            any_black = 1;
    }

    int left_read = ls.adc_reading[0] > BLACK_BOUNDARY;
    int right_read = ls.adc_reading[4] > BLACK_BOUNDARY;
    left_flag = left_read ? 1 : 0;
    right_flag = (!left_read && right_read) ? 1 : (left_read ? 0 : right_flag);

    if (!any_black)
        uturn_flag = 1;
    else
        uturn_flag = 0;

    if (sum != 0)
        error = wsum / sum;
    else
        error = (prev_error > 0) ? 10 : -10;
}

void compute_correction()
{
    pid_const_t p = read_pid_const();
    float diff = error - prev_error;
    cum_error += error;
    cum_error = bound(cum_error, -60, 60);
    correction = p.kp * error + p.ki * cum_error + p.kd * diff;
    prev_error = error;
}

void normalize_sensors()
{
    for (int i = 0; i < 5; ++i)
    {
        ls.adc_reading[i] = bound(ls.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
        ls.adc_reading[i] = map(ls.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, LSA_MIN, LSA_MAX);
        ls.adc_reading[i] = 1000 - ls.adc_reading[i];
    }
}

int execute_turns(motor_handle_t m0, motor_handle_t m1, adc_handle_t lsensor)
{
    float leftavg = hist_avg(s0_hist);
    float rightavg = hist_avg(s4_hist);

    // === CHECK FOR END AFTER LEFT TURN ===
    if (end_check_pending)
    {
        TickType_t now = xTaskGetTickCount();
        if ((now - left_turn_start) > pdMS_TO_TICKS(280))
        {
            ls = read_line_sensor(lsensor);
            normalize_sensors();
            
            if (ls.adc_reading[4] < BLACK_BOUNDARY)
            {
                ESP_LOGI("junction", "END reached!");
                set_motor_speed(m0, MOTOR_STOP, 0);
                set_motor_speed(m1, MOTOR_STOP, 0);
                return 1;
            }
            end_check_pending = 0;
        }
    }

    // === LEFT TURN ===
    if (left_flag && !right_flag)
    {
        if (!end_check_pending)
        {
            left_turn_start = xTaskGetTickCount();
            end_check_pending = 1;
        }

        if (ls.adc_reading[0] > BLACK_BOUNDARY && ls.adc_reading[1] > BLACK_BOUNDARY)
        {
            set_motor_speed(m0, MOTOR_BACKWARD, HIGH_DUTY);
            set_motor_speed(m1, MOTOR_FORWARD, HIGH_DUTY);
        }
        else if (ls.adc_reading[0] > BLACK_BOUNDARY)
        {
            set_motor_speed(m0, MOTOR_FORWARD, left_duty * 0.5);
            set_motor_speed(m1, MOTOR_FORWARD, right_duty);
        }
        else
        {
            set_motor_speed(m0, MOTOR_FORWARD, left_duty);
            set_motor_speed(m1, MOTOR_FORWARD, right_duty);
        }
    }
    // === RIGHT TURN ===
    else if (right_flag && !left_flag)
    {
        if (ls.adc_reading[2] > BLACK_BOUNDARY)
        {
            set_motor_speed(m0, MOTOR_FORWARD, left_duty);
            set_motor_speed(m1, MOTOR_FORWARD, right_duty);
        }
        else if (ls.adc_reading[3] > BLACK_BOUNDARY || ls.adc_reading[4] > BLACK_BOUNDARY)
        {
            set_motor_speed(m0, MOTOR_FORWARD, left_duty);
            set_motor_speed(m1, MOTOR_BACKWARD, left_duty);
        }
        else
        {
            set_motor_speed(m0, MOTOR_FORWARD, left_duty);
            set_motor_speed(m1, MOTOR_FORWARD, right_duty);
        }
    }
    // === T-JUNCTION ===
    else if (left_flag && right_flag)
    {
        set_motor_speed(m0, MOTOR_BACKWARD, HIGH_DUTY);
        set_motor_speed(m1, MOTOR_FORWARD, HIGH_DUTY);
    }
    // === U-TURN ===
    else if (uturn_flag)
    {
        end_check_pending = 0;
        
        if (junction_index > 0)
            junction_memory[junction_index - 1] = -1;
        if (rightavg > 0.1 && leftavg < 0.1)
        {
            set_motor_speed(m0, MOTOR_FORWARD, HIGH_DUTY);
            set_motor_speed(m1, MOTOR_BACKWARD, HIGH_DUTY);
        }
        else
        {
            set_motor_speed(m0, MOTOR_BACKWARD, HIGH_DUTY);
            set_motor_speed(m1, MOTOR_FORWARD, HIGH_DUTY);
        }
        vTaskDelay(400 / portTICK_PERIOD_MS);
    }
    // === NORMAL LINE FOLLOWING ===
    else
    {
        set_motor_speed(m0, MOTOR_FORWARD, left_duty);
        set_motor_speed(m1, MOTOR_FORWARD, right_duty);
    }

    return 0;
}

void line_follow_task(void *arg)
{
    motor_handle_t m0, m1;
    adc_handle_t lsensor;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    ESP_ERROR_CHECK(enable_motor_driver(&m0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&m1, MOTOR_A_1));
    ESP_ERROR_CHECK(enable_line_sensor(&lsensor));
    ESP_ERROR_CHECK(enable_bar_graph());

#ifdef CONFIG_ENABLE_OLED
    ESP_ERROR_CHECK(init_oled());
    vTaskDelay(100);
    lv_obj_clean(lv_scr_act());
#endif

    while (1)
    {
        // === BOX DETECTION ===
        int ir_state = gpio_get_level(IR_PIN);
        if (ir_state == 0)
        {
            ESP_LOGI("debug", "Box detected — pushing.");

            set_motor_speed(m0, MOTOR_FORWARD, OPT_DUTY);
            set_motor_speed(m1, MOTOR_FORWARD, OPT_DUTY);
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            ls = read_line_sensor(lsensor);
            normalize_sensors();
            int initial_color = (ls.adc_reading[2] > BLACK_BOUNDARY) ? 1 : 0;

            set_motor_speed(m0, MOTOR_BACKWARD, HIGH_DUTY);
            set_motor_speed(m1, MOTOR_BACKWARD, HIGH_DUTY);
            vTaskDelay(300 / portTICK_PERIOD_MS);

            set_motor_speed(m0, MOTOR_BACKWARD, HIGH_DUTY);
            set_motor_speed(m1, MOTOR_FORWARD, HIGH_DUTY);
            vTaskDelay(500 / portTICK_PERIOD_MS);

            end_check_pending = 0;
            continue;
        }

        // === LINE FOLLOWING ===
        ls = read_line_sensor(lsensor);
        normalize_sensors();

        compute_error();
        compute_correction();
        store_history();

        left_duty = bound((OPT_DUTY + correction), LOW_DUTY, HIGH_DUTY);
        right_duty = bound((OPT_DUTY - correction), LOW_DUTY, HIGH_DUTY);

        int end_detected = execute_turns(m0, m1, lsensor);
        if (end_detected)
        {
            while (1)
            {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }

#ifdef CONFIG_ENABLE_OLED
        if (read_pid_const().val_changed)
        {
            display_pid_values(read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
            reset_val_changed_pid_const();
        }
#endif
        ESP_LOGI("debug", "err: %f corr: %f Ld: %f Rd: %f", error, correction, left_duty, right_duty);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}	
