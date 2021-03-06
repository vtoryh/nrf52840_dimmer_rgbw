﻿#include "sdk_config.h"
#include "zb_error_handler.h"
#include "zb_ha_dimmable_light.h"
#include "zb_mem_config_med.h"
#include "zb_nrf52840_internal.h"
#include "zboss_api.h"
#include "zigbee_helpers.h"

#include "zboss_api_af_addons.h"

#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "nrf_drv_pwm.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define MAX_CHILDREN                 10                      // максимальное количество подключенных дочерних девайсов (для роутера)
#define IEEE_CHANNEL_MASK            (1l << ZIGBEE_CHANNEL)  // используемые каналы
#define HA_DIMMABLE_LIGHT_ENDPOINT_R 10                      // 4 эндпоинта для раздельного управления каждым из 4 цветов
#define HA_DIMMABLE_LIGHT_ENDPOINT_G 11                      //
#define HA_DIMMABLE_LIGHT_ENDPOINT_B 12                      //
#define HA_DIMMABLE_LIGHT_ENDPOINT_W 13                      //
#define ERASE_PERSISTENT_CONFIG      ZB_FALSE                // если установить в TRUE, то девайс будет с каждым включением всё забывать и спариваться заново.
#define DIMMER_PWM_INSTANCE          NRF_DRV_PWM_INSTANCE(0) // инстанс ШИМ, используемый девайсом
#define DIMMER_PWM_VALUE_MAX         256                     // настройки ШИМ, не вникал в них
#define DIMMER_PWM_VALUE_MIN         35                      // это вообще не используется, бездумно передрал с предыдущей

#define BULB_INIT_BASIC_APP_VERSION   01                                  /**< Version of the application software (1 byte). */
#define BULB_INIT_BASIC_STACK_VERSION 10                                  /**< Version of the implementation of the ZigBee stack (1 byte). */
#define BULB_INIT_BASIC_HW_VERSION    11                                  /**< Version of the hardware of the device (1 byte). */
#define BULB_INIT_BASIC_MANUF_NAME    "mluvke"                            /**< Manufacturer name (32 bytes). */
#define BULB_INIT_BASIC_MODEL_ID      "RGBW Dimmer 1.1"                   /**< Model number assigned by manufacturer (32-bytes long string). */
#define BULB_INIT_BASIC_DATE_CODE     "20191020"                          /**< First 8 bytes specify the date of manufacturer of the device in ISO 8601 format (YYYYMMDD). The rest (8 bytes) are manufacturer specific. */
#define BULB_INIT_BASIC_POWER_SOURCE  ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE /**< Type of power sources available for the device. For possible values see section 3.2.2.2.8 of ZCL specification. */
#define BULB_INIT_BASIC_LOCATION_DESC "Office desk"                       /**< Describes the physical location of the device (16 bytes). May be modified during commisioning process. */
#define BULB_INIT_BASIC_PH_ENV        ZB_ZCL_BASIC_ENV_UNSPECIFIED        /**< Describes the type of physical environment. For possible values see section 3.2.2.2.10 of ZCL specification. */

#define IDENTIFY_MODE_ENTER_BUTTON BSP_BOARD_BUTTON_0 // номер кнопки спаривания устройств в списке BUTTONS_LIST в файле описания платы
#define ZIGBEE_NETWORK_STATE_LED   BSP_BOARD_LED_0    // номер светодиода состояния подключения в списке LEDS_LIST в файле описания платы

// пины, используемые под каждый из 4 цветов.
// LED2_Dx задаются в файле описания платы (mluvke_debug_breadboard.h в моём случае),
// файл инклудится через дефайн CUSTOM_BOARD_INC в настройках проекта (+ дефайн BOARD_MLUVKE_DEBUG_BREADBOARD)
#define DIMMER_CHANNEL_PIN_R LED2_DR // pwm channel 0
#define DIMMER_CHANNEL_PIN_G LED2_DG // pwm channel 1
#define DIMMER_CHANNEL_PIN_B LED2_DB // pwm channel 2
#define DIMMER_CHANNEL_PIN_W LED2_DW // pwm channel 3

// настройки ШИМ, не вникал, т.к. работает само из коробки.
static nrf_drv_pwm_t m_led_pwm = DIMMER_PWM_INSTANCE; // инстанс ШИМ
static nrf_pwm_values_individual_t m_led_values; // массив из 4 яркостей для каждого из 4 светодиодов
static nrf_pwm_sequence_t const m_led_seq =
{
        .values.p_individual = &m_led_values,
        .length = NRF_PWM_VALUES_LENGTH(m_led_values),
        .repeats = 0,
        .end_delay = 0
};

// макрос, используемый для заполнения структуры в памяти под "простое" описание эндпоинта, когда эндпоинтов несколько.
#define ZB_ZCL_DECLARE_HA_DIMMABLE_LIGHT_SIMPLE_DESC_VA(ep_name, ep_id, in_clust_num, out_clust_num) \
    ZB_DECLARE_SIMPLE_DESC_VA(in_clust_num, out_clust_num, ep_name);                                 \
    ZB_AF_SIMPLE_DESC_TYPE_VA(in_clust_num, out_clust_num, ep_name)                                  \
    simple_desc_##ep_name =                                                                          \
        {                                                                                            \
            ep_id,                                                                                   \
            ZB_AF_HA_PROFILE_ID,                                                                     \
            ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,                                                          \
            ZB_HA_DEVICE_VER_DIMMABLE_LIGHT,                                                         \
            0,                                                                                       \
            in_clust_num,                                                                            \
            out_clust_num,                                                                           \
            {                                                                                        \
                ZB_ZCL_CLUSTER_ID_BASIC,                                                             \
                ZB_ZCL_CLUSTER_ID_IDENTIFY,                                                          \
                ZB_ZCL_CLUSTER_ID_SCENES,                                                            \
                ZB_ZCL_CLUSTER_ID_GROUPS,                                                            \
                ZB_ZCL_CLUSTER_ID_ON_OFF,                                                            \
                ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,                                                     \
            }}

// макрос описывает структуру эндпоинта целиком со всеми потрохами
#define ZB_HA_DECLARE_LIGHT_EP(ep_name, ep_id, cluster_list)                                                                                    \
    ZB_ZCL_DECLARE_HA_DIMMABLE_LIGHT_SIMPLE_DESC_VA(ep_name, ep_id, ZB_HA_DIMMABLE_LIGHT_IN_CLUSTER_NUM, ZB_HA_DIMMABLE_LIGHT_OUT_CLUSTER_NUM); \
    ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info##device_ctx_name, ZB_HA_DIMMABLE_LIGHT_REPORT_ATTR_COUNT);                                \
    ZBOSS_DEVICE_DECLARE_LEVEL_CONTROL_CTX(cvc_alarm_info##device_ctx_name, ZB_HA_DIMMABLE_LIGHT_CVC_ATTR_COUNT);                               \
    ZB_AF_DECLARE_ENDPOINT_DESC(ep_name, ep_id, ZB_AF_HA_PROFILE_ID,                                                                            \
        0,                                                                                                                                      \
        NULL,                                                                                                                                   \
        ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t),                                                                                 \
        cluster_list,                                                                                                                           \
        (zb_af_simple_desc_1_1_t *)&simple_desc_##ep_name,                                                                                      \
        ZB_HA_DIMMABLE_LIGHT_REPORT_ATTR_COUNT,                                                                                                 \
        reporting_info##device_ctx_name,                                                                                                        \
        ZB_HA_DIMMABLE_LIGHT_CVC_ATTR_COUNT,                                                                                                    \
        cvc_alarm_info##device_ctx_name)

// вспомогательный макрос, нужный в макросе ZB_DECLARE_DIMMABLE_CONTROL_CLUSTER_LIST для заполнения кластеров
#define ZB_ZCL_DECLARE_LEVEL_CONTROL_ATTRIB_LIST_VA(attr_list, current_level, remaining_time, ...) \
    zb_zcl_level_control_move_status_t move_status_data_ctx##__VA_ARGS__##_attr_list;              \
    ZB_ZCL_START_DECLARE_ATTRIB_LIST(attr_list)                                                    \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, (current_level))              \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_REMAINING_TIME_ID, (remaining_time))            \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_MOVE_STATUS_ID,                                 \
        (&(move_status_data_ctx##__VA_ARGS__##_attr_list)))                                        \
    ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST

#if !defined ZB_ROUTER_ROLE
#error Define ZB_ROUTER_ROLE to compile light bulb (Router) source code.
#endif

// структуры, которые в последствии войдут в bulb_device_ctx_t. Используются в bulb_device_ctx_t, где хранится
// полное состояние эндпоинта
typedef struct
{
    zb_uint8_t zcl_version;
    zb_uint8_t app_version;
    zb_uint8_t stack_version;
    zb_uint8_t hw_version;
    zb_char_t mf_name[32];
    zb_char_t model_id[32];
    zb_char_t date_code[16];
    zb_uint8_t power_source;
    zb_char_t location_id[17];
    zb_uint8_t ph_env;
    zb_char_t sw_ver[17];
} bulb_device_basic_attr_t;

typedef struct
{
    zb_uint16_t identify_time;
} bulb_device_identify_attr_t;

typedef struct
{
    zb_bool_t on_off;
    zb_bool_t global_scene_ctrl;
    zb_uint16_t on_time;
    zb_uint16_t off_wait_time;
} bulb_device_on_off_attr_t;

typedef struct
{
    zb_uint8_t current_level;
    zb_uint16_t remaining_time;
} bulb_device_level_control_attr_t;

typedef struct
{
    zb_uint8_t scene_count;
    zb_uint8_t current_scene;
    zb_uint8_t scene_valid;
    zb_uint8_t name_support;
    zb_uint16_t current_group;
} bulb_device_scenes_attr_t;

typedef struct
{
    zb_uint8_t name_support;
} bulb_device_groups_attr_t;

typedef struct
{
    uint8_t ep_id; // айди эндпоинта (10-13)
    uint8_t pwm_channel; // используемый канал ШИМ

    bulb_device_basic_attr_t basic_attr;
    bulb_device_identify_attr_t identify_attr;
    bulb_device_scenes_attr_t scenes_attr;
    bulb_device_groups_attr_t groups_attr;
    bulb_device_on_off_attr_t on_off_attr;
    bulb_device_level_control_attr_t level_control_attr;
} bulb_device_ctx_t;

// 4 структуры, в которых хранится полное состояние каждого из 4 эндпоинтов девайса (вкл/выкл, яркость и так далее).
static bulb_device_ctx_t m_dev_ctx_r;
static bulb_device_ctx_t m_dev_ctx_g;
static bulb_device_ctx_t m_dev_ctx_b;
static bulb_device_ctx_t m_dev_ctx_w;

// макрос, который описывает поддерживаемые эндпоинтом кластеры для управления им.
#define ZB_DECLARE_DIMMABLE_CONTROL_CLUSTER_LIST(dev_ctx_name, dimmable_light_bulb_cluster_list) \
    ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(dev_ctx_name##_identify_attr_list,                       \
        &dev_ctx_name.identify_attr.identify_time);                                              \
    ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(dev_ctx_name####_groups_attr_list,                         \
        &dev_ctx_name.groups_attr.name_support);                                                 \
    ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(dev_ctx_name##_scenes_attr_list,                           \
        &dev_ctx_name.scenes_attr.scene_count,                                                   \
        &dev_ctx_name.scenes_attr.current_scene,                                                 \
        &dev_ctx_name.scenes_attr.current_group,                                                 \
        &dev_ctx_name.scenes_attr.scene_valid,                                                   \
        &dev_ctx_name.scenes_attr.name_support);                                                 \
    ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(dev_ctx_name##_basic_attr_list,                         \
        &dev_ctx_name.basic_attr.zcl_version,                                                    \
        &dev_ctx_name.basic_attr.app_version,                                                    \
        &dev_ctx_name.basic_attr.stack_version,                                                  \
        &dev_ctx_name.basic_attr.hw_version,                                                     \
        dev_ctx_name.basic_attr.mf_name,                                                         \
        dev_ctx_name.basic_attr.model_id,                                                        \
        dev_ctx_name.basic_attr.date_code,                                                       \
        &dev_ctx_name.basic_attr.power_source,                                                   \
        dev_ctx_name.basic_attr.location_id,                                                     \
        &dev_ctx_name.basic_attr.ph_env,                                                         \
        dev_ctx_name.basic_attr.sw_ver);                                                         \
    ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST_EXT(dev_ctx_name##_on_off_attr_list,                       \
        &dev_ctx_name.on_off_attr.on_off,                                                        \
        &dev_ctx_name.on_off_attr.global_scene_ctrl,                                             \
        &dev_ctx_name.on_off_attr.on_time,                                                       \
        &dev_ctx_name.on_off_attr.off_wait_time);                                                \
    ZB_ZCL_DECLARE_LEVEL_CONTROL_ATTRIB_LIST_VA(dev_ctx_name##_level_control_attr_list,          \
        &dev_ctx_name.level_control_attr.current_level,                                          \
        &dev_ctx_name.level_control_attr.remaining_time);                                        \
    ZB_HA_DECLARE_DIMMABLE_LIGHT_CLUSTER_LIST(dimmable_light_bulb_cluster_list,                  \
        dev_ctx_name##_basic_attr_list,                                                          \
        dev_ctx_name##_identify_attr_list,                                                       \
        dev_ctx_name##_groups_attr_list,                                                         \
        dev_ctx_name##_scenes_attr_list,                                                         \
        dev_ctx_name##_on_off_attr_list,                                                         \
        dev_ctx_name##_level_control_attr_list);

// заполняем кластеры для каждого эндпоинта
ZB_DECLARE_DIMMABLE_CONTROL_CLUSTER_LIST(m_dev_ctx_r, m_dev_clusters_r);
ZB_DECLARE_DIMMABLE_CONTROL_CLUSTER_LIST(m_dev_ctx_g, m_dev_clusters_g);
ZB_DECLARE_DIMMABLE_CONTROL_CLUSTER_LIST(m_dev_ctx_b, m_dev_clusters_b);
ZB_DECLARE_DIMMABLE_CONTROL_CLUSTER_LIST(m_dev_ctx_w, m_dev_clusters_w);

// создаём 4 эндпоинта, указываем для них айди и кластеры
ZB_HA_DECLARE_LIGHT_EP(dimmable_light_ep_r, HA_DIMMABLE_LIGHT_ENDPOINT_R, m_dev_clusters_r);
ZB_HA_DECLARE_LIGHT_EP(dimmable_light_ep_g, HA_DIMMABLE_LIGHT_ENDPOINT_G, m_dev_clusters_g);
ZB_HA_DECLARE_LIGHT_EP(dimmable_light_ep_b, HA_DIMMABLE_LIGHT_ENDPOINT_B, m_dev_clusters_b);
ZB_HA_DECLARE_LIGHT_EP(dimmable_light_ep_w, HA_DIMMABLE_LIGHT_ENDPOINT_W, m_dev_clusters_w);

// полный список эндпоинтов, dimmable_light_ctx используется в ZB_AF_REGISTER_DEVICE_CTX() для регистрации
ZBOSS_DECLARE_DEVICE_CTX_EP_VA(dimmable_light_ctx, &dimmable_light_ep_r, &dimmable_light_ep_g, &dimmable_light_ep_b, &dimmable_light_ep_w);

// поиск контекста эндпоинта (где все настройки и состояние) по его айди. Эндпоинты с 10 по 13 для каждого цвета соответственно
bulb_device_ctx_t *find_ctx_by_ep_id(zb_uint8_t ep)
{
    if (ep == HA_DIMMABLE_LIGHT_ENDPOINT_R)
        return &m_dev_ctx_r;
    else if (ep == HA_DIMMABLE_LIGHT_ENDPOINT_G)
        return &m_dev_ctx_g;
    else if (ep == HA_DIMMABLE_LIGHT_ENDPOINT_B)
        return &m_dev_ctx_b;
    else if (ep == HA_DIMMABLE_LIGHT_ENDPOINT_W)
        return &m_dev_ctx_w;
    return NULL;
}

// инициализация таймеров
static void timer_init(void)
{
    uint32_t error_code = app_timer_init();
    APP_ERROR_CHECK(error_code);
}

// инициализация логгирования в уарт.
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

// функция прописывает яркость эндпоинта в нужном канале ШИМ
static void pwm_set_brightness(bulb_device_ctx_t *p_dimmable_light_ctx, zb_uint16_t brightness)
{
    uint16_t *p_channels = (uint16_t *)&m_led_values;

    p_channels[p_dimmable_light_ctx->pwm_channel] = brightness;
}

// устанавливает яркость для конкретного кананала, прописывает аттрибуты и прописывает ШИМ
static void level_control_set_value(bulb_device_ctx_t *p_dimmable_light_ctx, zb_uint16_t new_level)
{
    NRF_LOG_INFO("Set level value: %i for endpoint %hu", new_level, p_dimmable_light_ctx->ep_id);

    ZB_ZCL_SET_ATTRIBUTE(p_dimmable_light_ctx->ep_id, ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, (zb_uint8_t *)&new_level, ZB_FALSE);

    if (new_level == 0)
    {
        zb_uint8_t value = ZB_FALSE;
        ZB_ZCL_SET_ATTRIBUTE(p_dimmable_light_ctx->ep_id, ZB_ZCL_CLUSTER_ID_ON_OFF, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &value, ZB_FALSE);
    }
    else
    {
        zb_uint8_t value = ZB_TRUE;
        ZB_ZCL_SET_ATTRIBUTE(p_dimmable_light_ctx->ep_id, ZB_ZCL_CLUSTER_ID_ON_OFF, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &value, ZB_FALSE);
    }

    pwm_set_brightness(p_dimmable_light_ctx, p_dimmable_light_ctx->level_control_attr.current_level);
}

// включает/выключает канал (сохраняя старое значение при включении)
static void on_off_set_value(bulb_device_ctx_t *p_dimmable_light_ctx, zb_bool_t on)
{
    NRF_LOG_INFO("Set ON/OFF value: %i on endpoint: %hu", on, p_dimmable_light_ctx->ep_id);

    ZB_ZCL_SET_ATTRIBUTE(p_dimmable_light_ctx->ep_id, ZB_ZCL_CLUSTER_ID_ON_OFF, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, (zb_uint8_t *)&on, ZB_FALSE);

    if (on)
    {
        level_control_set_value(p_dimmable_light_ctx, p_dimmable_light_ctx->level_control_attr.current_level);
    }
    else
    {
        pwm_set_brightness(p_dimmable_light_ctx, 0);
    }
}

// обработчик нажатия клавиш на плате
static void buttons_handler(bsp_event_t evt)
{
    zb_ret_t zb_err_code;

    switch (evt)
    {
    case BSP_EVENT_KEY_0:
        if (m_dev_ctx_w.identify_attr.identify_time == ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE)
        {
            NRF_LOG_INFO("Bulb put in identifying mode");
            zb_err_code = zb_bdb_finding_binding_target(HA_DIMMABLE_LIGHT_ENDPOINT_W);
            ZB_ERROR_CHECK(zb_err_code);
        }
        else
        {
            NRF_LOG_INFO("Cancel F&B target procedure");
            zb_bdb_finding_binding_target_cancel();
        }
        break;

    default:
        NRF_LOG_INFO("Unhandled BSP Event received: %d", evt);
        break;
    }
}

// настройка портов и железа на плате
static void board_init(void)
{
    ret_code_t err_code;

    // настройка светиков и кнопок платы
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, buttons_handler);
    APP_ERROR_CHECK(err_code);

    // настройка ШИМ
    nrf_drv_pwm_config_t const led_pwm_config =
        {
            .output_pins =
                {
                    DIMMER_CHANNEL_PIN_R, // channel 0
                    DIMMER_CHANNEL_PIN_G, // channel 1
                    DIMMER_CHANNEL_PIN_B, // channel 2
                    DIMMER_CHANNEL_PIN_W, // channel 3
                },
            .irq_priority = APP_IRQ_PRIORITY_LOWEST,
            .base_clock = NRF_PWM_CLK_125kHz,
            .count_mode = NRF_PWM_MODE_UP,
            .top_value = DIMMER_PWM_VALUE_MAX,
            .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
            .step_mode = NRF_PWM_STEP_AUTO};

    err_code = nrf_drv_pwm_init(&m_led_pwm, &led_pwm_config, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_pwm_simple_playback(&m_led_pwm, &m_led_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    APP_ERROR_CHECK(err_code);
}

// заполнение настроек кластеров эндпоинта
static void dimmable_clusters_attr_init(bulb_device_ctx_t *p_dimmable_light_ctx)
{
    p_dimmable_light_ctx->basic_attr.zcl_version = ZB_ZCL_VERSION;
    p_dimmable_light_ctx->basic_attr.app_version = BULB_INIT_BASIC_APP_VERSION;
    p_dimmable_light_ctx->basic_attr.stack_version = BULB_INIT_BASIC_STACK_VERSION;
    p_dimmable_light_ctx->basic_attr.hw_version = BULB_INIT_BASIC_HW_VERSION;

    ZB_ZCL_SET_STRING_VAL(p_dimmable_light_ctx->basic_attr.mf_name, BULB_INIT_BASIC_MANUF_NAME, ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MANUF_NAME));
    ZB_ZCL_SET_STRING_VAL(p_dimmable_light_ctx->basic_attr.model_id, BULB_INIT_BASIC_MODEL_ID, ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MODEL_ID));
    ZB_ZCL_SET_STRING_VAL(p_dimmable_light_ctx->basic_attr.date_code, BULB_INIT_BASIC_DATE_CODE, ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_DATE_CODE));

    p_dimmable_light_ctx->basic_attr.power_source = BULB_INIT_BASIC_POWER_SOURCE;

    ZB_ZCL_SET_STRING_VAL(p_dimmable_light_ctx->basic_attr.location_id, BULB_INIT_BASIC_LOCATION_DESC, ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_LOCATION_DESC));

    p_dimmable_light_ctx->basic_attr.ph_env = BULB_INIT_BASIC_PH_ENV;

    p_dimmable_light_ctx->identify_attr.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

    p_dimmable_light_ctx->on_off_attr.on_off = (zb_bool_t)ZB_ZCL_ON_OFF_IS_ON;
    p_dimmable_light_ctx->on_off_attr.global_scene_ctrl = ZB_TRUE;
    p_dimmable_light_ctx->on_off_attr.on_time = 0;
    p_dimmable_light_ctx->on_off_attr.off_wait_time = 0;

    p_dimmable_light_ctx->level_control_attr.current_level = ZB_ZCL_LEVEL_CONTROL_LEVEL_MAX_VALUE;
    p_dimmable_light_ctx->level_control_attr.remaining_time = ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;

    ZB_ZCL_SET_ATTRIBUTE(p_dimmable_light_ctx->ep_id, ZB_ZCL_CLUSTER_ID_ON_OFF, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, (zb_uint8_t *)&p_dimmable_light_ctx->on_off_attr.on_off, ZB_FALSE);
    ZB_ZCL_SET_ATTRIBUTE(p_dimmable_light_ctx->ep_id, ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, (zb_uint8_t *)&p_dimmable_light_ctx->level_control_attr.current_level, ZB_FALSE);
}

// инициализация контекста эндпоинта
void zb_dimmable_light_init_ctx(bulb_device_ctx_t *p_dimmable_light_ctx, uint8_t ep_id, uint8_t pwm_channel)
{
    memset(p_dimmable_light_ctx, 0, sizeof(bulb_device_ctx_t));

    p_dimmable_light_ctx->ep_id = ep_id;
    p_dimmable_light_ctx->pwm_channel = pwm_channel;

    dimmable_clusters_attr_init(p_dimmable_light_ctx);
    level_control_set_value(p_dimmable_light_ctx, ZB_ZCL_LEVEL_CONTROL_LEVEL_MAX_VALUE);
}

zb_void_t zb_osif_go_idle(zb_void_t)
{
    zb_osif_wait_for_event();
}

// считывает атрибуты из поступившего сообщения и устанавливает яркость или включает-выключает канал.
zb_ret_t zb_dimmer_light_set_attribute(bulb_device_ctx_t *p_dimmable_light_ctx, zb_zcl_set_attr_value_param_t *p_savp)
{
    zb_ret_t ret = RET_NOT_IMPLEMENTED;

    if (p_savp->cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF)
    {
        if (p_savp->attr_id == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)
        {
            uint8_t value = p_savp->values.data8;
            on_off_set_value(p_dimmable_light_ctx, (zb_bool_t)value);
            ret = RET_OK;
            NRF_LOG_INFO("on/off attribute setting to %hd", value);
        }
    }
    else if (p_savp->cluster_id == ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL)
    {
        if (p_savp->attr_id == ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID)
        {
            uint16_t value = p_savp->values.data16;
            level_control_set_value(p_dimmable_light_ctx, value);
            ret = RET_OK;
            NRF_LOG_INFO("level control attribute setting to %hd", value);
        }
    }
    else
    {
        NRF_LOG_INFO("Unhandled cluster attribute id: %d", p_savp->cluster_id);
    }

    return ret;
}

// обработчик событий, вызываемый при изменении аттрибутов ZCL у эндпоинтов
static zb_void_t zcl_device_cb(zb_uint8_t param)
{
    zb_uint8_t cluster_id;
    zb_uint8_t attr_id;
    zb_buf_t *p_buffer = ZB_BUF_FROM_REF(param);
    zb_zcl_device_callback_param_t *p_device_cb_param = ZB_GET_BUF_PARAM(p_buffer, zb_zcl_device_callback_param_t);

    NRF_LOG_INFO("Received ZCL callback %hd on endpoint %hu", p_device_cb_param->device_cb_id, p_device_cb_param->endpoint);

    bulb_device_ctx_t *p_dimmable_light_ctx = find_ctx_by_ep_id(p_device_cb_param->endpoint);
    if (!p_dimmable_light_ctx)
    {
        NRF_LOG_WARNING("Context for endpoint %hu not found", p_device_cb_param->endpoint);
        return;
    }

    p_device_cb_param->status = RET_OK;

    switch (p_device_cb_param->device_cb_id)
    {
    case ZB_ZCL_LEVEL_CONTROL_SET_VALUE_CB_ID:
        NRF_LOG_INFO("Level control setting to %d", p_device_cb_param->cb_param.level_control_set_value_param.new_value);
        level_control_set_value(p_dimmable_light_ctx, p_device_cb_param->cb_param.level_control_set_value_param.new_value);
        break;

    case ZB_ZCL_SET_ATTR_VALUE_CB_ID:
        zb_dimmer_light_set_attribute(p_dimmable_light_ctx, &p_device_cb_param->cb_param.set_attr_value_param);
        break;

    default:
        p_device_cb_param->status = RET_ERROR;
        break;
    }

    NRF_LOG_INFO("zcl_device_cb status: %hd", p_device_cb_param->status);
}

// обработчик основных событий в стеке зигби - старт, стоп, наличие проблем и вот это вот всё.
void zboss_signal_handler(zb_uint8_t param)
{
    zb_zdo_app_signal_hdr_t *p_sg_p = NULL;
    zb_zdo_app_signal_type_t sig = zb_get_app_signal(param, &p_sg_p);
    zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(param);
    zb_bool_t comm_status;

    switch (sig)
    {
    case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (status == RET_OK)
        {
            NRF_LOG_INFO("Joined network successfully");
            bsp_board_led_on(ZIGBEE_NETWORK_STATE_LED);
        }
        else
        {
            NRF_LOG_ERROR("Failed to join network. Status: %d", status);
            bsp_board_led_off(ZIGBEE_NETWORK_STATE_LED);
            comm_status = bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
            ZB_COMM_STATUS_CHECK(comm_status);
        }
        break;

    case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        if (status != RET_OK)
        {
            NRF_LOG_WARNING("Production config is not present or invalid");
        }
        break;

    case ZB_ZDO_SIGNAL_LEAVE:
        if (status == RET_OK)
        {
            bsp_board_led_off(ZIGBEE_NETWORK_STATE_LED);

            zb_zdo_signal_leave_params_t *p_leave_params = ZB_ZDO_SIGNAL_GET_PARAMS(p_sg_p, zb_zdo_signal_leave_params_t);
            NRF_LOG_INFO("Network left. Leave type: %d", p_leave_params->leave_type);
        }
        else
        {
            NRF_LOG_ERROR("Unable to leave network. Status: %d", status);
        }
        break;

    case ZB_ZDO_SIGNAL_SKIP_STARTUP:
        NRF_LOG_INFO("ZB_ZDO_SIGNAL_SKIP_STARTUP: %d", status);
        break;
    case ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        NRF_LOG_INFO("ZB_ZDO_SIGNAL_DEVICE_ANNCE: %d", status);
        break;
    case ZB_BDB_SIGNAL_TOUCHLINK_NWK_JOINED_ROUTER:
        NRF_LOG_INFO("ZB_BDB_SIGNAL_TOUCHLINK_NWK_JOINED_ROUTER: %d", status);
        break;
    case ZB_BDB_SIGNAL_TOUCHLINK_NWK_STARTED:
        NRF_LOG_INFO("ZB_BDB_SIGNAL_TOUCHLINK_NWK_STARTED: %d", status);
        break;
    case ZB_BDB_SIGNAL_TOUCHLINK:
        NRF_LOG_INFO("ZB_BDB_SIGNAL_TOUCHLINK: %d", status);
        break;
    case ZB_BDB_SIGNAL_STEERING:
        NRF_LOG_INFO("ZB_BDB_SIGNAL_STEERING: %d", status);
        break;
    case ZB_BDB_SIGNAL_FORMATION:
        NRF_LOG_INFO("ZB_BDB_SIGNAL_FORMATION: %d", status);
        break;
    case ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED:
        NRF_LOG_INFO("ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED: %d", status);
        break;
    case ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED:
        NRF_LOG_INFO("ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED: %d", status);
        break;
    case ZB_BDB_SIGNAL_TOUCHLINK_TARGET:
        NRF_LOG_INFO("ZB_BDB_SIGNAL_TOUCHLINK_TARGET: %d", status);
        break;
    case ZB_BDB_SIGNAL_TOUCHLINK_NWK:
        NRF_LOG_INFO("ZB_BDB_SIGNAL_TOUCHLINK_NWK: %d", status);
        break;
    case ZB_ZDO_SIGNAL_LEAVE_INDICATION:
        NRF_LOG_INFO("ZB_ZDO_SIGNAL_LEAVE_INDICATION: %d", status);
        break;
    case ZB_ZGP_SIGNAL_COMMISSIONING:
        NRF_LOG_INFO("ZB_ZGP_SIGNAL_COMMISSIONING: %d", status);
        break;
    case ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        NRF_LOG_INFO("ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT: %d", status);
        break;

    case ZB_COMMON_SIGNAL_CAN_SLEEP:
        NRF_LOG_INFO("ZB_COMMON_SIGNAL_CAN_SLEEP: %d", status);
        break;

    default:
        NRF_LOG_INFO("Unhandled signal %d. Status: %d", sig, status);
        break;
    }

    if (param)
    {
        ZB_FREE_BUF_BY_REF(param);
    }
}

int main(void)
{
    zb_ret_t zb_err_code;
    zb_ieee_addr_t ieee_addr; // здесь будет длинный адрес ноды

    timer_init(); // запускаем таймеры, без них ничего работать не будет :)
    log_init(); // инициализация движка логгирования (в последовательный порт в нашем случае)
    board_init(); // инициализация платы

    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL); // настройки логгирования (как много писать в логи)
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    ZB_INIT("dimmer_rgbw");

    zb_osif_get_ieee_eui64(ieee_addr); // читаем полный адрес из памяти
    zb_set_long_address(ieee_addr); // и устанавливаем его

    zb_set_network_router_role(IEEE_CHANNEL_MASK); // роутером будем
    zb_set_max_children(MAX_CHILDREN); // максимальное количество дочерних девайсов у роутера
    zigbee_erase_persistent_storage(ERASE_PERSISTENT_CONFIG); // очищать или нет конфиг при старте
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));

    ZB_AF_REGISTER_DEVICE_CTX(&dimmable_light_ctx); // регистрируем список эндпоинтов в стеке

    ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb); // коллбэк-функция, которая будет вызываться при изменении состояния эндпоинтов

    zb_dimmable_light_init_ctx(&m_dev_ctx_r, HA_DIMMABLE_LIGHT_ENDPOINT_R, 0); // инициализируем каждый из 4 эндпоинтов, указываем номер канала ШИМ
    zb_dimmable_light_init_ctx(&m_dev_ctx_g, HA_DIMMABLE_LIGHT_ENDPOINT_G, 1);
    zb_dimmable_light_init_ctx(&m_dev_ctx_b, HA_DIMMABLE_LIGHT_ENDPOINT_B, 2);
    zb_dimmable_light_init_ctx(&m_dev_ctx_w, HA_DIMMABLE_LIGHT_ENDPOINT_W, 3);

    nrf_802154_tx_power_set(8);

    zb_err_code = zboss_start(); // запуск стека зигби
    ZB_ERROR_CHECK(zb_err_code);

    while (1) // вечный цикл, в котором крутится вся жизнь приложения
    {
        zboss_main_loop_iteration();
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    }
}