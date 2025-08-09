extern "C"{
    #include <stdio.h>
    #include <string.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/timers.h" //timers (considerar hacerlos manuales)
    #include "driver/spi_master.h" //spi
    #include "esp_log.h" 
    #include "driver/adc.h" // joystick con driver/adc.h
    #include "driver/gpio.h" // Para el botón del joystick
}

//------------------ DEBUG ----------------------
#define DEBUG_SPI 0
#define DEBUG_PANTALLA 0
#define DEBUG_RENDIMIENTO 0

//Estilos ------------------- [SOLO SELECCIONE UNO]
#define INTERFAZ_COLOR_AZUL 0
#define INTERFAZ_COLOR_ROJO 1
#define INTERFAZ_COLOR_MORADO 0


//SPI PINES
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

//Joystick pines
#define JOY_X_PIN 34  // Pin para VRX (Eje X)
#define JOY_Y_PIN 35  // Pin para VRY (Eje Y)
#define JOY_SW_PIN GPIO_NUM_33 // Pin para SW (botón)

//PANTALLA
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <fondo.cpp>

TFT_eSPI tft = TFT_eSPI();  // Usa el setup del archivo User_Setup.h

#include <InverseKinematics.h> // Inverse Kinematics


//LVGL ----------------------------------

static const uint16_t screenWidth = 280;
static const uint16_t screenHeight = 240;


static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[screenWidth * 20];
static lv_color_t buf2[screenWidth * 20];
static lv_disp_drv_t disp_drv;


// --------------------------------------


static const char* TAG = "SPI_Master";

// Estructura de datos a enviar
typedef struct {
    float anguloX;
    float anguloY;
    float anguloZ;
    int16_t apagado; 
    int16_t var_control; 
} Angulos;

// Estructura de respuesta
typedef struct {
    int16_t touchX;
    int16_t touchY;
    float encoderX;
    float encoderY;
    float encoderZ;
} Respuesta;


//Variables globales
spi_device_handle_t spi_com;
Respuesta datos_display;
Angulos datosParaEnviar = {0, 0, 0, 1, 0};  // o valores iniciales
//X, Y, Z, apagado, varcontrol

// --------------------------------------  Pantalla -----------------------------------

// Comandos UI para la cola de mensajes
typedef enum {
    UI_CMD_NONE = 0,
    UI_CMD_NAVIGATE_UP,
    UI_CMD_NAVIGATE_DOWN,
    UI_CMD_NAVIGATE_LEFT,
    UI_CMD_NAVIGATE_RIGHT,
    UI_CMD_SELECT,
    UI_CMD_JOYSTICK_CENTER  
} ui_cmd_t;

typedef struct {
    ui_cmd_t cmd;
    // Puedes añadir más parámetros si es necesario
} ui_msg_t;


enum Pantalla {
    MENU_PRINCIPAL,
    METODO_CONTROL,
    MODO_MANUAL,
    DATOS_VARIABLES,
    GRAFICAS,
    APAGADO
};
Pantalla pantalla_actual = MENU_PRINCIPAL;



//joystick values ----- 

int joystick_x  = 0;
int joystick_y  = 0;
bool boton_presionado = false;
bool boton_anterior = false;  // Nuevo: para detectar flanco

typedef enum {
    JOY_CENTER,
    JOY_UP,
    JOY_DOWN,
    JOY_LEFT,
    JOY_RIGHT
} joystick_direction_t;

joystick_direction_t joystick_pos; // global o estática


// -------------------- MENU --------------------fondo1

QueueHandle_t ui_msg_queue;
SemaphoreHandle_t xGuiSemaphore;
lv_obj_t *label;  // Global para modificarlo desde tareas

// MENU PRINCIPAL -------------------
lv_obj_t *menu_list;
lv_obj_t *btn_opciones[5];
int opcion_actual = 0;


lv_obj_t *btn_volver;

//ESTILOS --------------------------
static lv_style_t estilo_focused;
static lv_style_t estilo_default;
static lv_style_t spinbox_estilo_focused;
static lv_style_t spinbox_estilo_default;
static lv_style_t estilo_contenedor_oscuro;
static lv_style_t estilo_outline;

//METEODO DE CONTROL ------------------------
int metodo_control_sw = 0;
lv_obj_t *btn_metctrl[4];

//Datos Variables ------------------
lv_obj_t* label_encoder_x = nullptr;
lv_obj_t* label_encoder_y = nullptr;
lv_obj_t* label_encoder_z = nullptr;
lv_obj_t* label_touch_x = nullptr;
lv_obj_t* label_touch_y = nullptr;
lv_timer_t* timer_enc_touch = NULL;

//MODO MANUAL -------------
lv_obj_t* spinboxes[3];
lv_obj_t *btn_plus[3];
lv_obj_t *btn_minus[3];
int spinbox_index_actual = 0;

//GRAFICAS -------------
const int NUM_SUBMENUS = 2;
int submenu_actual = 0;
lv_obj_t * btn_prev;
lv_obj_t * btn_next;
lv_timer_t * chart_timer = nullptr;
lv_timer_t * circulo_timer = nullptr;
int opcion_actual2 = 0;


//-------------------------- PID --------------------------------------
float Kpa = 0.2, Kia = 0.1, Kda = 0.3;
float Kpb = 0.05, Kib = 0.07, Kdb = 0.2;
float Kpc = 0.05, Kic = 0.07, Kdc = 0.2;

float previous_error = 0;
float integral = 0;



// ------------------------- COMUNICACIÓN SPI -------------------------

// Inicializar SPI
void init_spi() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 4096,
        .flags = 0,
        .intr_flags = 0
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 1000000,
        .input_delay_ns = 0,
        .spics_io_num = PIN_NUM_CS,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };

    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi_com));
}

//Start byte para el arduino
void enviar_start_byte() {
    uint8_t start_byte = 0xA5;
    spi_transaction_t t_start = {};
    t_start.length = 8;
    t_start.tx_buffer = &start_byte;
    ESP_ERROR_CHECK(spi_device_transmit(spi_com, &t_start));
    //ESP_LOGI(TAG, "START_BYTE enviado: 0x%02X", start_byte);
}

//Envío de datos SPI
void enviar_datos(Angulos* datos) {
    uint8_t* ptr = (uint8_t*)datos;
    for (size_t i = 0; i < sizeof(Angulos); i++) {
        spi_transaction_t t_data = {};
        t_data.length = 8;
        t_data.tx_buffer = &ptr[i];
        ESP_ERROR_CHECK(spi_device_transmit(spi_com, &t_data));
        //ESP_LOGI(TAG, "Dato enviado: 0x%02X", ptr[i]);
    }
}

//Esperar el ready byte del arduino
bool esperar_ready_byte() {
    uint8_t dummy_tx = 0x00;
    uint8_t respuesta = 0x00;
    int timeout_ms = 3000;
    int elapsed = 0;

    while (respuesta != 0xAA && elapsed < timeout_ms) {
        spi_transaction_t t_ready = {};
        t_ready.length = 8;
        t_ready.tx_buffer = &dummy_tx;
        t_ready.rx_buffer = &respuesta;
        ESP_ERROR_CHECK(spi_device_transmit(spi_com, &t_ready));

        if (respuesta == 0xAA) {
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed += 10;
    }

    return false;
}

//Lee la respuesta del arduino
void leer_respuesta(Respuesta* resp) {
    uint8_t dummy_tx = 0x00;
    uint8_t* ptrResp = (uint8_t*)resp;

    for (size_t i = 0; i < sizeof(Respuesta); i++) {
        spi_transaction_t t_resp = {};
        t_resp.length = 8;
        t_resp.tx_buffer = &dummy_tx;
        t_resp.rx_buffer = &ptrResp[i];
        ESP_ERROR_CHECK(spi_device_transmit(spi_com, &t_resp));
    }
    //ESP_LOGI(TAG, "X: %d, Y: %d", resp->touchX, resp->touchY);
    #if DEBUG_SPI
    ESP_LOGI(TAG, "X: %d, Y: %d", resp->touchX, resp->touchY);
    ESP_LOGI(TAG, "EncoderX: %d, EncoderY: %d, EncoderZ: %d", resp->encoderX, resp->encoderY, resp->encoderZ);
    ESP_LOGI(TAG, "EncoderX: %.2f, EncoderY: %.2f, EncoderZ: %.2f", resp->encoderX, resp->encoderY, resp->encoderZ);
    #endif
}

//COMUNICACIÓN SPI
void comunicacion_spi(Angulos* datos) {
    enviar_start_byte();
    if(datos->anguloX > 90) datos->anguloX = 90;
    if(datos->anguloY > 90) datos->anguloY = 90;
    if(datos->anguloZ > 90) datos->anguloZ = 90;

    if(datos->anguloX < 0 && datos->var_control == 0) datos->anguloX = 0;
    if(datos->anguloY < 0 && datos->var_control == 0) datos->anguloY = 0;
    if(datos->anguloZ < 0 && datos->var_control == 0) datos->anguloZ = 0;

    enviar_datos(datos);

    if (esperar_ready_byte()) {
        //ESP_LOGI(TAG, "READY_BYTE recibido, leyendo respuesta...");
        Respuesta resp;
        leer_respuesta(&resp);
        datos_display = resp;  // <-- aquí actualizas la variable global

    } 
    //else ESP_LOGI(TAG, "No se recibió READY_BYTE a tiempo.");
    
    
    //MODO MANUAL
    ESP_LOGI(TAG, "X:  %.2f", datosParaEnviar.anguloX);

    switch (pantalla_actual)
    {
    case MODO_MANUAL:
        if (spinboxes[0] != nullptr && spinboxes[1] != nullptr && spinboxes[2] != nullptr) {
            datosParaEnviar.anguloX = lv_spinbox_get_value(spinboxes[0]);
            datosParaEnviar.anguloY = lv_spinbox_get_value(spinboxes[1]);
            datosParaEnviar.anguloZ = lv_spinbox_get_value(spinboxes[2]);
        } 

        #if DEBUG_PANTALLA
        else {
            ESP_LOGE("SPI", "mi_spinbox es nullptr");
        }
        #endif
        break;

    default:
        break;
    }
}


//TASK
void tarea_spi(void *pvParameters) {
    while (1) {
        comunicacion_spi(&datosParaEnviar);
        vTaskDelay(pdMS_TO_TICKS(15));  // Llama cada 10 ms
    }
}



//----------------------------- JOYSTICK -------------------------------------

// Inicializar el ADC (para versiones recientes de ESP-IDF)
void init_joystick() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12); // GPIO34
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12); // GPIO35

    gpio_set_direction(JOY_SW_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(JOY_SW_PIN, GPIO_PULLUP_ONLY);
}

joystick_direction_t get_joystick_direction(int x, int y) {
    const int deadzone = 700;
    const int center_x = 1900;
    const int center_y = 1900;

    if (x < center_x - deadzone) return JOY_DOWN;
    if (x > center_x + deadzone) return JOY_UP;
    if (y < center_y - deadzone) return JOY_LEFT;
    if (y > center_y + deadzone) return JOY_RIGHT;
    return JOY_CENTER;
}

String direction_to_string(joystick_direction_t dir) {
    switch (dir) {
      case JOY_UP: return "UP    ";
      case JOY_DOWN: return "DOWN  ";
      case JOY_LEFT: return "LEFT  ";
      case JOY_RIGHT: return "RIGHT ";
      case JOY_CENTER: return "CENTER";
      default: return "";
    }
  }

void leer_joystick() {
    joystick_x = adc1_get_raw(ADC1_CHANNEL_6); // GPIO34
    joystick_y = adc1_get_raw(ADC1_CHANNEL_7); // GPIO35
    bool estado_actual  = (gpio_get_level(JOY_SW_PIN) == 0);
    boton_presionado = (estado_actual && !boton_anterior); // Solo si antes estaba suelto
    boton_anterior = estado_actual; // Actualizamos para la siguiente llamada
    joystick_pos = get_joystick_direction(joystick_x, joystick_y);

}


void tarea_joystick(void *pvParameters) {
    joystick_direction_t prev_direction = JOY_CENTER;
    TickType_t last_repeat_time = 0;
    const TickType_t repeat_delay = pdMS_TO_TICKS(800);   // Retardo inicial
    const TickType_t repeat_rate  = pdMS_TO_TICKS(200);   // Intervalo entre repeticiones

    while (1) {
        leer_joystick();

        ui_msg_t msg = {};
        TickType_t now = xTaskGetTickCount();

        // Enviar si hay cambio o si se mantiene presionado lo suficiente
        if (joystick_pos != JOY_CENTER) {
            bool send = false;

            if (joystick_pos != prev_direction) {
                send = true;
                last_repeat_time = now + repeat_delay;  // Espera inicial antes de repetir
            } else if (now >= last_repeat_time) {
                send = true;
                last_repeat_time = now + repeat_rate;   // Repetir cada cierto tiempo
            }

            if (send) {
                switch (joystick_pos) {
                    case JOY_UP:    msg.cmd = UI_CMD_NAVIGATE_UP;   break;
                    case JOY_DOWN:  msg.cmd = UI_CMD_NAVIGATE_DOWN; break;
                    case JOY_LEFT:  msg.cmd = UI_CMD_NAVIGATE_LEFT; break;
                    case JOY_RIGHT: msg.cmd = UI_CMD_NAVIGATE_RIGHT;break;
                    default: break;
                }

                if (msg.cmd != 0) {
                    xQueueSend(ui_msg_queue, &msg, portMAX_DELAY);
                }
            }
        }
        else if (prev_direction != JOY_CENTER) {
            // El joystick volvió al centro: notificarlo
            msg.cmd = UI_CMD_JOYSTICK_CENTER;
            xQueueSend(ui_msg_queue, &msg, portMAX_DELAY);
        }

        if (boton_presionado) {
            msg.cmd = UI_CMD_SELECT;
            xQueueSend(ui_msg_queue, &msg, portMAX_DELAY);
        }

        prev_direction = joystick_pos;
        vTaskDelay(pdMS_TO_TICKS(80)); // Mayor sensibilidad de lectura
    }
}


//----------------- PANTALLA ---------------------------

//----------- Display Flush Callback -------------
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushPixels((uint16_t *)color_p, w * h);  // color_p ya es lv_color_t*
    tft.endWrite();

    #if DEBBUG_PANTALLA
    ESP_LOGI("LVGL", "Flush called: %dx%d", w, h);  // <--- Añade esto
    #endif

    lv_disp_flush_ready(disp);  // Indica a LVGL que terminó el dibujado
}


//----------- LVGL Setup -------------


// RESET MOTOR -----------------
void reset_motor_pos(){
    datosParaEnviar.anguloX = 0;
    datosParaEnviar.anguloY = 0;
    datosParaEnviar.anguloZ = 0;
}



void lvgl_setup() {
    lv_init();

    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, screenWidth * 20);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

}



void init_estilos() {
    // Estilo default (puedes personalizarlo si deseas)
        // Estilo default mejorado - Botón negro estético
    lv_style_init(&estilo_default);
    lv_style_set_bg_color(&estilo_default, lv_color_hex(0x1E1E1E));           // Fondo negro oscuro
    lv_style_set_bg_grad_color(&estilo_default, lv_color_hex(0x2C2C2C));      // Gradiente hacia gris oscuro
    lv_style_set_bg_grad_dir(&estilo_default, LV_GRAD_DIR_VER);              // Dirección del gradiente vertical
    lv_style_set_bg_opa(&estilo_default, LV_OPA_COVER);

    lv_style_set_radius(&estilo_default, 8);                                  // Bordes redondeados
    lv_style_set_border_width(&estilo_default, 2);
    lv_style_set_border_color(&estilo_default, lv_color_hex(0x444444));       // Borde gris oscuro

    lv_style_set_shadow_width(&estilo_default, 10);                           // Sombra alrededor del botón
    lv_style_set_shadow_color(&estilo_default, lv_color_hex(0x000000));       // Color de la sombra
    lv_style_set_shadow_ofs_x(&estilo_default, 2);                            // Desplazamiento X
    lv_style_set_shadow_ofs_y(&estilo_default, 4);                            // Desplazamiento Y

    lv_style_set_text_color(&estilo_default, lv_color_hex(0xFFFFFF));         // Texto blanco
    lv_style_set_outline_width(&estilo_default, 0);
    
    lv_style_set_text_font(&estilo_default, &lv_font_montserrat_14);

    #if INTERFAZ_COLOR_ROJO
    // -------------------- Estilo focused ------------------------
    lv_style_init(&estilo_focused);
    lv_style_set_bg_color(&estilo_focused, lv_color_hex(0xFF3B3B));           // Rojo fuerte
    lv_style_set_bg_grad_color(&estilo_focused, lv_color_hex(0xD10000));
    lv_style_set_bg_grad_dir(&estilo_focused, LV_GRAD_DIR_VER);
    lv_style_set_bg_opa(&estilo_focused, LV_OPA_COVER);

    lv_style_set_radius(&estilo_focused, 8);
    lv_style_set_border_width(&estilo_focused, 2);
    lv_style_set_border_color(&estilo_focused, lv_color_hex(0x880000));

    lv_style_set_shadow_width(&estilo_focused, 12);
    lv_style_set_shadow_color(&estilo_focused, lv_color_hex(0x550000));
    lv_style_set_shadow_ofs_x(&estilo_focused, 3);
    lv_style_set_shadow_ofs_y(&estilo_focused, 6);

    lv_style_set_text_color(&estilo_focused, lv_color_hex(0xFFFFFF));

    lv_style_set_text_font(&estilo_focused, &lv_font_montserrat_14);
    #endif

    #if INTERFAZ_COLOR_AZUL
    // -------------------- Estilo focused azul ------------------------
    lv_style_init(&estilo_focused);
    lv_style_set_bg_color(&estilo_focused, lv_color_hex(0x3B6EFF));           // Azul fuerte
    lv_style_set_bg_grad_color(&estilo_focused, lv_color_hex(0x003D99));      // Azul más oscuro
    lv_style_set_bg_grad_dir(&estilo_focused, LV_GRAD_DIR_VER);
    lv_style_set_bg_opa(&estilo_focused, LV_OPA_COVER);

    lv_style_set_radius(&estilo_focused, 8);
    lv_style_set_border_width(&estilo_focused, 2);
    lv_style_set_border_color(&estilo_focused, lv_color_hex(0x002266));       // Borde azul oscuro

    lv_style_set_shadow_width(&estilo_focused, 12);
    lv_style_set_shadow_color(&estilo_focused, lv_color_hex(0x001133));       // Sombra azul oscuro
    lv_style_set_shadow_ofs_x(&estilo_focused, 3);
    lv_style_set_shadow_ofs_y(&estilo_focused, 6);

    lv_style_set_text_color(&estilo_focused, lv_color_hex(0xFFFFFF));
    lv_style_set_text_font(&estilo_focused, &lv_font_montserrat_14);
    #endif

    #if INTERFAZ_COLOR_MORADO
    // -------------------- Estilo focused morado ------------------------
    lv_style_init(&estilo_focused);
    lv_style_set_bg_color(&estilo_focused, lv_color_hex(0xA64EFF));           // Morado brillante
    lv_style_set_bg_grad_color(&estilo_focused, lv_color_hex(0x5D00A6));      // Morado oscuro
    lv_style_set_bg_grad_dir(&estilo_focused, LV_GRAD_DIR_VER);
    lv_style_set_bg_opa(&estilo_focused, LV_OPA_COVER);

    lv_style_set_radius(&estilo_focused, 8);
    lv_style_set_border_width(&estilo_focused, 2);
    lv_style_set_border_color(&estilo_focused, lv_color_hex(0x3A0066));       // Borde morado profundo

    lv_style_set_shadow_width(&estilo_focused, 12);
    lv_style_set_shadow_color(&estilo_focused, lv_color_hex(0x220033));       // Sombra morado muy oscuro
    lv_style_set_shadow_ofs_x(&estilo_focused, 3);
    lv_style_set_shadow_ofs_y(&estilo_focused, 6);

    lv_style_set_text_color(&estilo_focused, lv_color_hex(0xFFFFFF));         // Texto blanco
    lv_style_set_text_font(&estilo_focused, &lv_font_montserrat_14);

    #endif

    //--------------------Spinbox Style focusedd-------------------

    lv_style_init(&spinbox_estilo_focused);
    lv_style_set_border_color(&spinbox_estilo_focused, lv_palette_main(LV_PALETTE_BLUE));
    

    lv_style_init(&spinbox_estilo_default);
    lv_style_set_border_color(&spinbox_estilo_default, lv_color_hex(0xe2e3e2));


    //---------- Contenedor style ----------------------------

    lv_style_init(&estilo_contenedor_oscuro);
    lv_style_set_bg_color(&estilo_contenedor_oscuro, lv_color_hex(0x1A1A1A));       // Fondo negro suave
    //lv_style_set_bg_grad_color(&estilo_contenedor_oscuro, lv_color_hex(0x2A2A2A));  // Gradiente muy sutil
    //lv_style_set_bg_grad_dir(&estilo_contenedor_oscuro, LV_GRAD_DIR_VER);
    lv_style_set_bg_opa(&estilo_contenedor_oscuro, LV_OPA_COVER);

    lv_style_set_radius(&estilo_contenedor_oscuro, 10);                             // Bordes redondeados
    lv_style_set_border_width(&estilo_contenedor_oscuro, 2);
    lv_style_set_border_color(&estilo_contenedor_oscuro, lv_color_hex(0x444444));   // Gris oscuro

    lv_style_set_shadow_width(&estilo_contenedor_oscuro, 12);
    lv_style_set_shadow_color(&estilo_contenedor_oscuro, lv_color_hex(0x000000));
    lv_style_set_shadow_ofs_x(&estilo_contenedor_oscuro, 2);
    lv_style_set_shadow_ofs_y(&estilo_contenedor_oscuro, 6);

    lv_style_set_pad_all(&estilo_contenedor_oscuro, 10);                            // Relleno interior
    lv_style_set_text_color(&estilo_contenedor_oscuro, lv_color_hex(0xFFFFFF));     // Texto blanco


    // ------------- Outline Style -------------------------------
    #if INTERFAZ_COLOR_ROJO
    lv_style_init(&estilo_outline);
    lv_style_set_outline_width(&estilo_outline, 2);
    lv_style_set_outline_color(&estilo_outline, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_outline_pad(&estilo_outline, 4);
    #endif

    #if INTERFAZ_COLOR_AZUL
    lv_style_init(&estilo_outline);
    lv_style_set_outline_width(&estilo_outline, 2);
    lv_style_set_outline_color(&estilo_outline, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_outline_pad(&estilo_outline, 4);
    #endif

    #if INTERFAZ_COLOR_MORADO
    lv_style_init(&estilo_outline);
    lv_style_set_outline_width(&estilo_outline, 2);
    lv_style_set_outline_color(&estilo_outline, lv_palette_main(LV_PALETTE_PURPLE));
    lv_style_set_outline_pad(&estilo_outline, 4);
    #endif

}

// FUNCION PARA ACTUALIZAR ESTADOS
void actualizar_seleccion() {
    switch (pantalla_actual)
    {
    case MENU_PRINCIPAL:
        for (int i = 0; i < 5; i++) {
            // Aplica el estilo correspondiente
            if (i == opcion_actual) {
                lv_obj_add_style(btn_opciones[i], &estilo_focused, 0);
            } else {
                lv_obj_add_style(btn_opciones[i], &estilo_default, 0);
            }
    }
        break;

    case METODO_CONTROL:

        if(opcion_actual < 4){
            lv_obj_add_style(btn_volver, &estilo_default, 0);
            for (int i = 0; i < 4; i++) {

                if (i == opcion_actual) lv_obj_add_style(btn_metctrl[i], &estilo_focused, 0);
                else lv_obj_add_style(btn_metctrl[i], &estilo_default, 0); 

                if (i == metodo_control_sw)  lv_obj_add_style(btn_metctrl[i], &estilo_outline, 0);
                else lv_obj_remove_style(btn_metctrl[i], &estilo_outline, 0); 
            }
        }
        else{
            for (int i = 0; i < 4; i++) {
                lv_obj_add_style(btn_metctrl[i], &estilo_default, 0);

                if (i == metodo_control_sw)  lv_obj_add_style(btn_metctrl[i], &estilo_outline, 0);
                else lv_obj_remove_style(btn_metctrl[i], &estilo_outline, 0); 
            }

            lv_obj_add_style(btn_volver, &estilo_focused, 0);
        }
    
        break;
    

    case MODO_MANUAL:
        if (opcion_actual < 3){
            lv_obj_add_style(btn_volver, &estilo_default, 0);

            for (int i = 0; i < 3; i++) {

                if (i == opcion_actual) lv_obj_add_style(spinboxes[i], &spinbox_estilo_focused, 0);
                else  lv_obj_add_style(spinboxes[i], &spinbox_estilo_default, 0);
                
            }
        }
        else{
            lv_obj_add_style(btn_volver, &estilo_focused, 0);

            for(int i =0; i < 3; i++){
                lv_obj_add_style(spinboxes[i], &spinbox_estilo_default, 0);
            }
    
        }
        break;
    
    case GRAFICAS:{

        if (opcion_actual2 == 0){ 
            lv_obj_add_style(btn_prev, &estilo_focused, 0);
            lv_obj_add_style(btn_next, &estilo_default, 0);
            lv_obj_add_style(btn_volver, &estilo_default, 0);
        }
        else if(opcion_actual2 == 1){
            lv_obj_add_style(btn_volver, &estilo_focused, 0);
            lv_obj_add_style(btn_prev, &estilo_default, 0);
            lv_obj_add_style(btn_next, &estilo_default, 0);
        }
        else{
            lv_obj_add_style(btn_next, &estilo_focused, 0);
            lv_obj_add_style(btn_prev, &estilo_default, 0);
            lv_obj_add_style(btn_volver, &estilo_default, 0);
        }
    
    }

    default:
        break;
    }

}

void actualizar_enc_touch_cb(lv_timer_t * timer) {
    if (label_encoder_x) {
        char buf[32];
        snprintf(buf, sizeof(buf), "Encoder X: %.2f", datos_display.encoderX);
        lv_label_set_text(label_encoder_x, buf);
    }
    if (label_encoder_y) {
        char buf_y[32];
        snprintf(buf_y, sizeof(buf_y), "Encoder Y: %.2f", datos_display.encoderY);
        lv_label_set_text(label_encoder_y, buf_y);
    }

    if (label_encoder_z) {
        char buf_z[32];
        snprintf(buf_z, sizeof(buf_z), "Encoder Z: %.2f", datos_display.encoderZ);
        lv_label_set_text(label_encoder_z, buf_z);
    }

    if (label_touch_x) {
        lv_label_set_text_fmt(label_touch_x, "Touch X: %d", datos_display.touchX);
    }
    if (label_touch_y) {
        lv_label_set_text_fmt(label_touch_y, "Touch Y: %d", datos_display.touchY);
    }
}


void scroll_event_cb(lv_event_t * e)
{   
    lv_obj_t * cont = lv_event_get_target(e);
    lv_area_t cont_a;
    lv_obj_get_coords(cont, &cont_a);
    lv_coord_t cont_y_center = cont_a.y1 + lv_area_get_height(&cont_a) / 2;

    lv_coord_t r = lv_obj_get_height(cont) * 7 / 10;
    uint32_t child_cnt = lv_obj_get_child_cnt(cont);

    for(uint32_t i = 0; i < child_cnt; i++) {
        lv_obj_t * child = lv_obj_get_child(cont, i);
        lv_area_t child_a;
        lv_obj_get_coords(child, &child_a);

        lv_coord_t child_y_center = child_a.y1 + lv_area_get_height(&child_a) / 2;
        lv_coord_t diff_y = LV_ABS(child_y_center - cont_y_center);

        lv_coord_t x;
        if(diff_y >= r) {
            x = r;
        } else {
            uint32_t x_sqr = r * r - diff_y * diff_y;
            lv_sqrt_res_t res;
            lv_sqrt(x_sqr, &res, 0x8000);
            x = r - res.i;
        }

        lv_obj_set_style_translate_x(child, x, 0);

        lv_opa_t opa = lv_map(x, 0, r, LV_OPA_TRANSP, LV_OPA_COVER);
        lv_obj_set_style_opa(child, LV_OPA_COVER - opa, 0);
    }
}


void lv_mi_menu() {
    lv_obj_clean(lv_scr_act());
    LV_IMG_DECLARE(fondoconv); // Asegúrate de tener la imagen como array de bytes en C
    lv_obj_t * fondo = lv_img_create(lv_scr_act()); // o el parent que uses
    lv_img_set_src(fondo, &fondoconv);
    lv_obj_align(fondo, LV_ALIGN_CENTER, 0, 0);
    lv_obj_move_background(fondo); // Esto es clave: lo manda al fondo


    opcion_actual = 0;

    menu_list = lv_obj_create(lv_scr_act());
    lv_obj_set_size(menu_list, 200, 200);  // Tamaño como el ejemplo
    lv_obj_center(menu_list);
    lv_obj_set_flex_flow(menu_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_event_cb(menu_list, scroll_event_cb, LV_EVENT_SCROLL, NULL);
    lv_obj_set_style_radius(menu_list, 20, 0);  // Eliminar borde circular
    lv_obj_set_style_clip_corner(menu_list, true, 0);  // No recortar los bordes
    lv_obj_set_scroll_dir(menu_list, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(menu_list, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(menu_list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_row(menu_list, 10, 0);

        // Fondo transparente
    lv_obj_set_style_bg_opa(menu_list, LV_OPA_TRANSP, 0);

    // Borde transparente (si tiene)
    lv_obj_set_style_border_opa(menu_list, LV_OPA_TRANSP, 0);

    // Si hay sombras (opcional)
    lv_obj_set_style_shadow_opa(menu_list, LV_OPA_TRANSP, 0);

    const char * textos[] = {
        "Metodo de control",
        "Modo manual",
        "Datos Variables",
        "Graficos",
        "Apagar"
    };

    for(int i = 0; i < 5; i++) {
        btn_opciones[i] = lv_btn_create(menu_list);
        lv_obj_set_width(btn_opciones[i], lv_pct(100));
        lv_obj_t * label = lv_label_create(btn_opciones[i]);
        lv_label_set_text(label, textos[i]);
    }

    // Forzar actualización visual al estilo scroll
    lv_event_send(menu_list, LV_EVENT_SCROLL, NULL);
    lv_obj_scroll_to_view(lv_obj_get_child(menu_list, 0), LV_ANIM_OFF);
    actualizar_seleccion();
}




void boton_volver(lv_obj_t *sub_scr){
        // Botón para volver
    btn_volver = lv_btn_create(sub_scr);
    lv_obj_align(btn_volver, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_t *lbl_btn = lv_label_create(btn_volver);
    lv_label_set_text(lbl_btn, "Volver");

    // Acción al pulsar "Volver"
    lv_obj_add_event_cb(btn_volver, [](lv_event_t *e) {
        lv_obj_del(lv_event_get_current_target(e));  // Borra el botón actual
        lv_mi_menu();  // Regresa al menú principal
    }, LV_EVENT_CLICKED, NULL);
}


// Función para mapear un valor de un rango a otro
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void mostrar_submenu_graficas(int numero2, lv_obj_t *sub_scr){
    // 1 = prev, 3 = next

    // Actualizar el índice de forma cíclica
    if(numero2 == 1) {
        submenu_actual = (submenu_actual - 1 + NUM_SUBMENUS) % NUM_SUBMENUS;
    } else if(numero2 == 3) {
        submenu_actual = (submenu_actual + 1) % NUM_SUBMENUS;
    }

    switch(submenu_actual){

        //GRAFICA pos vs t
        case 0:{
            // Fondo
            lv_obj_set_style_bg_color(sub_scr, lv_color_black(), 0);

            // Crear gráfica
            static lv_obj_t *chart;
            chart = lv_chart_create(sub_scr);
            lv_obj_set_size(chart, 280, 160);
            lv_obj_align(chart, LV_ALIGN_CENTER, 0, -20);  // 30 px más arriba
            lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
            lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);
            lv_chart_set_point_count(chart, 50);
            lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);


            lv_obj_t * chart_title = lv_label_create(sub_scr);
            lv_label_set_text(chart_title, "Pos pelota (x,y) vs tiempo");
            lv_obj_set_style_text_color(chart_title, lv_color_white(), 0);
            lv_obj_set_style_text_font(chart_title, &lv_font_montserrat_16, 0);
            lv_obj_set_style_text_color(chart_title, lv_color_black(), 0);
            lv_obj_align(chart_title, LV_ALIGN_CENTER, 0, -90);  // Posición encima del chart

            // Crear series para touchX y touchY
            static lv_chart_series_t * serX;
            static lv_chart_series_t * serY;
            serX = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
            serY = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);

            // Timer para actualizar las gráficas

            chart_timer = lv_timer_create(
                [](lv_timer_t * timer) {
                    lv_chart_set_next_value(chart, serX, datos_display.touchX);
                    lv_chart_set_next_value(chart, serY, datos_display.touchY);
                }, 
                100, NULL);

                break;

        }

        //BOLITA
        case 1: {
            const int cont_size = 200;
            const int r = 10;

            // Crear contenedor cuadrado
            lv_obj_t *cuadro = lv_obj_create(sub_scr);
            lv_obj_set_size(cuadro, cont_size, cont_size);
            lv_obj_align(cuadro, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_style_bg_color(cuadro, lv_color_make(50, 50, 50), 0);
            lv_obj_set_style_radius(cuadro, 0, 0);
            lv_obj_clear_flag(cuadro, LV_OBJ_FLAG_SCROLLABLE);

            // Crear círculo dentro del contenedor
            lv_obj_t *circulo = lv_obj_create(cuadro);
            lv_obj_set_size(circulo, r * 2, r * 2);
            lv_obj_set_style_radius(circulo, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(circulo, lv_color_make(255, 0, 0), 0);
            lv_obj_align(circulo, LV_ALIGN_CENTER, 0, 0);

            // Crear timer para mover el círculo
            circulo_timer = lv_timer_create([](lv_timer_t *t) {
                lv_obj_t *circulo = static_cast<lv_obj_t*>(t->user_data);

                // Valores de entrada del touch
                int x = datos_display.touchX;
                int y = datos_display.touchY;
                
                // Rango real de entrada del sensor touch
                const int x_min = 100, x_max = 930;
                const int y_min = 140, y_max = 870;
                
                // Mapear touchX y touchY al espacio del contenedor, considerando el radio
                // para que la pelota no salga del cuadro
                int mapped_x = map(x, x_min, x_max, -76, 100);
                int mapped_y = map(y, y_min, y_max, 100, -76);
                
                // Actualizar posición del círculo respecto al contenedor [DEBUG CONTENEDOR]
                //ESP_LOGI(TAG, "X: %d\n", mapped_x);
                //ESP_LOGI(TAG, "Y: %d\n", mapped_y);

                lv_obj_set_pos(circulo, mapped_x - r, mapped_y - r);
                
            }, 50, circulo); // Actualizar cada 50ms
            
            break;
        }  
        
        // AQUI PONES MÁS GRÁFICAS VE -------------
    }


}


// Función para mostrar un submenú
void mostrar_submenu(int numero, int numero2) {
    lv_obj_t *sub_scr = lv_obj_create(NULL);  // Nueva pantalla
    opcion_actual = 0;

    LV_IMG_DECLARE(fondoconv); // Asegúrate de tener la imagen como array de bytes en C
    lv_obj_t * fondo = lv_img_create(sub_scr); // o el parent que uses
    lv_img_set_src(fondo, &fondoconv);
    lv_obj_align(fondo, LV_ALIGN_CENTER, 0, 0);
    lv_obj_move_background(fondo); // Esto es clave: lo manda al fondo


    switch(numero) {
    
        case 1:{
            const char* etiquetas[] = {
                "Ninguno",
                "Control PID",
                "Control Moderno",
                "Red Neuronal"
            };

            for (int i = 0; i < 4; i++) {
                btn_metctrl[i] = lv_btn_create(sub_scr);
                lv_obj_align(btn_metctrl[i], LV_ALIGN_TOP_MID, 0, 10 + i * 46);
                lv_obj_set_size(btn_metctrl[i], 150, 40);
            

                lv_obj_t* label = lv_label_create(btn_metctrl[i]);
                lv_label_set_text(label, etiquetas[i]);
                lv_obj_center(label);
                }

                boton_volver(sub_scr);
                actualizar_seleccion();
                break;
            }

        //Modo manual
        case 2: {
            // Contenedor principal: acomoda 3 filas verticalmente

            lv_obj_t * main_cont = lv_obj_create(sub_scr);
            lv_obj_set_size(main_cont, 280, 200);
            lv_obj_set_flex_align(main_cont,
                LV_FLEX_ALIGN_CENTER,  // Centra horizontalmente las filas
                LV_FLEX_ALIGN_START,   // Las coloca de arriba hacia abajo
                LV_FLEX_ALIGN_CENTER); // Espacio entre elementos si aplica
            lv_obj_set_flex_flow(main_cont, LV_FLEX_FLOW_COLUMN);  // Vertical (una fila por spinbox)
            lv_obj_set_style_pad_column(main_cont, 6, 0);
            lv_obj_clear_flag(main_cont, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_pad_top(main_cont, 20, 0);  // 20 píxeles de padding superior

            lv_obj_set_style_bg_opa(main_cont, LV_OPA_TRANSP, 0);   // Fondo transparente
            lv_obj_set_style_border_width(main_cont, 0, 0);         // Sin borde



            // lv_obj_t * titulo = lv_label_create(main_cont); 
            // lv_label_set_text(titulo, "Modo Manual");
            // lv_obj_set_style_text_font(titulo, &lv_font_montserrat_20, 0);  // Puedes cambiar el tamaño
            // lv_obj_set_style_text_align(titulo, LV_TEXT_ALIGN_CENTER, 0);
            // lv_obj_set_width(titulo, LV_PCT(100));  // O usa mismo ancho que el contenedor
            // lv_obj_set_style_text_color(titulo, lv_color_white(), 0);
            // lv_obj_set_style_pad_top(titulo, 10, 0);  // Espacio debajo del título

            const char * titulos[3] = {"X", "Y", "Z"};
            lv_color_t colores[3] = {lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_GREEN), lv_color_make(0, 0, 139)};  // Rojo, verde, azul oscuro


            for (int i = 0; i < 3; i++) {

                // Fila: contenedor horizontal con [-] [spinbox] [+]
                lv_obj_t * row = lv_obj_create(main_cont);
                lv_obj_set_size(row, 230, 50);
                lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);  // Horizontal
                lv_obj_set_flex_align(row,
                    LV_FLEX_ALIGN_CENTER,  // ← Ahora estarán centrados horizontalmente también
                    LV_FLEX_ALIGN_CENTER,
                    LV_FLEX_ALIGN_CENTER);
                lv_obj_set_style_pad_all(row, 4, 0);
                lv_obj_set_style_pad_row(row, 4, 0);
                lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_style(row, &estilo_default, 0);



                // Etiqueta del motor
                lv_obj_t * botonmotor = lv_btn_create(row);
                lv_obj_set_size(botonmotor, 30, 30);
                lv_obj_set_style_bg_color(botonmotor, colores[i], LV_PART_MAIN);
                lv_obj_align(botonmotor, LV_ALIGN_BOTTOM_MID, 0, 0);
                lv_obj_t *lbl_btnmtr = lv_label_create(botonmotor);
                lv_label_set_text(lbl_btnmtr, titulos[i]);
                lv_obj_center(lbl_btnmtr);
                

                // Botón -
                btn_minus[i] = lv_btn_create(row);
                lv_obj_set_size(btn_minus[i], 30, 30); 
                lv_obj_set_style_bg_img_src(btn_minus[i], LV_SYMBOL_MINUS, 0);

                // Spinbox
                lv_obj_t * sb = lv_spinbox_create(row);
                lv_obj_set_size(sb, 70, 40);
                spinboxes[i] = sb;
                lv_spinbox_set_range(sb, 0, 90);
                lv_spinbox_set_digit_format(sb, 2, 0);
                lv_spinbox_set_step(sb, 5);

                lv_obj_add_style(sb, &estilo_default, 0);

                 
                lv_obj_remove_style(sb, NULL, LV_PART_CURSOR);
                lv_obj_set_style_text_align(sb, LV_TEXT_ALIGN_CENTER, 0);
                
        
                // Botón +
                btn_plus[i] = lv_btn_create(row);
                lv_obj_set_size(btn_plus[i], 30, 30);
                lv_obj_set_style_bg_img_src(btn_plus[i], LV_SYMBOL_PLUS, 0);

            
            }

            boton_volver(sub_scr);
            actualizar_seleccion();
            break;
        }

        //Datos variables
        case 3: {
            datosParaEnviar.var_control = 1;
            lv_obj_t* cont = lv_obj_create(sub_scr);
            lv_obj_set_size(cont, 200, 180);
            lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 10);  // Centrado horizontal, 20 px desde arriba
            lv_obj_set_layout(cont, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_row(cont, 6, 0);  // espacio entre filas


            lv_obj_set_style_radius(cont, 10, 0);
          
            //lv_obj_set_style_border_width(cont, 2, 0);
            //lv_obj_set_style_border_color(cont, lv_color_hex(0x444444), 0);
            //lv_obj_set_style_shadow_width(cont, 10, 0);
            //lv_obj_set_style_shadow_color(cont, lv_color_hex(0xaaaaaa), 0);


            lv_obj_set_style_text_font(cont, &lv_font_montserrat_16, 0);

            lv_obj_add_style(cont, &estilo_contenedor_oscuro, 0);

            

            lv_obj_t * titulo = lv_label_create(cont); 
            lv_label_set_text(titulo, "Sensores");
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_20, 0);  // Puedes cambiar el tamaño
            lv_obj_set_style_text_align(titulo, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_width(titulo, 160);


            label_encoder_x = lv_label_create(cont);
            lv_label_set_text_fmt(label_encoder_x, "Encoder X: %.2f", datos_display.encoderX);

            label_encoder_y = lv_label_create(cont);
            lv_label_set_text_fmt(label_encoder_y, "Encoder Y: %.2f", datos_display.encoderY);

            label_encoder_z = lv_label_create(cont);
            lv_label_set_text_fmt(label_encoder_z, "Encoder Z: %.2f", datos_display.encoderZ);
            lv_obj_set_style_pad_bottom(label_encoder_z, 6, 0);  // espacio entre filas

            label_touch_x = lv_label_create(cont);
            lv_label_set_text_fmt(label_touch_x, "Touch X: %d", datos_display.touchX);

            label_touch_y = lv_label_create(cont);
            lv_label_set_text_fmt(label_touch_y, "Touch Y: %d", datos_display.touchY);

            timer_enc_touch = lv_timer_create(actualizar_enc_touch_cb, 50, NULL);

            boton_volver(sub_scr);
            lv_obj_add_style(btn_volver, &estilo_focused, 0);
            break;
        }
        
        //graficas
        case 4: {
            
            mostrar_submenu_graficas(numero2, sub_scr);


            // Botón volver
            // Contenedor de botones en fila
            lv_obj_t * cont_botones = lv_obj_create(sub_scr);
            lv_obj_set_size(cont_botones, LV_PCT(100), 60);
            lv_obj_align(cont_botones, LV_ALIGN_BOTTOM_MID, 0, -4);
            lv_obj_set_flex_flow(cont_botones, LV_FLEX_FLOW_ROW);
            lv_obj_set_style_bg_opa(cont_botones, LV_OPA_TRANSP, 0);   // Fondo transparente
            lv_obj_set_style_border_width(cont_botones, 0, 0);             // sin bordes

            lv_obj_set_flex_align(cont_botones, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);




            // Botón '<'
            btn_prev = lv_btn_create(cont_botones);
            lv_obj_set_size(btn_prev, 30, 30);
            lv_obj_t * label_prev = lv_label_create(btn_prev);
            lv_label_set_text(label_prev, LV_SYMBOL_LEFT);
            lv_obj_center(label_prev);

            boton_volver(cont_botones);

            // Botón '>'
            btn_next = lv_btn_create(cont_botones);
            lv_obj_set_size(btn_next, 30, 30);
            lv_obj_t * label_next = lv_label_create(btn_next);
            lv_label_set_text(label_next, LV_SYMBOL_RIGHT);
            lv_obj_center(label_next);

            actualizar_seleccion();


            break;
        }
        
        //apagado
        case 5:{

            // Fondo negro
            lv_obj_set_style_bg_color(sub_scr, lv_color_black(), 0); //Pendiente decidir el fondo que se va a usar

            // Contenedor centrado (opcional para agrupar el texto)
            lv_obj_t * cont = lv_obj_create(sub_scr);
            lv_obj_set_size(cont, 240, 120);
            lv_obj_center(cont);
            lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0); // contenedor transparente
            lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

            // Título grande
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, "BALL & PLATE");
            lv_obj_set_style_text_color(titulo, lv_color_white(), 0);
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_28, 0);
            lv_obj_set_style_text_align(titulo, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_width(titulo, LV_PCT(100));

            // Subtítulo pequeño
            lv_obj_t * subtitulo = lv_label_create(cont);
            lv_label_set_text(subtitulo, "By Diego C. & Brian R.");
            lv_obj_set_style_text_color(subtitulo, lv_color_white(), 0);
            lv_obj_set_style_text_font(subtitulo, &lv_font_montserrat_16, 0);
            lv_obj_set_style_text_align(subtitulo, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_width(subtitulo, LV_PCT(100));
            lv_obj_set_style_pad_top(subtitulo, 10, 0);

            // Organiza los elementos verticalmente
            lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(cont,
                LV_FLEX_ALIGN_CENTER,  // centra horizontalmente
                LV_FLEX_ALIGN_CENTER,  // centra verticalmente
                LV_FLEX_ALIGN_CENTER); // espacio entre ellos

            //MOTOR A pos init
            reset_motor_pos();
            // Espera para que los motores regresen suavemente antes de apagarse
            vTaskDelay(pdMS_TO_TICKS(800));  // Espera 800 ms (ajustable según velocidad de los motores)
            datosParaEnviar.apagado = 1;
            metodo_control_sw = 0;

            break;

        }

    }
    

    lv_scr_load(sub_scr);  // Cambia la pantalla
}



void manejar_menu_principal(const ui_msg_t& msg) {
    switch (msg.cmd) {
        case UI_CMD_NAVIGATE_UP:
            opcion_actual = (opcion_actual + 4) % 5;
            //ESP_LOGI(TAG, "NAVIGATE_UP -> opcion_actual: %d\n", opcion_actual);
            actualizar_seleccion();
            lv_obj_scroll_to_view(lv_obj_get_child(menu_list, opcion_actual), LV_ANIM_ON);
            lv_event_send(menu_list, LV_EVENT_SCROLL, NULL);
            break;
        case UI_CMD_NAVIGATE_DOWN:
            opcion_actual = (opcion_actual + 1) % 5;
            //ESP_LOGI(TAG, "NAVIGATE_DOWN -> opcion_actual: %d\n", opcion_actual);
            actualizar_seleccion();
            lv_obj_scroll_to_view(lv_obj_get_child(menu_list, opcion_actual), LV_ANIM_ON);
            lv_event_send(menu_list, LV_EVENT_SCROLL, NULL);
            break;
        case UI_CMD_SELECT:
            pantalla_actual = (Pantalla)(opcion_actual + 1);
            printf("Seleccionaste opción %d\n", opcion_actual + 1);
            mostrar_submenu(opcion_actual + 1, 1);
            break;
        default:
            break;
    }
}


void manejar_metodo_control(const ui_msg_t& msg) {
    switch (msg.cmd) {
        case UI_CMD_NAVIGATE_UP:
            opcion_actual = (opcion_actual + 4) % 5;
            //ESP_LOGI(TAG, "NAVIGATE_UP -> opcion_actual: %d\n", opcion_actual);
            actualizar_seleccion();
            break;
        case UI_CMD_NAVIGATE_DOWN:
            opcion_actual = (opcion_actual + 1) % 5;
            //ESP_LOGI(TAG, "NAVIGATE_DOWN -> opcion_actual: %d\n", opcion_actual);
            actualizar_seleccion();
            break;
        case UI_CMD_SELECT:
            if(opcion_actual < 4){
                metodo_control_sw = opcion_actual;
                ESP_LOGI(TAG, "Método de control seleccionado: %d\n", metodo_control_sw);
                if(opcion_actual > 0){
                    datosParaEnviar.anguloX = 40;
                    datosParaEnviar.anguloY = 40;
                    datosParaEnviar.anguloZ = 40;
                }
            }
            else{
                pantalla_actual = MENU_PRINCIPAL;
                lv_mi_menu();
            }
            actualizar_seleccion();
            break;
        default:
            break;
    }
}


void manejar_modo_manual(const ui_msg_t& msg){
    switch (msg.cmd) {
        case UI_CMD_NAVIGATE_UP:
            opcion_actual = (opcion_actual + 3) % 4;  // Wrap-around 0→3→2→1...
            actualizar_seleccion();
            break;

        case UI_CMD_NAVIGATE_DOWN:
            opcion_actual = (opcion_actual + 1) % 4;  // Wrap-around 0→1→2→3→0
            actualizar_seleccion();
            break;

        case UI_CMD_NAVIGATE_LEFT:
            if (opcion_actual < 3){
                lv_spinbox_decrement(spinboxes[opcion_actual]);
                lv_obj_set_style_bg_color(btn_minus[opcion_actual], lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
            }
            break;

        case UI_CMD_NAVIGATE_RIGHT:
            if (opcion_actual < 3){
                lv_spinbox_increment(spinboxes[opcion_actual]);
                lv_obj_set_style_bg_color(btn_plus[opcion_actual], lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);

            }
            break;

        case UI_CMD_SELECT:
            if (opcion_actual == 3) {
                pantalla_actual = MENU_PRINCIPAL;
                lv_mi_menu();
            }
            else{
                lv_spinbox_set_value(spinboxes[opcion_actual], 0);
            }
            break;

        case UI_CMD_JOYSTICK_CENTER:
            for(int i =0; i < 3; i++){
            lv_obj_set_style_bg_color(btn_minus[i], lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
            lv_obj_set_style_bg_color(btn_plus[i], lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
            }

        default:
        break;

    }
}

void manejar_datos_variables(const ui_msg_t& msg){
    switch (msg.cmd) {
        case UI_CMD_NAVIGATE_RIGHT:
            datosParaEnviar.anguloX += 5; //quitar
            break;

        case UI_CMD_NAVIGATE_LEFT:
            datosParaEnviar.anguloX -= 5; //quitar
            break;

        case UI_CMD_SELECT:
            if (timer_enc_touch != NULL) {
                lv_timer_del(timer_enc_touch);
                timer_enc_touch = NULL;
                }
            pantalla_actual = MENU_PRINCIPAL;
            datosParaEnviar.var_control = 0; //quitar
            datosParaEnviar.anguloX = 0; //quitar
            datosParaEnviar.anguloY = 0; //quitar
            datosParaEnviar.anguloZ = 0; //quitar
            lv_mi_menu();
            break;
        default:
            break;
        }
}

void manejar_graficas(const ui_msg_t& msg){

    switch (msg.cmd) {
        case UI_CMD_NAVIGATE_LEFT:
            opcion_actual2 = (opcion_actual2 + 2) % 3;
            //ESP_LOGI(TAG, "NAVIGATE_LEFT -> opcion_actual2: %d\n", opcion_actual2);
            actualizar_seleccion();

            break;
        case UI_CMD_NAVIGATE_RIGHT: 
    
            opcion_actual2 = (opcion_actual2 + 1) % 3;
            
            //ESP_LOGI(TAG, "NAVIGATE_RIGHT -> opcion_actual2: %d\n", opcion_actual2);
            actualizar_seleccion();
            break;
        case UI_CMD_SELECT:
            //ESP_LOGI(TAG, "SELECT -> opcion_actual2: %d\n", opcion_actual2 + 1);
            if (chart_timer != nullptr) {
                lv_timer_del(chart_timer);
                chart_timer = nullptr;
            }

            if (circulo_timer != nullptr) {
                lv_timer_del(circulo_timer);
                circulo_timer = nullptr;
            }

            if (opcion_actual2 == 1){
                pantalla_actual = MENU_PRINCIPAL;
                opcion_actual2 = 0;
                lv_mi_menu();
                
            }
            else mostrar_submenu(4, opcion_actual2 + 1);

            
            break;
        default:
            break;
    }
}


void manejar_apagado(const ui_msg_t& msg){
    switch (msg.cmd) {
        case UI_CMD_SELECT:
            pantalla_actual = MENU_PRINCIPAL;
            datosParaEnviar.apagado = 0;
            lv_mi_menu();
            break;
        default:
            //Poner un texto desvanecido que diga presiona el joystick para continuar
            break;
    }
}


void procesar_ui_msg(const ui_msg_t& msg) {

    switch (pantalla_actual) {
        case MENU_PRINCIPAL:
            manejar_menu_principal(msg);
            break;
        case METODO_CONTROL:
            manejar_metodo_control(msg);
            break;
        case MODO_MANUAL:
            manejar_modo_manual(msg);
            break;
        case DATOS_VARIABLES:
            manejar_datos_variables(msg);
            break;
        case GRAFICAS:
            manejar_graficas(msg);
            break;
        case APAGADO:
            manejar_apagado(msg);
            break;  
        default:
            break;
    }
}



// --------------------- LVGL Tick Task ---------------------

void lv_tick_task(void *arg) {
    while (1) {
        lv_tick_inc(1);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void lvgl_task(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(10);
    ui_msg_t msg;

    while (true) {
        // Solo manejar LVGL en la sección crítica
        if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_timer_handler();  // Renderiza y maneja eventos LVGL
            xSemaphoreGive(xGuiSemaphore);
        }

        // Procesar mensajes FUERA del semáforo (esto evita conflictos)
        while (xQueueReceive(ui_msg_queue, &msg, 0)) {
            if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                //ESP_LOGI(TAG, "Mensaje recibido");
                procesar_ui_msg(msg);  // Esta función sí accede a la UI
                xSemaphoreGive(xGuiSemaphore);
            }
        }

        vTaskDelay(xDelay);
    }
}


// ------------------------- CONTROL ---------------------------


float map_output(float output, float in_min, float in_max, float out_min, float out_max) {
    return (output - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float PID(float setpoint, float measured, float dt, float Kpin, float Kdin, float Kiin) {
    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    float output = Kpin * error + Kiin * integral + Kdin * derivative;
    previous_error = error;
    return output;
}




void pid_task(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(50);
    float setpoint = 500; // valor deseado del sensor touch (ajustar)


        // Definir los límites de salida del PID
    float pid_output_min = -330;  // Mínimo valor esperado del PID
    float pid_output_max =  330;   // Máximo valor esperado del PID
    float output_min = 0;          // Mínimo valor después del mapeo
    float output_max = 90;        // Máximo valor después del mapeo

    while (true) {
        uint16_t measured = datos_display.touchY;
        uint16_t inverted_value = 900 - measured;
        switch(metodo_control_sw){
            case 0:
                if(pantalla_actual == METODO_CONTROL) reset_motor_pos();
                break;
            // PID
            case 1: {
                float output = PID(setpoint, (float)inverted_value, 0.05, Kpa, Kda, Kia); // dt = 50 ms = 0.05 s
                output = map_output(output,pid_output_min,pid_output_max,output_min,output_max);
                datosParaEnviar.anguloY = output;

                ESP_LOGI(TAG, "output %f\n", output);
            
                break;
            }

            default:
                break;
        }

        vTaskDelay(xDelay);
        
    }
}

/*
void pid_task(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(50);
    float setpointX = 400, setpointY = 400;

    while (true) {
        switch(metodo_control_sw){
            case 0:
                if(pantalla_actual == METODO_CONTROL) reset_motor_pos();
                break;
            // PID
            case 1: {
                float x = datos_display.touchX;
                float y = datos_display.touchY;

                float errorX = setpointX - x;
                float errorY = setpointY - y;

                float error_A = errorY;  // Motor A controla eje Y
                float error_B = (-errorX + errorY) / 1.4142; // Motor B: diagonal
                float error_C = (errorX + errorY) / 1.4142; // Motor C: diagonal

                float outA = PID(0, error_A, 0.05, Kpa, Kda, Kia);
                float outB = PID(0, error_B, 0.05, Kpb, Kdb, Kib);
                float outC = PID(0, error_C, 0.05, Kpc, Kdc, Kic);

                datosParaEnviar.anguloY = map_output(outA, -330, 330, 0, 90);
                datosParaEnviar.anguloX = map_output(outB, -330, 330, 0, 90); // ejemplo
                datosParaEnviar.anguloZ = map_output(outC, -330, 330, 0, 90);

                vTaskDelay(xDelay);
        }
    }
    }
}
*/


#if DEBUG_RENDIMIENTO
void rendimiento(void *args){
    while(true){
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    ESP_LOGI(TAG, "Heap libre: %d bytes\n", free_heap);
    vTaskDelay(20);
    }
}
#endif


// --------- Función principal ---------

extern "C" void app_main(void) {

    init_spi();
    init_joystick();  // Inicializar el joystick


    xGuiSemaphore = xSemaphoreCreateMutex();

    ui_msg_queue = xQueueCreate(10, sizeof(ui_msg_t));


    tft.init(); //inicializa la pantalla

    tft.setRotation(1);


    lvgl_setup();

    

    init_estilos();


    if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
        pantalla_actual = APAGADO;
        mostrar_submenu(5, 1); //Menu apagado
        xSemaphoreGive(xGuiSemaphore);

    }


    xTaskCreate(lvgl_task, "LVGL Task", 4096, NULL, 1, NULL);

    xTaskCreate(lv_tick_task, "LVGL Tick", 1024, NULL, 1, NULL);

    xTaskCreate(tarea_joystick, "TareaJoystick", 4096, NULL, 1, NULL);
    
    xTaskCreate(tarea_spi, "TareaSPI", 2048, NULL, 1, NULL);

    xTaskCreate(pid_task, "PID Control", 2048, NULL, 1, NULL);



    #if DEBUG_RENDIMIENTO
    xTaskCreate(rendimiento, "RendimientoESP", 2048, NULL, 1, NULL);
    #endif


    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
