#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/uart.h"
#define SPI_HOST    HSPI_HOST
#define DMA_CHAN    1
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 23
#define PIN_NUM_SCLK 18
#define PIN_NUM_CS 5
static const char *TAG = "SPI_BUS";
spi_device_handle_t spi;

void spi_bus_init() {
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    ret = spi_bus_initialize(SPI_HOST, &buscfg, DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20000000, // 20 MHz
        .mode = 0, // SPI mode
        .spics_io_num = PIN_NUM_CS,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0
    };
    ret = spi_bus_initialize(SPI_HOST, &buscfg, DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }
    ret = spi_bus_add_device(SPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return;
    }
}

void mcp4921_write(uint16_t value, uint8_t buf, uint8_t ga, uint8_t shdn) {
    uint16_t packet = (buf << 15) | (ga << 14) | (shdn << 13) | (value & 0x0FFF);
    spi_transaction_t trans = {
        .length = 16,
        .tx_data = &packet
    };
    spi_device_transmit(spi, &trans);
}   

int app_main() {
    spi_bus_init();
    uint16_t value = 2048;
    uint8_t buf = 1;
    uint8_t ga = 1;
    uint8_t shdn = 1;
    mcp4921_write(value, buf, ga, shdn);
    return 0;
}

float mcp4921_set_threshold(float Vu) {
    float Vdac = Vu / (1 + (22000.0 / 10000.0));
    uint16_t N = (uint16_t)((Vdac / 3.3) * 4096);
    if (N > 4095) {
        N = 4095; 
    }
    mcp4921_write(N, 1, 1, 1);
    float Vumbral_real = (N / 4096.0) * 3.3 * (1 + (22000.0 / 10000.0));
    return Vumbral_real;
}

void uart_report_threshold(uint16_t N, float Vumbral_real) {
    printf("[SIATA] Umbral actualizado: N = %u | Vumbral = %.2f V\n", N, Vumbral_real);
}

void mcp4921_shutdown(uint16_t last_N) {
    uint16_t packet = (0 << 15) | (0 << 14) | (0 << 13) | (last_N & 0x0FFF); // BUF=0, GA=0, SHDN=0
    spi_transaction_t trans = {
        .length = 16,
        .tx_data = &packet
    };
    spi_device_transmit(spi, &trans);
}   

void app_main() {
    const int uart_num = UART_NUM_0; 
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, 1024 * 2, 0, 0, NULL, 0);

    spi_bus_init();
    
    while (1) {
        char buffer[64];
        int len = uart_read_bytes(uart_num, (uint8_t *)buffer, sizeof(buffer) - 1, portMAX_DELAY);
        if (len > 0) {
            buffer[len] = '\0'; // Null-terminate the string
            float Vu_desired = atof(buffer);
            if (Vu_desired < 0 || Vu_desired > (3.3 * (1 + (22000.0 / 10000.0)))) {
                printf("[SIATA] Error: Valor de Vu fuera de rango. Rango válido: 0 - %.2f V\n", 3.3 * (1 + (22000.0 / 10000.0)));
            } else {
                float Vumbral_real = mcp4921_set_threshold(Vu_desired);
                uint16_t N_real = (uint16_t)((Vumbral_real / (1 + (22000.0 / 10000.0))) / 3.3 * 4096);
                uart_report_threshold(N_real, Vumbral_real);
            }
        }
    }
}

