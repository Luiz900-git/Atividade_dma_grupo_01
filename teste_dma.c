#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "pico/cyw43_arch.h"


// CONFIGURAÇÃO


#define I2C_BUS         i2c0
#define PIN_SDA         0
#define PIN_SCL         1

#define ADDR_LUX        0x23 
#define ADDR_COLOR      0x29 

// Comandos e configuração para o TCS34725
#define TCS_CMD_BIT     0x80
#define TCS_REG_ENABLE  0x00
#define TCS_REG_ATIME   0x01
#define TCS_REG_CONTROL 0x0F
#define TCS_REG_CDATAL  0x14

volatile uint8_t buffer_color[8];
volatile uint8_t buffer_lux[2];
int dma_chan;


// Funções para o I2C e o DMA


// Função segura para trocar o alvo do I2C (Luz e Cor)
void i2c_set_target(uint8_t addr) {
    i2c_get_hw(I2C_BUS)->enable = 0; // Desliga I2C
    i2c_get_hw(I2C_BUS)->tar = addr; // Troca Endereço
    i2c_get_hw(I2C_BUS)->enable = 1; // Religa I2C
}

// Configuração Inicial do DMA
int config_dma(i2c_inst_t *i2c) {
    int chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, i2c_get_dreq(i2c, false));
    dma_channel_configure(chan, &c, NULL, &i2c_get_hw(i2c)->data_cmd, 0, false);
    return chan;
}

// Dispara Leitura 
void dma_trigger_read(int chan, volatile uint8_t *dest, int count) {
    // 1. Arma o DMA
    dma_channel_set_write_addr(chan, (void*)dest, false);
    dma_channel_set_trans_count(chan, count, true);

    // 2. Bombeia os comandos de leitura
    // O endereço já foi configurado antes pela i2c_set_target
    for (int i = 0; i < count; i++) {
        bool last = (i == count - 1);
        uint32_t cmd = I2C_IC_DATA_CMD_CMD_BITS; // Bit de Leitura
        if (last) cmd |= I2C_IC_DATA_CMD_STOP_BITS;
        
        // loop para a leitura
        while (!(i2c_get_hw(I2C_BUS)->status & I2C_IC_STATUS_TFNF_BITS)) tight_loop_contents();
        
        i2c_get_hw(I2C_BUS)->data_cmd = cmd;
    }
}

// Espera com Timeout
bool dma_wait_timeout(int chan, int timeout_ms) {
    while (dma_channel_is_busy(chan) && timeout_ms > 0) {
        sleep_ms(1);
        timeout_ms--;
    }
    if (timeout_ms == 0) {
        dma_channel_abort(chan);
        return false; 
    }
    return true;
}

// Função auxiliar de configuração (Write Blocking)
bool setup_sensor(uint8_t addr, uint8_t *data, size_t len) {
    i2c_set_target(addr); // Garante que estamos falando com o cara certo
    int ret = i2c_write_blocking(I2C_BUS, addr, data, len, false);
    return (ret == len);
}

const char* detect_cor(int r, int g, int b) {
    if (r > g && r > b) return "Vermelho";
    if (g > r && g > b) return "Verde";
    if (b > r && b > g) return "Azul";
    return "-";
}


// Código principla

int main() {
    stdio_init_all();
    if (cyw43_arch_init()) return -1;

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    while (!stdio_usb_connected()) sleep_ms(100);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(1000);

    printf("\n=== SISTEMA I2C0 (FIXED TARGET) ===\n");

    i2c_init(I2C_BUS, 100 * 1000); 
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    // --- CONFIGURAÇÃO ---
    printf("1. Configurando Sensores...\n");
    
    // BH1750
    uint8_t cmd_lux = 0x10; 
    if(setup_sensor(ADDR_LUX, &cmd_lux, 1)) printf("OK: Luz\n");
    else printf("ERRO: Luz\n");

    // TCS34725
    uint8_t cmd_pwr[] = {TCS_CMD_BIT | TCS_REG_ENABLE, 0x03}; 
    if(setup_sensor(ADDR_COLOR, cmd_pwr, 2)) printf("OK: Cor (Pwr)\n");
    else printf("ERRO: Cor (Pwr)\n");
    
    // Configurações extras de Cor
    sleep_ms(5);
    uint8_t cmd_time[] = {TCS_CMD_BIT | TCS_REG_ATIME, 0xC0};
    setup_sensor(ADDR_COLOR, cmd_time, 2);
    uint8_t cmd_gain[] = {TCS_CMD_BIT | TCS_REG_CONTROL, 0x01};
    setup_sensor(ADDR_COLOR, cmd_gain, 2);

    printf("2. DMA Init...\n");
    dma_chan = config_dma(I2C_BUS);
    printf("Pronto.\n\n");

    while (true) {
        printf("\033[H\033[J"); // Limpa tela
        printf(">>> MONITORAMENTO <<<\n");

        // --- LER LUZ ---
        // 1. Troca o alvo para o Sensor de Luz
        i2c_set_target(ADDR_LUX);
        
        // 2. Dispara DMA
        dma_trigger_read(dma_chan, buffer_lux, 2);
        
        // 3. Espera
        if (dma_wait_timeout(dma_chan, 100)) {
            uint16_t lux = (buffer_lux[0] << 8) | buffer_lux[1];
            printf("LUZ: %.1f Lux\n", lux / 1.2f);
        } else {
            printf("LUZ: [TIMEOUT]\n");
        }

        // --- LER COR ---
        // 1. Troca o alvo para o Sensor de Cor
        i2c_set_target(ADDR_COLOR);
        
        // 2. Precisamos dizer registrador ler (Write Address)
        uint8_t reg_cmd = TCS_CMD_BIT | TCS_REG_CDATAL;
        
        i2c_write_blocking(I2C_BUS, ADDR_COLOR, &reg_cmd, 1, true); // true = restart

        // 3. Dispara DMA para ler os dados (Read Data)
        dma_trigger_read(dma_chan, buffer_color, 8);
        
        // 4. Espera
        if (dma_wait_timeout(dma_chan, 100)) {
            uint16_t c = buffer_color[0] | (buffer_color[1] << 8);
            uint16_t r = buffer_color[2] | (buffer_color[3] << 8);
            uint16_t g = buffer_color[4] | (buffer_color[5] << 8);
            uint16_t b = buffer_color[6] | (buffer_color[7] << 8);
            printf("COR: R:%d G:%d B:%d (C:%d) -> %s\n", r, g, b, c, detect_cor(r, g, b));
        } else {
            printf("COR: [TIMEOUT]\n");
        }

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(50);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        
        sleep_ms(1000);
    }
}