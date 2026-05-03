#include "dwm3000.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

/* Pull in the canonical Qorvo register definitions. All packed addresses,
 * bit masks, and bit offsets come from this header — no more retyping. */
#include "dw3000_deca_regs.h"

static const char *TAG = "dwm3000";
#define SPI_CLOCK_HZ  (2 * 1000 * 1000)

static spi_device_handle_t s_dw = NULL;
static int s_rst_pin = -1;
static bool s_bus_initialized = false;

/* ---- Hard reset / SPI bus management (unchanged) ---------------------- */

void dwm3000_hard_reset(void)
{
    if (s_rst_pin < 0) return;
    gpio_set_direction(s_rst_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(s_rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_direction(s_rst_pin, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(100));
}

esp_err_t dwm3000_init(int mosi, int miso, int sclk, int cs, int rst)
{
    dwm3000_deinit();
    s_rst_pin = rst;

    spi_bus_config_t bus = {
        .mosi_io_num = mosi, .miso_io_num = miso, .sclk_io_num = sclk,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 256,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }
    s_bus_initialized = true;

    spi_device_interface_config_t dev = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = 0, .spics_io_num = cs, .queue_size = 1,
    };
    err = spi_bus_add_device(SPI2_HOST, &dev, &s_dw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        spi_bus_free(SPI2_HOST);
        s_bus_initialized = false;
        return err;
    }

    dwm3000_hard_reset();
    return ESP_OK;
}

esp_err_t dwm3000_deinit(void)
{
    if (s_dw != NULL) { spi_bus_remove_device(s_dw); s_dw = NULL; }
    if (s_bus_initialized) { spi_bus_free(SPI2_HOST); s_bus_initialized = false; }
    s_rst_pin = -1;
    return ESP_OK;
}

esp_err_t dwm3000_read_devid(uint32_t *out_devid)
{
    if (s_dw == NULL) return ESP_ERR_INVALID_STATE;
    uint8_t tx[5] = {0}, rx[5] = {0};
    spi_transaction_t t = { .length = 5 * 8, .tx_buffer = tx, .rx_buffer = rx };
    esp_err_t err = spi_device_polling_transmit(s_dw, &t);
    if (err != ESP_OK) return err;
    *out_devid = ((uint32_t)rx[1])
               | ((uint32_t)rx[2] << 8)
               | ((uint32_t)rx[3] << 16)
               | ((uint32_t)rx[4] << 24);
    return ESP_OK;
}

esp_err_t dwm3000_wait_ready(int timeout_ms)
{
    int elapsed = 0;
    uint32_t devid = 0;
    while (elapsed < timeout_ms) {
        if (dwm3000_read_devid(&devid) == ESP_OK &&
            (devid >> 16) == DW3000_DEV_ID_EXPECTED_HI) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
        elapsed += 5;
    }
    ESP_LOGE(TAG, "Not ready after %d ms (last DEV_ID=0x%08lX)",
             timeout_ms, (unsigned long)devid);
    return ESP_ERR_TIMEOUT;
}

void dwm3000_reset_pin_only(int rst) {
    gpio_set_direction((gpio_num_t)rst, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_direction((gpio_num_t)rst, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(100));
}

/* ===================================================================== */
/* SPI transaction layer                                                  */
/* ===================================================================== */

static void dw_make_header(uint8_t *hdr, uint8_t base_id,
                           uint8_t sub_addr, bool write)
{
    hdr[0] = (uint8_t)((write ? 0x80 : 0x00) | 0x40 | (base_id & 0x3F));
    hdr[1] = (uint8_t)((sub_addr & 0x7F) << 1);
}

esp_err_t dwm3000_reg_read(uint8_t base_id, uint8_t sub_addr,
                           void *dst, size_t len)
{
    if (s_dw == NULL) return ESP_ERR_INVALID_STATE;
    if (len == 0 || len > 8 || dst == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t tx[2 + 8] = {0};
    uint8_t rx[2 + 8] = {0};
    dw_make_header(tx, base_id, sub_addr, false);

    spi_transaction_t t = {
        .length    = (2 + len) * 8,
        .rxlength  = (2 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(s_dw, &t);
    if (err != ESP_OK) return err;
    memcpy(dst, &rx[2], len);
    return ESP_OK;
}

esp_err_t dwm3000_reg_write(uint8_t base_id, uint8_t sub_addr,
                            const void *src, size_t len)
{
    if (s_dw == NULL) return ESP_ERR_INVALID_STATE;
    if (len == 0 || len > 8 || src == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t tx[2 + 8] = {0};
    dw_make_header(tx, base_id, sub_addr, true);
    memcpy(&tx[2], src, len);

    spi_transaction_t t = {
        .length    = (2 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = NULL,
    };
    return spi_device_polling_transmit(s_dw, &t);
}

esp_err_t dwm3000_read32(uint8_t b, uint8_t s, uint32_t *o) {
    if (!o) return ESP_ERR_INVALID_ARG;
    return dwm3000_reg_read(b, s, o, 4);
}
esp_err_t dwm3000_write32(uint8_t b, uint8_t s, uint32_t v) {
    return dwm3000_reg_write(b, s, &v, 4);
}
esp_err_t dwm3000_read8(uint8_t b, uint8_t s, uint8_t *o) {
    if (!o) return ESP_ERR_INVALID_ARG;
    return dwm3000_reg_read(b, s, o, 1);
}
esp_err_t dwm3000_write8(uint8_t b, uint8_t s, uint8_t v) {
    return dwm3000_reg_write(b, s, &v, 1);
}

/* ===================================================================== */
/* Packed-address helpers — addr = (file << 16) | sub_offset.            */
/* Maps directly onto the packed _ID constants in dw3000_deca_regs.h.    */
/* ===================================================================== */

#define DW_FILE(addr)   ((uint8_t)(((addr) >> 16) & 0x3F))
#define DW_SUB(addr)    ((uint8_t)((addr) & 0x7F))

static inline esp_err_t dw_rd32(uint32_t addr, uint32_t *out) {
    return dwm3000_read32(DW_FILE(addr), DW_SUB(addr), out);
}
static inline esp_err_t dw_wr32(uint32_t addr, uint32_t v) {
    return dwm3000_write32(DW_FILE(addr), DW_SUB(addr), v);
}
static inline esp_err_t dw_wr16(uint32_t addr, uint16_t v) {
    return dwm3000_reg_write(DW_FILE(addr), DW_SUB(addr), &v, 2);
}
static inline esp_err_t dw_wr8(uint32_t addr, uint8_t v) {
    return dwm3000_write8(DW_FILE(addr), DW_SUB(addr), v);
}

/* SYS_STATUS: from header. RCINIT bit lives in the high half. */
#ifndef SYS_STATUS_RCINIT_BIT_MASK
#define SYS_STATUS_RCINIT_BIT_MASK  0x00080000u
#endif

esp_err_t dwm3000_soft_reset(void)
{
    esp_err_t err;
    uint32_t v;

    /* 1. Force XTI clock. CLK_CTRL low 2 bits select system clock; 0b01 = XTI. */
    err = dw_rd32(CLK_CTRL_ID, &v);
    if (err != ESP_OK) return err;
    v = (v & 0xFFFFFFFCu) | 0x01u;
    err = dw_wr32(CLK_CTRL_ID, v);
    if (err != ESP_OK) return err;

    /* 2. Clear AINIT2IDLE so chip stays in IDLE_RC. Bit 8 of SEQ_CTRL. */
    err = dw_rd32(SEQ_CTRL_ID, &v);
    if (err != ESP_OK) return err;
    v &= ~(1u << 8);
    err = dw_wr32(SEQ_CTRL_ID, v);
    if (err != ESP_OK) return err;

    /* 3. Pulse SOFT_RST. Top nibble of SOFT_RST register; 0x0 = reset, 0xF = released. */
    uint32_t ctrl0;
    err = dw_rd32(SOFT_RST_ID, &ctrl0);
    if (err != ESP_OK) return err;
    err = dw_wr32(SOFT_RST_ID, ctrl0 & 0x0FFFFFFFu);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(2));

    return dw_wr32(SOFT_RST_ID, ctrl0 | 0xF0000000u);
}

esp_err_t dwm3000_wait_idle_rc(int timeout_ms)
{
    if (s_dw == NULL) return ESP_ERR_INVALID_STATE;
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    uint32_t s = 0;
    do {
        /* SYS_STATUS lives in GEN_CFG (file 0x00); offset varies but the
         * RCINIT bit is in the second 32-bit word at offset 0x44. */
        if (dwm3000_read32(0x00, 0x44, &s) == ESP_OK) {
            if (s & SYS_STATUS_RCINIT_BIT_MASK) {
                ESP_LOGI(TAG, "IDLE_RC reached, SYS_STATUS=0x%08lX",
                         (unsigned long)s);
                return ESP_OK;
            }
        }
        esp_rom_delay_us(50);
    } while (xTaskGetTickCount() < deadline);
    ESP_LOGE(TAG, "IDLE_RC timeout, last SYS_STATUS=0x%08lX",
             (unsigned long)s);
    return ESP_ERR_TIMEOUT;
}

/* ===================================================================== */
/* Phase 3a (corrected): OTP access via OTP_IF file 0x0B                  */
/* ===================================================================== */

/* OTP_CFG control bits (from header):
 *   bit 0 = OTP_MAN_CTR_EN  (manual control enable — must be set first)
 *   bit 1 = OTP_READ        (manual read trigger)
 *   bit 2 = OTP_WRITE       (manual write/program trigger)
 *   bit 3 = OTP_WRITE_MR    (write mode register)
 *   bit 7 = LDO_KICK        (load LDO_TUNE shadow from OTP)
 *   bit 8 = BIAS_KICK       (load BIAS_TUNE shadow from OTP)
 *
 * Manual read sequence (from Qorvo reference):
 *   1. write OTP_ADDR
 *   2. write OTP_CFG = MAN_CTR_EN
 *   3. write OTP_CFG = MAN_CTR_EN | OTP_READ
 *   4. read  OTP_RDATA
 *   5. write OTP_CFG = 0
 */

esp_err_t dwm3000_otp_read(uint16_t otp_addr, uint32_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    if (otp_addr > 0x7FF) return ESP_ERR_INVALID_ARG;

    esp_err_t err;

    /* 1. Address */
    uint16_t a = otp_addr & 0x7FFu;
    err = dwm3000_reg_write(DW_FILE(OTP_ADDR_ID), DW_SUB(OTP_ADDR_ID), &a, 2);
    if (err != ESP_OK) return err;

    /* 2. Enable manual control */
    err = dw_wr16(OTP_CFG_ID, OTP_CFG_OTP_MAN_CTR_EN_BIT_MASK);
    if (err != ESP_OK) return err;

    /* 3. Trigger read (keep MAN_CTR_EN set) */
    err = dw_wr16(OTP_CFG_ID,
                  OTP_CFG_OTP_MAN_CTR_EN_BIT_MASK | OTP_CFG_OTP_READ_BIT_MASK);
    if (err != ESP_OK) return err;

    esp_rom_delay_us(2);

    /* 4. Read data */
    err = dw_rd32(OTP_RDATA_ID, out);
    if (err != ESP_OK) return err;

    /* 5. Release */
    return dw_wr16(OTP_CFG_ID, 0);
}

esp_err_t dwm3000_otp_read_eui64(uint64_t *out_eui)
{
    if (!out_eui) return ESP_ERR_INVALID_ARG;
    uint32_t lo = 0, hi = 0;
    esp_err_t err = dwm3000_otp_read(0x004, &lo);
    if (err != ESP_OK) return err;
    err = dwm3000_otp_read(0x005, &hi);
    if (err != ESP_OK) return err;
    *out_eui = ((uint64_t)hi << 32) | lo;
    if (lo == 0 && hi == 0) return ESP_ERR_NOT_FOUND;
    return ESP_OK;
}

void dwm3000_otp_dump_calibration(void)
{
    static const struct { uint16_t addr; const char *name; } words[] = {
        { 0x000, "WORD_0x000 " },
        { 0x001, "WORD_0x001 " },
        { 0x002, "WORD_0x002 " },
        { 0x003, "WORD_0x003 " },
        { 0x004, "EUI64_LO   " },
        { 0x005, "EUI64_HI   " },
        { 0x006, "PART_ID    " },
        { 0x007, "LOT_ID     " },
        { 0x008, "VBAT_TEMP  " },
        { 0x009, "WORD_0x009 " },
        { 0x00A, "BIAS_TUNE  " },
        { 0x01C, "ANT_DLY_A  " },
        { 0x01D, "ANT_DLY_B  " },
        { 0x01E, "XTAL_TRIM  " },
    };
    ESP_LOGI(TAG, "OTP calibration dump:");
    for (size_t i = 0; i < sizeof(words)/sizeof(words[0]); ++i) {
        uint32_t v = 0;
        if (dwm3000_otp_read(words[i].addr, &v) == ESP_OK) {
            ESP_LOGI(TAG, "  [0x%03X] %s = 0x%08lX",
                     words[i].addr, words[i].name, (unsigned long)v);
        } else {
            ESP_LOGW(TAG, "  [0x%03X] %s = <read failed>",
                     words[i].addr, words[i].name);
        }
    }
}

esp_err_t dwm3000_load_factory_trims(void)
{
    esp_err_t err;
    uint32_t ldo_lo = 0, ldo_hi = 0, bias = 0, xtal = 0;

    (void)dwm3000_otp_read(0x004, &ldo_lo);
    (void)dwm3000_otp_read(0x005, &ldo_hi);
    (void)dwm3000_otp_read(0x00A, &bias);
    (void)dwm3000_otp_read(0x01E, &xtal);

    bool have_ldo  = (ldo_lo != 0) || (ldo_hi != 0);
    bool have_bias = (bias != 0);
    bool have_xtal = ((xtal & XTAL_XTAL_TRIM_BIT_MASK) != 0);

    /* Kick LDO and/or BIAS into analog shadow regs. */
    uint16_t kick = 0;
    if (have_ldo)  kick |= OTP_CFG_LDO_KICK_BIT_MASK;
    if (have_bias) kick |= OTP_CFG_BIAS_KICK_BIT_MASK;

    if (kick) {
        err = dw_wr16(OTP_CFG_ID, kick);
        if (err != ESP_OK) return err;
        esp_rom_delay_us(10);
        err = dw_wr16(OTP_CFG_ID, 0);
        if (err != ESP_OK) return err;
        ESP_LOGI(TAG, "Kicked LDO=%d BIAS=%d from OTP", have_ldo, have_bias);
    } else {
        ESP_LOGW(TAG, "OTP LDO/BIAS empty — using chip defaults");
    }

    /* XTAL_TRIM: low 7 bits of the XTAL register. */
    uint8_t trim;
    if (have_xtal) {
        trim = (uint8_t)(xtal & XTAL_XTAL_TRIM_BIT_MASK);
        ESP_LOGI(TAG, "XTAL_TRIM from OTP: 0x%02X", trim);
    } else {
        trim = 0x2E;
        ESP_LOGW(TAG, "OTP XTAL_TRIM empty — using default 0x%02X", trim);
    }
    return dw_wr8(XTAL_ID, trim);
}