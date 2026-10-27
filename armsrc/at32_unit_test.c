//
// Created by dxl on 2026/6/4.
//
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "at32f435_437.h"
#include "at32f435_437_misc.h"
#include "at32f435_437_crm.h"
#include "at32f435_437_gpio.h"
#include "at32f435_437_spi.h"
#include "at32f435_437_dma.h"
#include "at32f435_437_i2c.h"
#include "at32f435_437_i2c_app.h"
#include "at32f435_437_usart.h"
#include "at32f435_437_tmr.h"
#include "at32f435_437_exint.h"
#include "at32f435_437_scfg.h"

#include "cdc_class.h"
#include "usb_core.h"
#include "printf.h"
#include "usb_cdc_apis.h"
#include "flashmem.h"
#include "gpio_apis.h"
#include "util.h"
#include "fpga_apis.h"
#include "rssi_apis.h"
#include "fpga_gw_jtag.h"
#include "i2c.h"
#include "commonutil.h"
#include "sys_apis.h"
#include "ticks_apis.h"
#include "proxmark3_arm.h"
#include "appmain.h"

// Enable or Disable unit test.
#define DXL_DEBUG 1
#if DXL_DEBUG

// 检查按钮是否按下
static uint8_t is_btn_pressed(void) {
    if (BUTTON_PRESS()) {
        SpinDelay(2); // 等待一小会儿，简单防抖
        if (BUTTON_PRESS()) {
            return 1;
        }
    }
    return 0;
}

char debug_pbuf[1024] = {0};

void dxl_print_dbg(const char *fmt, ...);

void dxl_print_dbg(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    kvsprintf(fmt, debug_pbuf, 10, ap);
    va_end(ap);
    usb_write((uint8_t *) debug_pbuf, strlen(debug_pbuf)); // 直接串口传回去，这样子不需要开客户端
}

void test_i2c_rgb_simple(void) {
    int idx = 0;
    uint8_t addr = 0x48;
    uint8_t buf_rgb[3] = {0, 0, 200};
    // uint8_t buf_flash_time[] = {50, 50}; // 1s on, 500ms off.

    StartTicks();
    I2C_init(true);
    I2C_WriteByte(idx, 0x02, addr << 1); // 写索引寄存器，设置后续操作的RGB索引
    I2C_WriteByte(1, 0x01, addr << 1); // 写数量寄存器，设置硬件挂1个灯,很重要！！！，不然无法闪灯
    I2C_BufferWrite(buf_rgb, sizeof(buf_rgb), 0x03, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
    // I2C_WriteByte(1, 0x06, addr << 1); // 写闪灯使能寄存器，使能 idx 对应的灯珠的可控闪烁
    // I2C_BufferWrite(buf_flash_time, sizeof(buf_flash_time), 0x07, addr << 1); // 写闪灯使能寄存器，使能 idx 对应的灯珠的可控闪烁
}

void test_usb_id_pin(void) {
    gpio_init_type gpio_init_struct;
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);
    usb_enable(); // 要初始化USB口，printf调试大法好
    SpinDelay(1000);
#define TEST_USB1_ID_PIN_MODE 1    // 测试模式，为 0 是直接测试读取IO，为1测试OTGFS外设中断
#if TEST_USB1_ID_PIN_MODE == 0 // 测试读取IO的模式
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_10; // PA10_USB1_ID
    gpio_init(GPIOA, &gpio_init_struct);
    while (1) {
        dxl_print_dbg("当前设备是%s模式\n", GpioInputStatus(GPIOA, GPIO_PINS_10) ? "从机" : "主机");
        SpinDelay(800);
    }
#endif
#if TEST_USB1_ID_PIN_MODE == 1 // 测试OTGFS外设中断的模式
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE); // 使能USB_OTG1互联口的时钟
    // 初始化和MUX互联口的ID脚
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_10);
    while (1) {
        dxl_print_dbg("当前设备是%s模式\n", OTG1_GLOBAL->gotgctrl_bit.cidsts ? "从机" : "主机");
        SpinDelay(800);
    }
#endif
}

#define I2Cx_ADDRESS (0x58 << 1)
#define I2C_TIMEOUT  0xFFFFFFFF

void i2c_lowlevel_init(i2c_handle_type *hi2c) {
    gpio_init_type gpio_init_structure;

    /* i2c periph clock enable */
    crm_periph_clock_enable(CRM_I2C1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

    /* configure i2c pins:  sda &scl */
    gpio_init_structure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_structure.gpio_mode = GPIO_MODE_MUX;
    gpio_init_structure.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_structure.gpio_pull = GPIO_PULL_NONE;
    gpio_init_structure.gpio_pins = GPIO_PINS_7 | GPIO_PINS_6; // PC7_I2C1_SDA | PC6_I2C1_SCL
    gpio_init(GPIOC, &gpio_init_structure);

    /* gpio configuration */
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE7, GPIO_MUX_4);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE6, GPIO_MUX_4);

    /* config i2c */
    // 0xB170FFFF   // 10K
    // 0xC0E06969   // 50K
    // 0x80504C4E   // 100K
    // 0x30F03C6B   // 200K
    i2c_init(hi2c->i2cx, 0x0F, 0xB170FFFF);

    i2c_own_address1_set(hi2c->i2cx, I2C_ADDRESS_MODE_7BIT, I2Cx_ADDRESS);
}

void test_i2c_rgb(void) {
    i2c_status_type i2c_status;

    usb_enable(); // 要初始化USB口，printf调试大法好
    SpinDelay(1000);

    // 初始化I2C外设
    i2c_handle_type hi2cx;
    hi2cx.i2cx = I2C1;
    i2c_config(&hi2cx);

#define I2C_RGB_TEST_BUF_SIZE (6)
    uint8_t tx_buf[I2C_RGB_TEST_BUF_SIZE];
    uint8_t rx_buf[I2C_RGB_TEST_BUF_SIZE];
    (void) tx_buf;
    (void) rx_buf;

    while (1) {
        while (is_btn_pressed()) {
        } // 等待松开按钮

#if 0 // 需要测试读取吗

        dxl_print_dbg("按下按钮开始测试读取 > \r\n");

        // 等待按钮按下，就去读取一次
        while (!is_btn_pressed()) {
        }

        if ((i2c_status = i2c_master_receive(&hi2cx, I2Cx_ADDRESS, rx_buf, I2C_RGB_TEST_BUF_SIZE, I2C_TIMEOUT)) !=
                I2C_OK) {
            dxl_print_dbg("读取异常：%d\r\n", i2c_status);
            continue; // 异常的话，直接跳过下面的代码，重新尝试执行
        }

        dxl_print_dbg("读取完成：");
        for (int i = 0; i < I2C_RGB_TEST_BUF_SIZE; ++i) dxl_print_dbg("%02x ", rx_buf[i]);
        dxl_print_dbg("\r\n");

#endif

        SpinDelay(500);

#if 1 // 需要测试写入吗

        dxl_print_dbg("按下按钮开始测试写入 > \r\n");

        // 等待按钮按下，就去写入一次
        while (!is_btn_pressed()) {
        }

        // 初始化tx_buf，填充一些奇怪的数据进去
        for (int i = 0; i < I2C_RGB_TEST_BUF_SIZE; ++i) {
            tx_buf[i] = i; // 把序号填进去就行了
        }

        dxl_print_dbg("开始写入... \r\n");

        if ((i2c_status = i2c_master_transmit(&hi2cx, I2Cx_ADDRESS, tx_buf, I2C_RGB_TEST_BUF_SIZE, I2C_TIMEOUT)) !=
                I2C_OK) {
            dxl_print_dbg("写入异常：%d\r\n", i2c_status);
            continue; // 异常的话，直接跳过下面的代码，重新尝试执行
        }

        dxl_print_dbg("写入完成\r\n");

#endif
    }
}

void test_i2c_cc(void) {
    i2c_status_type i2c_status;

    usb_enable(); // 要初始化USB口，printf调试大法好
    SpinDelay(1000);

    // 初始化I2C外设
    i2c_handle_type hi2cx;
    hi2cx.i2cx = I2C1;
    i2c_config(&hi2cx);

    uint8_t rx_buf[7]; // ID 寄存器 7 个字节

    while (1) {
        SpinDelay(1000);

        dxl_print_dbg("开始读取...\r\n");

        // 手册上 0x47 ，实际上发送的时候，这个封装库没有处理位移，因此我们需要自行处理。
        //  需要将实际地址左移一位，也就是低八位的地址
        if ((i2c_status = i2c_master_receive(&hi2cx, 0x47 << 1, rx_buf, sizeof(rx_buf), I2C_TIMEOUT)) != I2C_OK) {
            dxl_print_dbg("CC 控制器 ID 读取异常：%d\r\n", i2c_status);
            continue; // 异常的话，直接跳过下面的代码，重新尝试执行
        }

        dxl_print_dbg("读取完成：");
        for (int i = 0; i < sizeof(rx_buf); ++i) dxl_print_dbg("%02x ", rx_buf[i]);
        dxl_print_dbg("\r\n");
    }
}

// extern uint32_t _stack_start[], _stack_end[];

void test_i2c_rgb_software(void) {
    uint8_t buf[12] = {255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0,}; // 四个 R
    uint8_t buf1[12] = {0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0,}; // 四个 G
    uint8_t buf2[12] = {0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255,}; // 四个 B
    uint8_t buf_off_1rgb[] = {0x00, 0x00, 0x00};
    uint8_t buf24[24] = {
        255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0
    }; // 四个 R + 四个 G
    uint8_t buf24_empty[24] = {0x00};
    (void) buf;
    (void) buf1;
    (void) buf2;
    (void) buf_off_1rgb;
    (void) buf24;
    (void) buf24_empty;

    usb_enable(); // 要初始化USB口，printf调试大法好
    SpinDelay(1000);

    StartTicks();

    while (1) {
        while (is_btn_pressed()) {
        } // 等待松开按钮

        dxl_print_dbg("按下按钮开始测试软件I2C > \r\n");

        I2C_init(true);

        // 等待按钮按下
        while (!is_btn_pressed()) {
        }

        // 测试是否会干扰到 CC 和 RGB 灯
        // I2C_Reset_EnterMainProgram();

        // 尝试读取数据
        // 0x47 << 1
        // 0x58 << 1
        // int16_t res = I2C_BufferRead(rx_buf, sizeof(rx_buf), 0x01, 0x58 << 1);
        // dxl_print_dbg("读取结果： %d\r\n", res);

        uint8_t addr = 0x58;
        (void) addr;

        uint8_t idx = 0;
        (void) idx;
        bool ret = true;
        (void) ret;

#if 1

        uint8_t addrs[] = {0x48, 0x49, 0x68, 0x69,};
        for (int i = 0; i < sizeof(addrs); ++i) {
            ret = I2C_BufferReadRaw(buf24_empty, 1, 0xFF, addrs[i] << 1);
            dxl_print_dbg("固件版本寄存器读取结果： %d, v%d.%d\r\n", ret, buf24_empty[0], buf24_empty[1]);
            if (ret) {
                dxl_print_dbg("轮询地址为 %02x\r\n", addrs[i]);
                addr = addrs[i];
            }
        }

#endif

        ret = I2C_BufferReadRaw(buf24_empty, 1, 0xFF, addr << 1);
        dxl_print_dbg("固件版本寄存器读取结果： %d, v%d.%d\r\n", ret, buf24_empty[0], buf24_empty[1]);

        ret = I2C_BufferWrite(buf24_empty, 2, 0xFF, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
        dxl_print_dbg("固件版本寄存器写入结果： %d\r\n", ret);

#if 0

        uint8_t addrs[4] = {0x48, 0x58, 0x68, 0x78};
        dxl_print_dbg("开始检测RGB-I2C地址接法\r\n");

#if 0

        uint8_t idx_last_ok = 255;
        while (1) {
            ret = I2C_BufferReadRaw(buf24_empty, 1, 0xFF, addrs[idx] << 1);
            if (ret) {
                if (idx_last_ok == idx) {
                    continue;
                }
                idx_last_ok = idx;
                dxl_print_dbg("接法变动，当前是：");
                if (0 == idx) {
                    dxl_print_dbg("VCC接法\r\n");
                }
                if (1 == idx) {
                    dxl_print_dbg("GND接法\r\n");
                }
                if (2 == idx) {
                    dxl_print_dbg("SCL接法\r\n");
                }
                if (3 == idx) {
                    dxl_print_dbg("SDA接法\r\n");
                }
            }
            if (++idx == 4) {
                idx = 0;
            }
            SpinDelay(200);
        }

#endif

        idx = 2;
        while (1) {
            ret = I2C_BufferReadRaw(buf24_empty, 1, 0xFF, addrs[idx] << 1);
            if (!ret) {
                dxl_print_dbg("通信失败\r\n");
            }
        }

#endif

#if 0

        ret = I2C_BufferReadRaw(buf24_empty, 1, 0x01, addr << 1);
        dxl_print_dbg("数量寄存器读取结果： %d, 值 = %d\r\n", ret, buf24_empty[0]);

        ret = I2C_BufferReadRaw(buf24_empty, 1, 0x02, addr << 1);
        dxl_print_dbg("索引寄存器读取结果： %d, 值 = %d\r\n", ret, buf24_empty[0]);

        // ---- 写入相关数据，测试后续的数据读取功能是否正常

        ret = I2C_WriteByte(8, 0x01, addr << 1); // 写数量寄存器，设置硬件挂8个灯
        dxl_print_dbg("数量寄存器写入结果： %d\r\n", ret);

        ret = I2C_WriteByte(0, 0x02, addr << 1); // 写索引寄存器，设置后续操作的RGB索引
        dxl_print_dbg("索引寄存器写入结果： %d\r\n", ret);

        ret = I2C_BufferWrite(buf24, sizeof(buf24), 0x03, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
        dxl_print_dbg("数据寄存器写入结果： %d\r\n", ret);

        // ---- 写入结束

        ret = I2C_BufferReadRaw(buf24_empty, sizeof(buf24_empty), 0x03, addr << 1);
        dxl_print_dbg("数据寄存器读取结果： %d\r\n", ret);
        for (int i = 0; i < sizeof(buf24); ++i) dxl_print_dbg("%02x ", (uint8_t) buf24_empty[i]);
        dxl_print_dbg("\r\n");

        ret = I2C_BufferReadRaw(buf24_empty, 1, 0x04, addr << 1);
        dxl_print_dbg("熄灯寄存器读取结果： %d, 值 = %d\r\n", ret, buf24_empty[0]);

        // ---- 测试写索引锁定寄存器然后再读取

        ret = I2C_WriteByte(1, 0x05, addr << 1); // 写索引锁定寄存器，使能自增
        dxl_print_dbg("索引锁定寄存器（使能锁定）写入结果： %d\r\n", ret);

        ret = I2C_BufferReadRaw(buf24_empty, 1, 0x05, addr << 1);
        dxl_print_dbg("索引锁定寄存器读取结果： %d, 值 = %d\r\n", ret, buf24_empty[0]);

        ret = I2C_WriteByte(0, 0x05, addr << 1); // 写索引锁定寄存器，使能自增
        dxl_print_dbg("索引锁定寄存器（关闭锁定）写入结果： %d\r\n", ret);

        ret = I2C_BufferReadRaw(buf24_empty, 1, 0x05, addr << 1);
        dxl_print_dbg("索引锁定寄存器读取结果： %d, 值 = %d\r\n", ret, buf24_empty[0]);

        // ---- 测试结束

        // ---- 测试写闪灯使能寄存器然后再读取

        ret = I2C_WriteByte(1, 0x06, addr << 1); // 写闪灯使能寄存器，使能自增
        dxl_print_dbg("闪灯使能寄存器（使能闪灯）写入结果： %d\r\n", ret);

        ret = I2C_BufferReadRaw(buf24_empty, 1, 0x06, addr << 1);
        dxl_print_dbg("闪灯使能寄存器读取结果： %d, 值 = %d\r\n", ret, buf24_empty[0]);

        ret = I2C_WriteByte(0, 0x06, addr << 1); // 写闪灯使能寄存器，使能自增
        dxl_print_dbg("闪灯使能寄存器（关闭闪灯）写入结果： %d\r\n", ret);

        ret = I2C_BufferReadRaw(buf24_empty, 1, 0x06, addr << 1);
        dxl_print_dbg("闪灯使能寄存器读取结果： %d, 值 = %d\r\n", ret, buf24_empty[0]);

        // ---- 测试结束

        ret = I2C_BufferReadRaw(buf24_empty, 2, 0x07, addr << 1);
        dxl_print_dbg("闪灯使能寄存器读取结果： %d, 亮时长 = %d, 灭时长 = %d\r\n", ret, buf24_empty[0], buf24_empty[1]);

#endif

#if 0

        ret = I2C_WriteByte(8, 0x01, addr << 1); // 写数量寄存器，设置硬件挂8个灯
        dxl_print_dbg("数量寄存器写入结果： %d\r\n", ret);

        // 熄灭所有的灯，重新开始跑新的一轮流水
        ret = I2C_WriteByte(0x00, 0x04, addr << 1); // 写熄灯寄存器，数据可传可不传，无所谓
        dxl_print_dbg("熄灯寄存器写入结果： %d\r\n", ret);
        SpinDelay(10);

#endif

#if 0

        ret = I2C_WriteByte(0, 0x02, addr << 1); // 写索引寄存器，设置后续操作的RGB索引
        dxl_print_dbg("索引寄存器写入结果： %d\r\n", ret);

        while (1) {
            ret = I2C_BufferWrite(buf24, sizeof(buf24), 0x03, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
            dxl_print_dbg("数据寄存器写入结果： %d\r\n", ret);

            SpinDelay(50);

            // 按一次修改一次第一个灯的颜色
            if (BUTTON_PRESS()) {
                buf24[0] += 5;
            }
        }

#endif

#if 1

        ret = I2C_WriteByte(idx, 0x02, addr << 1); // 写索引寄存器，设置后续操作的RGB索引
        dxl_print_dbg("索引寄存器写入结果： %d\r\n", ret);

        ret = I2C_WriteByte(0, 0x05, addr << 1); // 写索引锁定寄存器，使能自增
        dxl_print_dbg("索引锁定寄存器（关闭锁定）写入结果： %d\r\n", ret);

        ret = I2C_BufferWrite(buf, sizeof(buf), 0x03, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
        dxl_print_dbg("数据寄存器写入结果： %d\r\n", ret);

        SpinDelay(1000);

        ret = I2C_WriteByte(idx, 0x02, addr << 1); // 写索引寄存器，设置后续操作的RGB索引
        dxl_print_dbg("索引寄存器写入结果： %d\r\n", ret);

        ret = I2C_WriteByte(1, 0x06, addr << 1); // 写闪灯使能寄存器，使能 idx 对应的灯珠的可控闪烁
        dxl_print_dbg("闪灯使能寄存器写入结果(开)： %d\r\n", ret);

        uint8_t buf_flash_time[] = {50, 50}; // 1s on, 500ms off.

        ret = I2C_BufferWrite(buf_flash_time, sizeof(buf_flash_time), 0x07, addr << 1); // 写闪灯使能寄存器，使能 idx 对应的灯珠的可控闪烁
        dxl_print_dbg("闪灯时长寄存器写入结果(1)： %d\r\n", ret);

        SpinDelay(1000);
        SpinDelay(1000);
        SpinDelay(1000);
        SpinDelay(1000);

        buf_flash_time[0] = 100;
        buf_flash_time[1] = 50;

        ret = I2C_BufferWrite(buf_flash_time, sizeof(buf_flash_time), 0x07, addr << 1); // 写闪灯使能寄存器，使能 idx 对应的灯珠的可控闪烁
        dxl_print_dbg("闪灯时长寄存器写入结果(2)： %d\r\n", ret);

        SpinDelay(1000);
        SpinDelay(1000);
        SpinDelay(1000);
        SpinDelay(1000);

        buf_flash_time[0] = 50;
        buf_flash_time[1] = 100;

        ret = I2C_BufferWrite(buf_flash_time, sizeof(buf_flash_time), 0x07, addr << 1); // 写闪灯使能寄存器，使能 idx 对应的灯珠的可控闪烁
        dxl_print_dbg("闪灯时长寄存器写入结果(3)： %d\r\n", ret);

        SpinDelay(1000);
        SpinDelay(1000);
        SpinDelay(1000);
        SpinDelay(1000);

        // 以下逻辑是测试5个关闪条件的，1、主动关闪 2、写索引关闪 3、写数据关闪 4、写熄灯关闪 5、读数据关闪

        //ret = I2C_WriteByte(0, 0x06, addr << 1); // 写闪灯使能寄存器，使能 idx 对应的灯珠的可控闪烁
        //dxl_print_dbg("闪灯使能寄存器写入结果(关)： %d\r\n", ret);

        //ret = I2C_WriteByte(idx, 0x02, addr << 1); // 写索引寄存器，设置后续操作的RGB索引
        //dxl_print_dbg("索引寄存器写入结果： %d\r\n", ret);

        //ret = I2C_BufferWrite(buf, sizeof(buf), 0x03, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
        //dxl_print_dbg("数据寄存器写入结果： %d\r\n", ret);

        //ret = I2C_WriteByte(0x00, 0x04, addr << 1); // 写熄灯寄存器，数据可传可不传，无所谓
        //dxl_print_dbg("熄灯寄存器写入结果： %d\r\n", ret);

        ret = I2C_BufferReadRaw(buf24_empty, sizeof(buf24_empty), 0x03, addr << 1);
        dxl_print_dbg("数据寄存器读取结果： %d\r\n", ret);
        for (int i = 0; i < sizeof(buf24); ++i) dxl_print_dbg("%02x ", (uint8_t) buf24_empty[i]);
        dxl_print_dbg("\r\n");

#endif


#if 0

        ret = I2C_WriteByte(idx, 0x02, addr << 1); // 写索引寄存器，设置后续操作的RGB索引
        dxl_print_dbg("索引寄存器写入结果： %d\r\n", ret);

        ret = I2C_WriteByte(0, 0x05, addr << 1); // 写索引锁定寄存器，使能自增
        dxl_print_dbg("索引锁定寄存器写入结果： %d\r\n", ret);

        uint8_t buf_8r[] = {
            255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0,
        }; // 8个 R

        ret = I2C_BufferWrite(buf_8r, sizeof(buf_8r), 0x03, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
        dxl_print_dbg("数据寄存器写入结果： %d\r\n", ret);

#endif


#if 0

        ret = I2C_WriteByte(idx, 0x00, addr << 1); // 写索引寄存器，设置后续操作的RGB索引
        dxl_print_dbg("索引寄存器写入结果： %d\r\n", ret);

        ret = I2C_WriteByte(1, 0x05, addr << 1); // 写索引锁定寄存器，让操作的RGB索引不会自增
        dxl_print_dbg("索引锁定寄存器写入结果： %d\r\n", ret);

        while (1) {
            ret = I2C_BufferWrite(buf_off_1rgb, 3, 0x03, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
            (void) ret;
            //dxl_print_dbg("数据寄存器写入结果1： %d\r\n", ret);

            // GpioOutputInv(GPIOE, GPIO_PINS_8); // 反转调试脚

            SpinDelay(150);

            // GpioOutputInv(GPIOE, GPIO_PINS_8); // 反转调试脚

            // 只传3个字节，RGB888，表示只刷一个灯
            ret = I2C_BufferWrite(buf, 3, 0x03, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
            (void) ret;
            //dxl_print_dbg("数据寄存器写入结果2： %d\r\n", ret);

            // GpioOutputInv(GPIOE, GPIO_PINS_8); // 反转调试脚

            SpinDelay(150);

            // GpioOutputInv(GPIOE, GPIO_PINS_8); // 反转调试脚
        }

#endif


#if 0

        ret = I2C_WriteByte(0, 0x05, addr << 1); // 写索引锁定寄存器，让操作的RGB索引不会自增
        dxl_print_dbg("索引锁定寄存器写入结果： %d\r\n", ret);

        while (1) {
            ret = I2C_WriteByte(idx, 0x02, addr << 1); // 写索引寄存器，设置后续操作的RGB索引
            //dxl_print_dbg("索引寄存器写入结果： %d\r\n", ret);

            // 只传3个字节，RGB888，表示只刷一个灯
            ret = I2C_BufferWrite(buf, 3, 0x03, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
            //dxl_print_dbg("数据寄存器写入结果： %d\r\n", ret);

            SpinDelay(100);

            ret = I2C_WriteByte(idx, 0x02, addr << 1); // 写索引寄存器，设置后续操作的RGB索引
            //dxl_print_dbg("索引寄存器写入结果： %d\r\n", ret);

            uint8_t buf_off[] = {0x00, 0x00, 0x00};
            ret = I2C_BufferWrite(buf_off, 3, 0x03, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
            //dxl_print_dbg("数据寄存器写入结果： %d\r\n", ret);

            SpinDelay(100);

            if (++idx == 8) {
                idx = 0;
            }
        }

#endif

#if 0

        ret = I2C_WriteByte(0, 0x05, addr << 1); // 写索引锁定寄存器，使能自增
        dxl_print_dbg("索引锁定寄存器写入结果： %d\r\n", ret);

        while (1) {
            ret = I2C_WriteByte(0, 0x02, addr << 1); // 写索引寄存器，设置后续操作的RGB索引
            // dxl_print_dbg("索引寄存器写入结果： %d\r\n", ret);

            uint8_t *pbuf = NULL;
            if (idx == 0) {
                pbuf = buf;
                idx = 1;
            } else if (idx == 1) {
                pbuf = buf1;
                idx = 2;
            } else if (idx == 2) {
                pbuf = buf2;
                idx = 0;
            }

            ret = I2C_BufferWrite(pbuf, sizeof(buf), 0x03, addr << 1); // 写数据寄存器，每三个字节就是对应的RGB888值
            // dxl_print_dbg("数据寄存器写入结果： %d\r\n", ret);

            SpinDelay(300);
        }

#endif
    }
}

void test_i2c_ant_software(void) {
    usb_enable(); // 要初始化USB口，printf调试大法好
    SpinDelay(1000);
    StartTicks();

    while (1) {
        while (is_btn_pressed()) {
        } // 等待松开按钮

        dxl_print_dbg("按下按钮开始测试软件I2C（多频复合天线） > \r\n");

        I2C_init(true);

        // 等待按钮按下
        while (!is_btn_pressed()) {
        }

        uint8_t addr = 0x51;
        (void) addr;
        uint8_t buf8_empty[8] = {0x00};
        (void) buf8_empty;
        bool ret = true;
        (void) ret;

        // 读版本号，通信如果没问题应当成功
        ret = I2C_BufferReadRaw(buf8_empty, 1, 0xFF, addr << 1);
        dxl_print_dbg("固件版本寄存器读取结果： %d, v%d.%d\r\n", ret, buf8_empty[0], buf8_empty[1]);

        // 写版本号寄存器，肯定要失败才对的
        ret = I2C_BufferWrite(buf8_empty, 2, 0xFF, addr << 1);
        dxl_print_dbg("固件版本寄存器写入结果： %d\r\n", ret);

        // 设备标志寄存器，标志当前是pm5的多频复合天线，理论上要读取到：0x70 0x6D 0x35 0x5F 0x61 0x6E 0x74 0x78
        ret = I2C_BufferReadRaw(buf8_empty, 8, 0xFE, addr << 1);
        dxl_print_dbg("设备标志寄存器读取结果(%d)：  %02x %02x %02x %02x %02x %02x %02x %02x\r\n", ret,
                      buf8_empty[0], buf8_empty[1], buf8_empty[2], buf8_empty[3],
                      buf8_empty[4], buf8_empty[5], buf8_empty[6], buf8_empty[7]);

        // 写设备标志寄存器，肯定要失败才对的
        ret = I2C_BufferWrite(buf8_empty, 8, 0xFE, addr << 1);
        dxl_print_dbg("设备标志寄存器写入结果： %d\r\n", ret);

        // 读取IO数据寄存器，8个bit控制8个IO
        ret = I2C_BufferReadRaw(buf8_empty, 1, 0x01, addr << 1);
        dxl_print_dbg("IO数据寄存器读取结果： %d\r\n", ret);

        // 等待按钮重新按下
        while (is_btn_pressed()) {
        }
        dxl_print_dbg("按下按钮开始测试IO写入 > \r\n");
        while (!is_btn_pressed()) {
        }
        int io_idx = 0;
        while (1) {
#if 0 // 测试直接写IO寄存器
            buf8_empty[0] = 0; // 先清除所有之前的设置
            buf8_empty[0] |= 1 << io_idx++; // 然后设置当前的idx位置的io为1，然后顺带自增一下idx
            ret = I2C_BufferWrite(buf8_empty, 1, 0x01, addr << 1);
            dxl_print_dbg("IO数据寄存器写入结果： value = %d, ret = %d\r\n", buf8_empty[0], ret);
#else // 测试写映射寄存器
            buf8_empty[0] = 0x00; // 如果五个高位都不为1，则默认应该是125k
            // 125 134 250 375 500 HFLED LFLED Q
            // buf8_empty[0] = 1 << 7; // 配置为125+低q+关灯
            // buf8_empty[0] = 1 << 6; // 配置为134+低q+关灯
            // buf8_empty[0] = 1 << 5; // 配置为250+低q+关灯
            // buf8_empty[0] = 1 << 4; // 配置为375+低q+关灯
            // buf8_empty[0] = 1 << 3; // 配置为500+低q+关灯
            // buf8_empty[0] |= 1 << 2; // HFLED
            // buf8_empty[0] |= 1 << 1; // LFLED
            // buf8_empty[0] |= 0x01; // 高q
            ret = I2C_BufferWrite(buf8_empty, 1, 0x02, addr << 1);
            dxl_print_dbg("IO映射寄存器写入结果： value = %d, ret = %d\r\n", buf8_empty[0], ret);
#endif

            // 读取IO数据寄存器，8个bit控制8个IO
            ret = I2C_BufferReadRaw(buf8_empty, 1, 0x01, addr << 1);
            dxl_print_dbg("IO数据寄存器读取结果： value = %d, ret = %d\r\n", buf8_empty[0], ret);
            // 延迟一下，流水灯测试
            SpinDelay(50);
            // 重置idx
            if (io_idx == sizeof(buf8_empty)) {
                io_idx = 0;
            }
        }
    }
}

void test_init_debug_pin(void) {
    gpio_init_type gpio_init_struct;
    crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = GPIO_PINS_8; // EXP_IO_IO1
    gpio_init(GPIOE, &gpio_init_struct);
}

void test_usb_xlink_spi(void) {
    // 当前设备上运行的测试模式，两台机器，一主一从
    uint16_t spi1_mode = SPI_MODE_SLAVE;
    uint8_t spi1_buffer[20] = {0x01, 0x02, 0x03, 0x00};

    // 初始化身份切换口
    gpio_inter_usb_spi_role_setup();

    // 主从身份不一样，需要做某些特定的参数配置
    if (spi1_mode == SPI_MODE_MASTER) {
        usb_update_serial(110000001); // 主从设备的USB序列号不同，解决上线慢的问题
        Gpio_Inter_USB_SPI_Role_High(); // spi口需要切换为特定的gpio组，否则使typec协议的txrx再次交叉回来
    } else {
        usb_update_serial(110000002);
        Gpio_Inter_USB_SPI_Role_Low();
    }

    usb_enable(); // 要初始化USB口，printf调试大法好
    SpinDelay(1000);

    gpio_init_type gpio_initstructure;
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

    /* spi1 cs pin */
    gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_initstructure.gpio_pull = GPIO_PULL_UP;
    gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    if (spi1_mode == SPI_MODE_MASTER) {
        gpio_initstructure.gpio_mode = GPIO_MODE_OUTPUT;
    } else {
        gpio_initstructure.gpio_mode = GPIO_MODE_MUX;
        gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE4, GPIO_MUX_5);
    }
    gpio_initstructure.gpio_pins = GPIO_PINS_4;
    gpio_init(GPIOA, &gpio_initstructure);

    /* spi1 sck pin */
    gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_initstructure.gpio_pull = GPIO_PULL_DOWN;
    gpio_initstructure.gpio_mode = GPIO_MODE_MUX;
    gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_initstructure.gpio_pins = GPIO_PINS_5;
    gpio_init(GPIOA, &gpio_initstructure);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE5, GPIO_MUX_5);

    /* spi1 miso pin */
    gpio_initstructure.gpio_pull = GPIO_PULL_UP;
    gpio_initstructure.gpio_pins = GPIO_PINS_6;
    gpio_init(GPIOA, &gpio_initstructure);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE6, GPIO_MUX_5);

    /* spi1 mosi pin */
    gpio_initstructure.gpio_pull = GPIO_PULL_UP;
    gpio_initstructure.gpio_pins = GPIO_PINS_7;
    gpio_init(GPIOA, &gpio_initstructure);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_5);

    /* non communication time: master pull up CS pin release slave */
    if (spi1_mode == SPI_MODE_MASTER) {
        gpio_bits_set(GPIOA, GPIO_PINS_4);
    }

    // ------------- spi 初始化

    spi_init_type spi_init_struct;

    /* master spi initialization */
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    spi_default_para_init(&spi_init_struct);

    /* dual line unidirectional full-duplex mode */
    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = spi1_mode;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_1024;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_LSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    if (spi1_mode == SPI_MODE_MASTER) {
        spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    } else {
        spi_init_struct.cs_mode_selection = SPI_CS_HARDWARE_MODE;
    }
    spi_init(SPI1, &spi_init_struct);
    spi_enable(SPI1, TRUE);

    // while (1) {
    //     gpio_bits_reset(GPIOA, GPIO_PINS_4);
    //     SpinDelay(100);
    //     gpio_bits_set(GPIOA, GPIO_PINS_4);
    //     SpinDelay(100);
    // }

    // ------------- 只测试主机发送，从机接收
    uint8_t idx = 0;
    while (1) {
        if (spi1_mode == SPI_MODE_MASTER) {
            dxl_print_dbg("USB互联口，开始发送\r\n");
            // 主机拉低，片选从机
            gpio_bits_reset(GPIOA, GPIO_PINS_4);
            while (idx < sizeof(spi1_buffer)) {
                while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET);
                spi_i2s_data_transmit(SPI1, spi1_buffer[idx]);
                idx++;
            }
            dxl_print_dbg("USB互联口，SPI主机发送完成\r\n");
            idx = 0;
            /* wait master and slave idle when communication end */
            while (spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) != RESET);
            // 主机拉高，释放从机
            gpio_bits_set(GPIOA, GPIO_PINS_4);
            SpinDelay(500);
        } else {
            dxl_print_dbg("USB互联口，开始接收\r\n");
            while (idx < sizeof(spi1_buffer)) {
                while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET);
                spi1_buffer[idx] = spi_i2s_data_receive(SPI1);
                idx++;
            }
            dxl_print_dbg("USB互联口，SPI从机接收完成: ");
            for (idx = 0; idx < sizeof(spi1_buffer); idx++) dxl_print_dbg("%02x ", spi1_buffer[idx]);
            dxl_print_dbg("\r\n");
            idx = 0;
            /* wait master and slave idle when communication end */
            while (spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) != RESET);
        }
    }
}

void test_usb_xlink_1line_uart(void) {
    // 标记当前身份为主机
    const bool masterIam = 0;

    gpio_init_type gpio_init_struct;

    /* enable the usart1 and gpio clock */
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the usart1 tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_7);

    /* configure usart1 param */
    usart_init(USART1, 57600, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(USART1, USART_PARITY_NONE); // 8 n 1
    usart_transmitter_enable(USART1, FALSE); // 直接接收，不发送
    usart_receiver_enable(USART1, TRUE);
    usart_single_line_halfduplex_select(USART1, TRUE);
    usart_enable(USART1, TRUE);

    // 主从设备的USB序列号不同，解决上线慢的问题
    if (masterIam) {
        usb_update_serial(110000001);
    } else {
        usb_update_serial(110000002);
    }

    usb_enable(); // 要初始化USB口，printf调试大法好
    SpinDelay(1000);

    dxl_print_dbg("单线测试模式启动\r\n");

    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x04, 0x03, 0x02, 0x01,};

    while (1) {
        // 只测试主机发送从机接收
        if (masterIam) {
            for (int i = 0; i < sizeof(data); ++i) {
                while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
                usart_data_transmit(USART1, data[i]);
            }
            // 发完了就打印一下，然后等一会儿再继续发
            dxl_print_dbg("USB互联口，单线串口发送完毕，稍后继续发送\r\n");
            SpinDelay(500);
        } else {
            for (int i = 0; i < sizeof(data); ++i) {
                while (usart_flag_get(USART1, USART_RDBF_FLAG) == RESET);
                data[i] = usart_data_receive(USART1);
            }
            dxl_print_dbg("USB互联口，单线串口接收完成: ");
            for (int i = 0; i < sizeof(data); ++i) dxl_print_dbg("%02x ", data[i]);
            dxl_print_dbg("\r\n");
        }
    }
}

void test_isp_exit(void) {
    usb_enable();
    SpinDelay(1000); // 等一会儿，USB上线以后，PC端重连完成了再继续后面的步骤，避免错过打印的消息。

    dxl_print_dbg("Send 'exit' cmd for isp mode exit >\r\n");

    // 等待发送退出指令
    uint8_t buf[100] = {0x00};
    uint32_t length = 0;
    while (1) {
        length += usb_read(buf + length, 4);
        if (length >= 4 && memcmp(buf, "exit", 4) == 0) {
            length = 0;
            break;
        }
    }

    dxl_print_dbg("Received 'exit' cmd, start exit isp...\r\n");

    // 模拟按钮按下退出isp
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    crm_periph_clock_enable(AT32_GPIO_BTN_CLK, TRUE);
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = AT32_GPIO_BTN_PIN;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    AT32_GPIO_BTN->scr = AT32_GPIO_BTN_PIN; // 按钮按下超过强制退出isp的指定时长，则会强制退出isp模式
    gpio_init(AT32_GPIO_BTN, &gpio_init_struct);
    for (int i = 0; i < 11; i++) {
        // 新版本是3s，老版本是10s，我们先测老版本的
        SpinDelay(1000);
    }
    AT32_GPIO_BTN->clr = AT32_GPIO_BTN_PIN;
    SpinDelay(1000);

    dxl_print_dbg("ISP mode exited.\r\n");
}

void test_beep(void) {
    gpio_init_type gpio_init_struct;

    // PB13 使能，PC9 调制，使用 TMR8_CH4 输出调制
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR8_PERIPH_CLOCK, TRUE);

#define BEEPER_EN_GPIO       GPIOB
#define BEEPER_EN_GPIO_PIN   GPIO_PINS_13
#define BEEPER_MOD_GPIO      GPIOC
#define BEEPER_MOD_GPIO_PIN  GPIO_PINS_9
#define BEEPER_MOD_GPIO_SRC  GPIO_PINS_SOURCE9
#define BEEPER_MOD_GPIO_MUX  GPIO_MUX_3
#define BEEPER_MOD_TMR       TMR8
#define BEEPER_MOD_TMR_CH    TMR_SELECT_CHANNEL_4

    gpio_default_para_init(&gpio_init_struct);
    // 蜂鸣器使能脚
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = BEEPER_EN_GPIO_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(BEEPER_EN_GPIO, &gpio_init_struct);
    gpio_bits_write(BEEPER_EN_GPIO, BEEPER_EN_GPIO_PIN, FALSE);
    // 蜂鸣器调制脚
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = BEEPER_MOD_GPIO_PIN;
    gpio_init(BEEPER_MOD_GPIO, &gpio_init_struct);
    gpio_pin_mux_config(BEEPER_MOD_GPIO, BEEPER_MOD_GPIO_SRC, BEEPER_MOD_GPIO_MUX);

    tmr_internal_clock_set(BEEPER_MOD_TMR);
    tmr_reset(BEEPER_MOD_TMR);
    tmr_base_init(BEEPER_MOD_TMR, 999, 95); // 192M出2k
    tmr_output_config_type tmr_output_struct;
    tmr_output_default_para_init(&tmr_output_struct);
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_channel_config(BEEPER_MOD_TMR, BEEPER_MOD_TMR_CH, &tmr_output_struct);
    tmr_channel_value_set(BEEPER_MOD_TMR, BEEPER_MOD_TMR_CH, 500); // 比较值=500 (50%占空比)
    tmr_counter_enable(BEEPER_MOD_TMR, TRUE);
    tmr_output_enable(BEEPER_MOD_TMR, TRUE);

    while (1) {
        BEEPER_MOD_TMR->pr = 999;
        gpio_bits_write(BEEPER_EN_GPIO, BEEPER_EN_GPIO_PIN, TRUE);
        SpinDelay(20);
        gpio_bits_write(BEEPER_EN_GPIO, BEEPER_EN_GPIO_PIN, FALSE);
        SpinDelay(400);
        BEEPER_MOD_TMR->pr = 1100;
        tmr_channel_value_set(BEEPER_MOD_TMR, BEEPER_MOD_TMR_CH, 550);
        gpio_bits_write(BEEPER_EN_GPIO, BEEPER_EN_GPIO_PIN, TRUE);
        SpinDelay(20);
        gpio_bits_write(BEEPER_EN_GPIO, BEEPER_EN_GPIO_PIN, FALSE);
        SpinDelay(400);
        BEEPER_MOD_TMR->pr = 1200;
        tmr_channel_value_set(BEEPER_MOD_TMR, BEEPER_MOD_TMR_CH, 600);
        gpio_bits_write(BEEPER_EN_GPIO, BEEPER_EN_GPIO_PIN, TRUE);
        SpinDelay(20);
        gpio_bits_write(BEEPER_EN_GPIO, BEEPER_EN_GPIO_PIN, FALSE);
        SpinDelay(400);
        BEEPER_MOD_TMR->pr = 1300;
        tmr_channel_value_set(BEEPER_MOD_TMR, BEEPER_MOD_TMR_CH, 650);
        gpio_bits_write(BEEPER_EN_GPIO, BEEPER_EN_GPIO_PIN, TRUE);
        SpinDelay(20);
        gpio_bits_write(BEEPER_EN_GPIO, BEEPER_EN_GPIO_PIN, FALSE);
        SpinDelay(400);

        // 响完一轮之后直接软件reset
        // ResetChip();
    }
}

void test_power_off_by_btn(void) {
    usb_enable();
    SpinDelay(1000);
    // dxl_print_dbg("SystemStart\n");
    while (1) {
        // 第一阶段，等按下
        if (is_btn_pressed()) {
            // 第二阶段，等抬起
            while (is_btn_pressed()) {}
            // 第三阶段，等一会儿
            SpinDelay(50);
            // 拉低直接关机
            // dxl_print_dbg("SystemOff\n");
            Gpio_ARM_Power_ON_Low();
            // 拉低关机的话，还会有一段PWR电容放电时间，此时我们应当让系统进入死循环，不再处理任何事情
            while (1) {
                // dxl_print_dbg("Waiting Power Off\n");
            }
        }
    }
}

void test_bat_coulometer(void) {
    usb_enable(); // 要初始化USB口，printf调试大法好
    SpinDelay(1000);
    StartTicks();

    while (1) {
        while (is_btn_pressed()) {
        } // 等待松开按钮

        dxl_print_dbg("按下按钮开始测试库仑计 > \r\n");

        I2C_init(true);

        // 等待按钮按下
        while (!is_btn_pressed()) {
        }

        // 用 I2C_BufferReadRaw 读取
        // 用 I2C_BufferWrite 写入

        uint8_t addr = 0xAA; // 最高位为 1010101（地址位） [0|1]（读写位），所以我们给出的整字节地址就是 0xAA 即可
        uint8_t data[20];

        while (1) {
            // Voltage(): 0x04 and 0x05
            bool ret = I2C_BufferReadRaw(data, 2, 0x04, addr);
            dxl_print_dbg("Read result: %d, voltage = %dmv\n", ret, *(uint16_t *) data);
            SpinDelay(500);
        }
    }
}

void test_bat_charger(void) {
    usb_enable(); // 要初始化USB口，printf调试大法好
    SpinDelay(1000);
    StartTicks();

    while (1) {
        while (is_btn_pressed()) {
        } // 等待松开按钮

        dxl_print_dbg("按下按钮开始测试充电器 > \r\n");

        I2C_init(true);

        // 等待按钮按下
        while (!is_btn_pressed()) {
        }

        // 用 I2C_BufferReadRaw 读取
        // 用 I2C_BufferWrite 写入

        uint8_t addr = 0x93; // 最高位为 1001001（地址位） [0|1]（读写位），所以我们给出的整字节地址就是 0x92 即可
        uint8_t data[20];

        // 0x00 是输入控制，电压和电流
        bool ret = I2C_BufferReadRaw(data, 1, 0x00, addr);
        dxl_print_dbg("Read result: %d, InputSourceCtrl: 0x%02x\n", ret, data[0]);

        // 0x09 故障寄存器
        ret = I2C_BufferReadRaw(data, 1, 0x09, addr);
        dxl_print_dbg("Read result: %d, Fault: 0x%02x\n", ret, data[0]);

#if 0 // 测试运输模式，会导致vmi关闭输出
        // 0x06 MainControl 默认0xC0 B5 是 FET_DIS
        ret = I2C_BufferReadRaw(data, 1, 0x06, addr);
        dxl_print_dbg("Read result: %d, MainControl: 0x%02x\n", ret, data[0]);
        // 判断是否处于运输模式，如果是，则不做啥，如果不是，则进入运输模式
        if ((data[0] >> 5) & 0x01) {
            dxl_print_dbg("Is shipping mode\n");
        } else {
            dxl_print_dbg("Not shipping mode, enter now.\n");
            data[0] |= (0x01 << 5); // 使能运输模式（FET_DIS = 1）
            ret = I2C_BufferWrite(data, 1, 0x06, addr);
            dxl_print_dbg("Write result: %d, MainControl: 0x%02x\n", ret, data[0]);
        }
#endif

#if 1
        // 0x09 PowerOnConfig 寄存器
        ret = I2C_BufferReadRaw(data, 1, 0x01, addr);
        dxl_print_dbg("Read result: %d, PowerOnCfg: 0x%02x\n", ret, data[0]);
        // 第三位是充电控制位，为0时使能充电
        if (!(data[0] >> 3 & 0x01)) {
            dxl_print_dbg("Is charge mode\n");
        } else {
            dxl_print_dbg("Not charge mode, enter now.\n");
            data[0] &= ~(0x01 << 3);
            ret = I2C_BufferWrite(data, 1, 0x01, addr);
            dxl_print_dbg("Write result: %d, MainControl: 0x%02x\n", ret, data[0]);
        }
        // 0x02 ChargeCurrentControl 寄存器
        ret = I2C_BufferReadRaw(data, 1, 0x02, addr);
        dxl_print_dbg("Read result: %d, ChargeCurrentControl: 0x%02x\n", ret, data[0]);
        // 写充电电流控制寄存器为 0x1F 将设置充电电流为 256ma
        data[0] = 0x1F;
        ret = I2C_BufferWrite(data, 1, 0x02, addr);
        dxl_print_dbg("Write result: %d, ChargeCurrentControl: 0x%02x\n", ret, data[0]);
#endif
    }
}

void test_bat_charger_only_settings(void) {
    StartTicks();
    I2C_init(true);

    uint8_t addr = 0x93; // 最高位为 1001001（地址位） [0|1]（读写位），所以我们给出的整字节地址就是 0x92 即可
    uint8_t data[20];

    // 0x09 PowerOnConfig 寄存器
    I2C_BufferReadRaw(data, 1, 0x01, addr);
    // 第三位是充电控制位，为0时使能充电
    if (!(data[0] >> 3 & 0x01)) {
        // 已经是充电模式
    } else {
        // 不是充电模式，现在进入充电模式
        data[0] &= ~(0x01 << 3);
        I2C_BufferWrite(data, 1, 0x01, addr);
    }

    // 0x02 ChargeCurrentControl 寄存器
    data[0] = 0x1F; // 写充电电流控制寄存器为 0x1F 将设置充电电流为 256ma
    I2C_BufferWrite(data, 1, 0x02, addr);

    // 0x03 DischargeCurrentControl 寄存器
    data[0] = 0xE1; // 修改放电电流到 3A
    I2C_BufferWrite(data, 1, 0x03, addr);

    // 0x05 ChargerTermination/TimerControl 寄存器
    data[0] = 0x1A; // 禁用定时器，正常为了安全可能需要在MainLoop喂狗
    I2C_BufferWrite(data, 1, 0x05, addr);

    // 0x0B IndividualChargeRegister 寄存器
    data[0] = 0x6B; // 将预充电电流调整到11ma
    I2C_BufferWrite(data, 1, 0x0B, addr);
}

void EXINT3_IRQHandler(void) {
    if (exint_interrupt_flag_get(EXINT_LINE_3) != RESET) {
        GpioOutputInv(GPIOA, GPIO_PINS_2); // 翻转pa2，传递库仑计+充电器的中断信号
        exint_flag_clear(EXINT_LINE_3);
    }
}

void test_coulometer_charger_int(void) {
    usb_enable(); // 要初始化USB口，printf调试大法好
    SpinDelay(1000);
    StartTicks();

    // 我们测试的时候，把 PWR_INT 接到扩展口的 RX 上了，需要配置该口为上拉输入
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_3;
    gpio_init(GPIOA, &gpio_init_struct); // PA3_RX

    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = GPIO_PINS_2;
    gpio_init(GPIOA, &gpio_init_struct); // PA2_TX

    // 配置复用为外部中断源
    exint_init_type exint_init_struct;
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE3);
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_select = EXINT_LINE_3;
    exint_init_struct.line_polarity = EXINT_TRIGGER_FALLING_EDGE; // PWR_INT 默认高电平，触发中断时产生下降沿
    exint_init(&exint_init_struct);
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(EXINT3_IRQn, 1, 0);


    while (1) {
        while (is_btn_pressed()) {
        } // 等待松开按钮

        dxl_print_dbg("按下按钮开始测试充电器&库仑计中断事件 > \r\n");

        I2C_init(true);

        // 等待按钮按下
        while (!is_btn_pressed()) {
        }

        // 用 I2C_BufferReadRaw 读取
        // 用 I2C_BufferWrite 写入

        bool ret;
        uint8_t data[20];
        uint8_t charger_addr = 0x93; // 最高位为 1001001（地址位） [0|1]（读写位），所以我们给出的整字节地址就是 0x92 即可
        uint8_t coulometer_addr = 0xAA; // 最高位为 1010101（地址位） [0|1]（读写位），所以我们给出的整字节地址就是 0xAA 即可

        // 我们需要打开充电
        ret = I2C_BufferReadRaw(data, 1, 0x01, charger_addr);
        dxl_print_dbg("Read result: %d, PowerOnCfg: 0x%02x\n", ret, data[0]);
        // 第三位是充电控制位，为0时使能充电
        if (!(data[0] >> 3 & 0x01)) {
            dxl_print_dbg("Is charge mode\n");
        } else {
            dxl_print_dbg("Not charge mode, enter now.\n");
            data[0] &= ~(0x01 << 3);
            ret = I2C_BufferWrite(data, 1, 0x01, charger_addr);
            dxl_print_dbg("Write result: %d, MainControl: 0x%02x\n", ret, data[0]);
        }

        // 读取事件
        uint8_t int_status = GpioInputStatus(GPIOA, GPIO_PINS_3);
        dxl_print_dbg("start listen event for PWR_INT, status on init: %d\n", int_status);
        while (1) {
#if 1 // 测试主动触发库仑计触发中断
            data[0] = 0x00;
            data[0] = 0x23;
            ret = I2C_BufferWrite(data, 2, 0x00, coulometer_addr);
            // dxl_print_dbg("PULSE_SOC_INT trigger: %d, status: %d\n", ret, GpioInputStatus(GPIOA, GPIO_PINS_3));
            SpinDelay(100);
#endif
#if 0 // 测试读取中断脚
            uint8_t int_new_status = GpioInputStatus(GPIOA, GPIO_PINS_3);
            if (int_status != int_new_status) {
                int_status = int_new_status;
                dxl_print_dbg("PWR_INT trigger\n");
            }
#endif
        }
    }
}

void test_4leds(void) {
    while (1) {
        // A灯闪烁
        LED_A_ON();
        SpinDelay(200);
        LED_A_OFF();
        SpinDelay(200);
        // B灯闪烁
        LED_B_ON();
        SpinDelay(200);
        LED_B_OFF();
        SpinDelay(200);
        // C灯闪烁
        LED_C_ON();
        SpinDelay(200);
        LED_C_OFF();
        SpinDelay(200);
        // D灯闪烁
        LED_D_ON();
        SpinDelay(200);
        LED_D_OFF();
        SpinDelay(200);
    }
}

void EXINT9_5_IRQHandler(void) {
    if (exint_interrupt_flag_get(EXINT_LINE_8) != RESET) {
        GpioOutputInv(GPIOA, GPIO_PINS_2); // 翻转pa2，传递中断信号
        exint_flag_clear(EXINT_LINE_8);
    }
}

#include "dbprint.h"

bool cep_spi_data_available(void) {
    uint8_t len_header[2] = {0x00};
    for (size_t i = 0; i < sizeof(len_header); ++i) {
        uint64_t timeout = 0;
        while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET) {
            if (timeout++ > 100000) {
                return 0; // 超时了，识别长度失败
            }
        }
        len_header[i] = spi_i2s_data_receive(SPI1);
    }
    uint16_t data_len = (len_header[1] << 8) | len_header[0];
    if (data_len > PM3_CMD_DATA_SIZE * 2) {
        return false; // 无效的数据长度，大于两倍payload大小这怎么可能
    }
    return true;
}

// 临时实现cep端口的spi读写函数，后续可以根据实际需求完善成通用的spi读写函数，目前先这样测试互联功能
uint32_t cep_spi_read_ng(uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        uint64_t timeout = 0;
        while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET) {
            if (timeout++ > 100000) {
                return i; // 超时了，返回已经读取的长度
            }
        }
        data[i] = spi_i2s_data_receive(SPI1);
    }
    return len;
}

int cep_spi_write_sync(uint8_t *data, size_t len) {
    // 头部两个字节是数据长度，这是我们约定好的SPI通信规范，SPI的从机应答的数据的头部两个字节一定要是数据长度
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET);
    spi_i2s_data_transmit(SPI1, len & 0xFF);

    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET);
    spi_i2s_data_transmit(SPI1, (len >> 8) & 0xFF);

    // 循环发送数据
    for (size_t i = 0; i < len; ++i) {
        while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET);
        spi_i2s_data_transmit(SPI1, data[i]);
    }

    // 让SPI电平归位为0，这是我们约定的每次通信结束后的电平状态，SPI的从机可以通过检测这个电平来判断通信是否结束
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET);
    spi_i2s_data_transmit(SPI1, 0x00);
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET);
    spi_i2s_data_transmit(SPI1, 0x00);

    // 等待传输结束
    while (spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) == SET);

    return PM3_SUCCESS;
}

void test_f0_com_by_usb_cep(void) {
#define TEST_F0_PRINT      0
#define TEST_F0_HANDSHAKE  0
#define TEST_F0_CONNECTION 1

    // 要初始化USB口，printf调试大法好
#if TEST_F0_PRINT
    usb_enable();
    SpinDelay(1000);
    dxl_print_dbg("The communication test for FlipperZero & Proxmark5 started.\r\n");
#endif

    LED_D_ON();

    // 配置UART
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_7);

    usart_init(USART1, 2400, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(USART1, USART_PARITY_NONE); // 8 n 1
    usart_transmitter_enable(USART1, FALSE); // 直接接收，不发送
    usart_receiver_enable(USART1, TRUE);
    usart_single_line_halfduplex_select(USART1, TRUE);
    usart_enable(USART1, TRUE);


    // 配置ID脚，识别主从
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_10; // PA10_USB1_ID
    gpio_init(GPIOA, &gpio_init_struct);


    // PA2_TX 拿来调试中断切换
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = GPIO_PINS_2;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init(GPIOA, &gpio_init_struct); // PA2_TX


    // 配置INT脚，专门就是CC控制器用的
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_8;
    gpio_init(GPIOC, &gpio_init_struct); // PC8_I2C_INT
    // 配置复用为外部中断源
    exint_init_type exint_init_struct;
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE8);
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_select = EXINT_LINE_8;
    exint_init_struct.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
    exint_init(&exint_init_struct);
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(EXINT9_5_IRQn, 1, 0);


    uint8_t data[20];
    uint8_t rx_len = 0;
    uint8_t is_cep_connected = false;
    uint8_t cc_ctrl_data;

    (void) data;
    (void) rx_len;
    (void) is_cep_connected;

#if TEST_F0_HANDSHAKE

    // 主从切换
    bool is_slave_mode = GpioInputStatus(GPIOA, GPIO_PINS_10);
    gpio_inter_usb_spi_role_setup();
    if (is_slave_mode) {
        Gpio_Inter_USB_SPI_Role_High();
    } else {
        Gpio_Inter_USB_SPI_Role_Low();
    }

#if TEST_F0_PRINT
    dxl_print_dbg("Current device mode: %s\n", is_slave_mode ? "Slave" : "Master");
#endif

    // SPI相关配置
    spi_master_slave_mode_type spi1_mode = is_slave_mode ? SPI_MODE_SLAVE : SPI_MODE_MASTER;
    gpio_init_type gpio_initstructure;
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    /* spi1 cs pin */
    gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_initstructure.gpio_pull = GPIO_PULL_UP;
    gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    if (spi1_mode == SPI_MODE_MASTER) {
        gpio_initstructure.gpio_mode = GPIO_MODE_OUTPUT;
    } else {
        gpio_initstructure.gpio_mode = GPIO_MODE_MUX;
        gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE4, GPIO_MUX_5);
    }
    gpio_initstructure.gpio_pins = GPIO_PINS_4;
    gpio_init(GPIOA, &gpio_initstructure);
    /* spi1 sck pin */
    gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_initstructure.gpio_pull = GPIO_PULL_DOWN;
    gpio_initstructure.gpio_mode = GPIO_MODE_MUX;
    gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_initstructure.gpio_pins = GPIO_PINS_5;
    gpio_init(GPIOA, &gpio_initstructure);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE5, GPIO_MUX_5);
    /* spi1 miso pin */
    gpio_initstructure.gpio_pull = GPIO_PULL_UP;
    gpio_initstructure.gpio_pins = GPIO_PINS_6;
    gpio_init(GPIOA, &gpio_initstructure);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE6, GPIO_MUX_5);
    /* spi1 mosi pin */
    gpio_initstructure.gpio_pull = GPIO_PULL_UP;
    gpio_initstructure.gpio_pins = GPIO_PINS_7;
    gpio_init(GPIOA, &gpio_initstructure);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_5);
    /* non communication time: master pull up CS pin release slave */
    if (spi1_mode == SPI_MODE_MASTER) {
        gpio_bits_set(GPIOA, GPIO_PINS_4);
    }
    spi_init_type spi_init_struct;
    /* master spi initialization */
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    spi_default_para_init(&spi_init_struct);
    /* dual line unidirectional full-duplex mode */
    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = spi1_mode;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_1024;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_1EDGE;
    if (spi1_mode == SPI_MODE_MASTER) {
        spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    } else {
        spi_init_struct.cs_mode_selection = SPI_CS_HARDWARE_MODE;
    }
    spi_init(SPI1, &spi_init_struct);
    spi_enable(SPI1, TRUE);

#endif


    // 初始化I2C，等下需要拿来做状态切换
    StartTicks();
    I2C_init(true);
    uint8_t cc_controller_addr = 0x47;


    while (1) {
#if TEST_F0_HANDSHAKE

        bool is_f0_msg = false;
        while (1) {
            if (usart_flag_get(USART1, USART_FERR_FLAG) != RESET) {
                dxl_print_dbg("RX frame error\n");
                GpioOutputInv(GPIOA, GPIO_PINS_2); // 翻转pa2，传递中断信号
            }
            if (usart_flag_get(USART1, USART_NERR_FLAG) != RESET) {
                dxl_print_dbg("RX noise error\n");
                GpioOutputInv(GPIOA, GPIO_PINS_2); // 翻转pa2，传递中断信号
            }

            // 一直尝试接收来自于互联口的UART的数据
            if (usart_flag_get(USART1, USART_RDBF_FLAG) != RESET) {
                uint8_t rx_data = usart_data_receive(USART1);
                if (rx_data == 0x02) {
                    rx_len = 0; // STX received, restart rx frame.
                    rx_data = usart_data_receive(USART1); // Clear data for next RX.
                    (void) rx_data;
                    continue;
                }
                data[rx_len++] = rx_data;
                if (rx_len == sizeof(data)) {
                    rx_len = 0;
                }
            }
            if (rx_len == 10) {
                // 打印调试
#if TEST_F0_PRINT
                data[10] = '\0';
                dxl_print_dbg("Rx msg: %s\n", data);
#endif
                rx_len = 0;
                // 如果确定是f0的轮询，则可以尝试进行回应
                if (memcmp("iamf0rupm5", data, 10) == 0) {
                    is_f0_msg = true;
                    break;
                }
            }
        }


        if (is_f0_msg) {
            // dxl_print_dbg("The message from FlipperZero by CEP(TypeC Extend Port)\n");
            uint8_t response_f0[] = {0x04, 0x00, 'y', 'e', 's', 0x00};
            for (int i = 0; i < sizeof(response_f0); i++) {
                uint32_t resp_wait_spi_tdbe = GetTicks(); // 1us = 1.5t
                while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET) {
                    if (GetTicks() - resp_wait_spi_tdbe > 1000 * 1000) {
                        // 等待超过1s还没有发送出去，说明可能和F0的连接已经断开了
                        // 如果和F0的连接已经断开，则不再进行等待传输
                        bool ret = I2C_BufferReadRaw(&cc_ctrl_data, 1, 0x09, cc_controller_addr << 1);
                        if (ret && (cc_ctrl_data >> 6 & 0x03) == 0) {
                            is_cep_connected = false;
#if TEST_F0_PRINT
                            dxl_print_dbg("Disconnected with F0 during spi transmission\n");
#endif
                            break;
                        }
                    }
                }
                spi_i2s_data_transmit(SPI1, response_f0[i]);
            }

            // TODO DXL 测试，直接进入主循环，通过SPI进行通信交互（CEP端口）
            AppMain();
        }

#endif

#if TEST_F0_CONNECTION

        // 检查和F0的互联是否已经断开
        bool ret = I2C_BufferReadRaw(&cc_ctrl_data, 1, 0x09, cc_controller_addr << 1);
        if (ret) {
#if TEST_F0_PRINT
            // dxl_print_dbg("cc_ctrl_data: %d\n", cc_ctrl_data);
#endif
            uint8_t attached_state = cc_ctrl_data >> 6 & 0x03;
            if (attached_state == 0x00) {
                // 断开状态，如果之前是连接状态的话，则我们需要告知断开事件发生
                if (is_cep_connected) {
#if TEST_F0_PRINT
                    dxl_print_dbg("Disconnected with F0\n");
#endif
                }
                is_cep_connected = false;
            } else {
                // 连接状态的话，如果之前是断开状态，则我们需要告知连接事件发生
                if (!is_cep_connected) {
#if TEST_F0_PRINT
                    dxl_print_dbg("Connected with F0\n");
#endif
                }
                is_cep_connected = true;
            }
            // 根据手册描述如果触发过CC控制器的中断，则我们需要清除，否则不会触发下一次中断
            uint8_t int_state = cc_ctrl_data >> 4 & 0x01;
            if (int_state) {
                cc_ctrl_data |= 1 << 4; // 正确的清除方式是bit4写1
                ret = I2C_BufferWrite(&cc_ctrl_data, 1, 0x09, cc_controller_addr << 1);
#if TEST_F0_PRINT
                dxl_print_dbg("Clear CC int reg: %d\n", ret);
#endif
                SpinDelay(100);
            }
        }

#endif
    }
}

void test_24c02(void) {
    // 初始化I2C，等下需要拿来做状态切换
    StartTicks();
    I2C_init(true);
    uint8_t addr_24c02 = 0x50;

    SpinDelay(1000);
    usb_enable(); // 要初始化USB口，printf调试大法好

    while (1) {
        while (is_btn_pressed()) {
        } // 等待松开按钮

        dxl_print_dbg("按下按钮开始测试 24c02 外部EEPROM > \r\n");
        while (!is_btn_pressed()) {
        } // 等待按下按钮

        // 用 I2C_BufferReadRaw 读取
        // 用 I2C_BufferWrite 写入

        uint8_t data[256];
        bool ret = I2C_BufferReadRaw(data, sizeof(data), 0x00, addr_24c02 << 1);
        dxl_print_dbg("Read result: %d, data: %s\n", ret, data);

        // 尝试解析为结构化的出厂数据
        struct factory_info_v1 {
            uint8_t factory_info_version;
            uint8_t ecdsa_secp256k1_signature[64];

            struct {
                uint64_t unix_timestamp;
                uint8_t chip_unique_id[12];
                uint32_t production_id;
                uint32_t hardware_version;
                uint8_t aes_key[16];
                uint8_t reserved[147];
            }
            PACKED info;
        }
        PACKED;
        struct factory_info_v1 *factory_info = (struct factory_info_v1 *) data;
        // 打印全部信息
        dxl_print_dbg("factory_info_version: %d\n", factory_info->factory_info_version);
        dxl_print_dbg("ecdsa_secp256k1_signature: ");
        for (size_t i = 0; i < sizeof(factory_info->ecdsa_secp256k1_signature); ++i) {
            dxl_print_dbg("%02x", factory_info->ecdsa_secp256k1_signature[i]);
        }
        dxl_print_dbg("\n");
        dxl_print_dbg("unix_timestamp: %llu\n", factory_info->info.unix_timestamp);
        dxl_print_dbg("chip_unique_id: ");
        for (size_t i = 0; i < sizeof(factory_info->info.chip_unique_id); ++i) {
            dxl_print_dbg("%02x", factory_info->info.chip_unique_id[i]);
        }

        // 打印从芯片端获取的唯一id
        dxl_print_dbg("\n");
        dxl_print_dbg("chip_unique_id(From runtime): ");
        uint8_t *chip_unique_id_runtime = GetChipUniqueId(NULL);
        for (size_t i = 0; i < sizeof(factory_info->info.chip_unique_id); ++i) {
            dxl_print_dbg("%02x", chip_unique_id_runtime[i]);
        }

        dxl_print_dbg("\n");
        dxl_print_dbg("production_id: %u\n", factory_info->info.production_id);
        dxl_print_dbg("hardware_version: %u\n", factory_info->info.hardware_version);
        dxl_print_dbg("aes_key: ");
        for (size_t i = 0; i < sizeof(factory_info->info.aes_key); ++i) {
            dxl_print_dbg("%02x", factory_info->info.aes_key[i]);
        }
        dxl_print_dbg("\n");

#if 0

        const char *test_str = "hello";
        ret = I2C_BufferWrite((uint8_t *) test_str, strlen(test_str), 0x00, addr_24c02 << 1);
        dxl_print_dbg("Write result: %d\n", ret);

#endif
    }
}

void test_vusb_check(void) {
    gpio_vusb_setup();
    volatile bool vusb = false;
    // 测试VUSB的供电检测很简单，直接读取GPIO口的电平然后亮灯即可，不需要USB通信（需要插着电池）
    while (1) {
        vusb = Gpio_VUSB_Read();
        if (vusb) {
            LED_D_ON();
        } else {
            LED_D_OFF();
        }
        (void) vusb;
    }
}

void test_bwm_uart(void) {
    usb_enable(); // 要初始化USB口，printf调试大法好

    /* enable the uart4 and gpio clock */
    crm_periph_clock_enable(CRM_UART4_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);

    /* configure the uart4 tx, rx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE0, GPIO_MUX_8);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE1, GPIO_MUX_8);

    /* configure uart4 param */
    usart_init(UART4, 460800, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(UART4, USART_PARITY_NONE);
    usart_transmitter_enable(UART4, TRUE);
    usart_receiver_enable(UART4, TRUE);
    usart_enable(UART4, TRUE);

    while (1) {
        while (is_btn_pressed()) {
        } // 等待松开按钮

        dxl_print_dbg("按下按钮开始测试 电池套件UART通信 > \r\n");
        while (!is_btn_pressed()) {
        } // 等待按下按钮

        // 测试发送和接收数据，看看BWM是否正常通信
        uint8_t data_tx[] = { 0x7c, 0xc7, 0xfa, 0x03, 0x00, 0x00, 0xb5, 0xba };
        for (size_t i = 0; i < sizeof(data_tx); ++i) {
            while (usart_flag_get(UART4, USART_TDBE_FLAG) == RESET);
            usart_data_transmit(UART4, data_tx[i]);
        }
        uint8_t data_rx[9] = { 0x00 }; // 应答是 2d3dfa03010001c8d1
        for (size_t i = 0; i < sizeof(data_rx); ++i) {
            while (usart_flag_get(UART4, USART_RDBF_FLAG) == RESET);
            data_rx[i] = usart_data_receive(UART4);
        }
        dxl_print_dbg("Received data from BWM: ");
        for (size_t i = 0; i < sizeof(data_rx); ++i) {
            dxl_print_dbg("%02x", data_rx[i]);
        }
        dxl_print_dbg("\n");
    }
}

void test_config_uart_tx2_to_dbgio(void) {
    // 配置uart tx2 为推挽输出
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = GPIO_PINS_2;
    gpio_init(GPIOA, &gpio_init_struct); // PA2_TX
}

#include "fpga_loader.h"
#include "lfsampling.h"

void test_max_power(void) {
    FpgaResetComInterface();
    FpgaWriteConfWord(FPGA_MAJOR_MODE_OFF);

    SpinDelay(100);
    FpgaSetup24MHzClk();

    uint8_t data;

    // 开所有的灯
    LED_A_ON();
    LED_B_ON();
    LED_C_ON();
    LED_D_ON();
    test_i2c_rgb_simple();

    // 0x03 DischargeCurrentControl 寄存器
    data = 0xE1; // 修改放电电流到 3A
    I2C_BufferWrite(&data, 1, 0x03, 0x93);
    // 0x05 ChargerTermination/TimerControl 寄存器
    data = 0x1A; // 禁用定时器，正常为了安全可能需要在MainLoop喂狗
    I2C_BufferWrite(&data, 1, 0x05, 0x93);

#if 0
    FpgaDownloadAndGo(FPGA_BITSTREAM_HF);
    FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER);
#else

    // 开场
    // FPGA_MAJOR_MODE_LF_READER | FPGA_LF_ADC_READER_FIELD
    // FPGA_MAJOR_MODE_HF_READER
    // FpgaWriteConfWord(FPGA_MAJOR_MODE_LF_READER | FPGA_LF_ADC_READER_FIELD);
    LFSetupFPGAForADC(LF_DIVISOR_125, true);

    gpio_fpga_mod_only_setup();
    Gpio_SSC_DOUT_Low();

    // 配置低频高Q
    data = 0x87; // 0x87 = 10000111: 125k + 高Q + 开启两个LED
    I2C_BufferWrite(&data, 1, 0x02, 0x51 << 1);

#endif

    // 调到最高输出电压
    FpgaSendCommand(FPGA_CMD_SET_PWR_PWM_LOW_COUNT, 4095u & 0xFFF);

    // 蜂鸣器持续响着
    test_beep();
}

void test_nvic_reset(void) {
    LED_A_ON();
    SpinDelay(100);
    LED_A_OFF();
    LED_B_ON();
    while (1) {
        if (is_btn_pressed()) {
            LED_B_OFF();
            LED_C_ON();
            while (is_btn_pressed()) {}
            NVIC_SystemReset();
        }
    }
}

#ifndef SMARTCARD_SYNC_TEST
#define SMARTCARD_SYNC_TEST 1
#endif

#if !SMARTCARD_SYNC_TEST
// ---------------------------------------------------------------------------
// ISO7816 T=0（异步卡）最小收发实现，使用 AT32 USART 的 SmartCard 模式。
//
// SmartCard 模式本身就是单线半双工：TX 引脚（PA2）既发也收，不要再打开
// SLBEN（单线半双工），两者同时开会把 TX 波形屏蔽掉。卡片时钟仍由
// TMR2 -> PA3 -> TDA8035 CLKIN 提供（TDA8035 再分频给卡片），USART 自己的
// CK（PA4）没有接线，保持关闭。
//
// T=0 的特点：发完 5 字节命令头后，卡片会先回一个 procedure byte，
//   0x60      = NULL，继续等
//   INS       = ACK，接受后续所有数据（或表示数据开始输出）
//   ~INS      = ACK，只接受下一个数据字节
//   0x6C xx   = Le 不对，用 xx 重发命令头
//   0x61 xx   = 还有 xx 字节，需要发 GET RESPONSE 取回
//   其它 6X/9X = SW1，随后跟 SW2，本次交换结束
// 时序基准：GetTicks() 为 1.5MHz（StartTicks 已在上层调用），1 tick = 2/3us
// ---------------------------------------------------------------------------

#define SC_USART           USART2
#define SC_IO_PORT         GPIOA
#define SC_IO_PIN          GPIO_PINS_2
#define SC_CLK_PIN         GPIO_PINS_3
#define SC_DEFAULT_BAUD    9677u
#define SC_CLOCK_DEFAULT   3600000u  // TDA8035 fXTAL/2
#define SC_CLOCK_FAST      7200000u  // TDA8035 fXTAL
#define SC_RX_TIMEOUT_TICKS 225000u  // 150ms，T=0 的 work waiting time
#define SC_TX_TIMEOUT_TICKS 75000u   // 50ms，留足 TC1 大保护时间的余量
#define SC_TICK_US(ticks)  ((uint32_t)(ticks) * 2u / 3u)

// ISO/IEC 7816-3 表 7/8：TA1 的高/低半字节分别是 Fi/Di 的索引。
static const uint16_t sc_fi_tab[16] = {372, 372, 558, 744, 1116, 1488, 1860, 0,
                                       0, 512, 768, 1024, 1536, 2048, 0, 0
                                      };
static const uint8_t sc_di_tab[16] = {0, 1, 2, 4, 8, 16, 32, 64,
                                      12, 20, 0, 0, 0, 0, 0, 0
                                     };
static const uint16_t sc_fmax_khz[16] = {4000, 5000, 6000, 8000, 12000, 16000, 20000, 0,
                                         0, 5000, 7500, 10000, 15000, 20000, 0, 0
                                        };

static bool sc_reg_write(uint8_t addr, uint8_t reg, uint8_t val) {
    return I2C_BufferWrite(&val, 1, reg, (uint8_t)(addr << 1));
}

// 只改波特率/帧格式；SCEN 临时关掉再打开，避免帧格式位被模式位锁住。
static void sc_set_baud(uint32_t baud) {
    usart_enable(SC_USART, FALSE);
    usart_smartcard_mode_enable(SC_USART, FALSE);
    usart_init(SC_USART, baud, USART_DATA_9BITS, USART_STOP_1_5_BIT);
    usart_parity_selection_config(SC_USART, USART_PARITY_EVEN);
    usart_smartcard_mode_enable(SC_USART, TRUE);
    usart_enable(SC_USART, TRUE);
}

static void sc_set_guard_time(uint8_t gt) {
    usart_enable(SC_USART, FALSE);
    usart_smartcard_guard_time_set(SC_USART, gt);
    usart_enable(SC_USART, TRUE);
}

static void sc_usart_init(uint32_t baud) {
    usart_enable(SC_USART, FALSE);
    usart_init(SC_USART, baud, USART_DATA_9BITS, USART_STOP_1_5_BIT);
    usart_parity_selection_config(SC_USART, USART_PARITY_EVEN);
    usart_smartcard_guard_time_set(SC_USART, 2);
    // SmartCard 模式自带单线半双工，SLBEN 必须关闭。
    usart_single_line_halfduplex_select(SC_USART, FALSE);
    // PA4 没有接到卡片，CK 输出保持关闭；时钟走 TMR2/PA3。
    usart_clock_enable(SC_USART, FALSE);
    usart_transmitter_enable(SC_USART, TRUE);
    usart_receiver_enable(SC_USART, TRUE);
    usart_enable(SC_USART, TRUE);
    // 与 AT32 参考例程一致：UEN 之后再开 NACK / SmartCard 模式。
    usart_smartcard_nack_set(SC_USART, TRUE);
    usart_smartcard_mode_enable(SC_USART, TRUE);
}

static void sc_rx_flush(void) {
    while (usart_flag_get(SC_USART, USART_RDBF_FLAG) != RESET) {
        (void) usart_data_receive(SC_USART);
    }
    usart_flag_clear(SC_USART, USART_PERR_FLAG | USART_FERR_FLAG |
                     USART_NERR_FLAG | USART_ROERR_FLAG | USART_TDC_FLAG);
}

// 发出一个字符：先等发送缓冲空，发完再等 TDC 确认该字符已经完整移出到 I/O 线上。
// 只等 TDBE 是不够的：TDBE 只表示数据寄存器空，半双工方向切换过早会把最后几位切掉。
static bool sc_t0_send_byte(uint8_t b) {
    uint32_t start = GetTicks();
    while (usart_flag_get(SC_USART, USART_TDBE_FLAG) == RESET) {
        if (GetTicksDelta(start) > SC_TX_TIMEOUT_TICKS) {
            return false;
        }
    }
    usart_data_transmit(SC_USART, b);

    start = GetTicks();
    while (usart_flag_get(SC_USART, USART_TDC_FLAG) == RESET) {
        if (GetTicksDelta(start) > SC_TX_TIMEOUT_TICKS) {
            return false;
        }
    }
    usart_flag_clear(SC_USART, USART_TDC_FLAG);

    // 接收器挂在同一根 I/O 上，清掉自己刚发出去的回灌字节。
    // 卡片不可能在 TDC 后几个微秒内回应，所以这里不会吞掉真正的应答。
    while (usart_flag_get(SC_USART, USART_RDBF_FLAG) != RESET) {
        (void) usart_data_receive(SC_USART);
    }
    return true;
}

// 接收一个字符，超时返回 false。
// ISO7816 T=0 的 work waiting time 默认是 960 etu，9677bps 下约 100ms，
// 所以这里必须给足余量（150ms），否则会把正常的处理延迟误判成无响应。
static bool sc_t0_recv_byte_timeout(uint8_t *b, uint32_t timeout_ticks) {
    uint32_t start = GetTicks();
    while (usart_flag_get(SC_USART, USART_RDBF_FLAG) == RESET) {
        if (GetTicksDelta(start) > timeout_ticks) {
            return false;
        }
    }
    *b = (uint8_t) usart_data_receive(SC_USART);
    return true;
}

static bool sc_t0_recv_byte(uint8_t *b) {
    return sc_t0_recv_byte_timeout(b, SC_RX_TIMEOUT_TICKS);
}

// 发一条 APDU 并把应答数据读进 out。返回 SW（0x9000 正常），0xFFFF 表示通讯超时/异常。
static uint16_t sc_t0_apdu(uint8_t cla, uint8_t ins, uint8_t p1, uint8_t p2,
                           const uint8_t *in, uint8_t lc, uint8_t le,
                           uint8_t *out, uint16_t out_max, uint16_t *out_len) {
    uint8_t  cur_ins = ins;
    uint8_t  rx;
    uint8_t  sw2;
    uint8_t  sent = 0;
    uint16_t n = 0;
    // 还需要从卡片读回的字节数。T=0 的 case 2 命令里 P3=0 表示 Le=256
    // （READ RECORD 就是 Le=0）；SELECT 这类带 Lc 的命令不受影响。
    uint16_t remain = (lc == 0 && le == 0) ? 256 : le;
    uint16_t guard;

    *out_len = 0;

    // 丢掉之前残留的字节
    sc_rx_flush();

    // 命令头：P3 有数据时是 Lc，否则是 Le
    if (!sc_t0_send_byte(cla) || !sc_t0_send_byte(ins) || !sc_t0_send_byte(p1) ||
            !sc_t0_send_byte(p2) || !sc_t0_send_byte(lc ? lc : le)) {
        return 0xFFFF;
    }

    for (guard = 0; guard < 1000; ++guard) {
        if (!sc_t0_recv_byte(&rx)) {
            // MUX 模式下 GPIOA->idt 仍反映引脚真实电平：
            //   1 = 总线空闲高（卡片没回应，命令可能没送到）
            //   0 = 卡片正拉着总线低（卡片在输出，但我们没收到 -> 波特率/校验位不匹配）
            dxl_print_dbg("    RX timeout: I/O=%d sts=0x%03x (PERR=%d FERR=%d NERR=%d ROVER=%d)\n",
                          GpioInputStatus(SC_IO_PORT, SC_IO_PIN), (unsigned) SC_USART->sts,
                          (SC_USART->sts & USART_PERR_FLAG) ? 1 : 0,
                          (SC_USART->sts & USART_FERR_FLAG) ? 1 : 0,
                          (SC_USART->sts & USART_NERR_FLAG) ? 1 : 0,
                          (SC_USART->sts & USART_ROERR_FLAG) ? 1 : 0);
            return 0xFFFF;
        }

        if (rx == 0x60) { // NULL
            continue;
        }

        // 送数据阶段：卡片 ACK 后再送数据
        if (sent < lc && (rx == cur_ins || rx == (uint8_t) ~cur_ins)) {
            uint8_t cnt = (rx == cur_ins) ? (uint8_t)(lc - sent) : 1;
            while (cnt-- > 0 && sent < lc) {
                if (!sc_t0_send_byte(in[sent++])) {
                    return 0xFFFF;
                }
            }
            continue;
        }

        // 数据开始输出：INS = 后面全部数据；~INS = 每次只给一个字节
        if (remain > 0 && (rx == cur_ins || rx == (uint8_t) ~cur_ins)) {
            uint16_t cnt = (rx == cur_ins) ? remain : 1;
            while (cnt-- > 0 && remain > 0) {
                if (!sc_t0_recv_byte(&rx)) {
                    return 0xFFFF;
                }
                if (n < out_max) {
                    out[n++] = rx;
                }
                remain--;
            }
            *out_len = n;
            continue;
        }

        // 卡片说还有数据，发 GET RESPONSE 把它取回来。
        // GET RESPONSE 的 CLA 用逻辑通道 0（0x00），不能沿用 GPO 的 0x80。
        if (rx == 0x61) {
            uint8_t more;
            if (!sc_t0_recv_byte(&more)) {
                return 0xFFFF;
            }
            uint8_t get_resp_cla = (uint8_t)(cla & 0x03);
            if (!sc_t0_send_byte(get_resp_cla) || !sc_t0_send_byte(0xC0) ||
                    !sc_t0_send_byte(0x00) || !sc_t0_send_byte(0x00) ||
                    !sc_t0_send_byte(more)) {
                return 0xFFFF;
            }
            cur_ins = 0xC0;
            sent = lc;   // GET RESPONSE 没有数据段
            remain = more;
            continue;
        }

        // Le 不对：按卡片要求换长度重发命令头
        if (rx == 0x6C && lc == 0) {
            uint8_t want;
            if (!sc_t0_recv_byte(&want)) {
                return 0xFFFF;
            }
            if (!sc_t0_send_byte(cla) || !sc_t0_send_byte(cur_ins) || !sc_t0_send_byte(p1) ||
                    !sc_t0_send_byte(p2) || !sc_t0_send_byte(want)) {
                return 0xFFFF;
            }
            remain = want;
            continue;
        }

        // 剩下的都是 SW1
        if (!sc_t0_recv_byte(&sw2)) {
            return 0xFFFF;
        }
        *out_len = n;
        return ((uint16_t) rx << 8) | sw2;
    }

    *out_len = n;
    return 0xFFFF;
}

// 量两个连续字符起始沿的间隔。GT=0 与 GT=2 各测一次，差值应约为 2 etu。
// 发 0x00（偶校验）：起始位之后全是 0，停止位前没有其它下降沿，
// 所以第二个下降沿一定是下一个字符的起始位。
// 测量时卡片保持复位，I/O 高阻，不会干扰。
static uint32_t sc_measure_char_ticks(void) {
    uint32_t prev = 1;
    uint32_t t_first = 0;
    uint32_t start;
    uint8_t edges = 0;

    sc_rx_flush();
    start = GetTicks();
    while (usart_flag_get(SC_USART, USART_TDBE_FLAG) == RESET) {
        if (GetTicksDelta(start) > SC_TX_TIMEOUT_TICKS) {
            return 0;
        }
    }
    usart_data_transmit(SC_USART, 0x00);

    start = GetTicks();
    while (GetTicksDelta(start) < 60000) { // 40ms 上限
        uint32_t lvl = GpioInputStatus(SC_IO_PORT, SC_IO_PIN);
        if (prev && !lvl) {
            uint32_t now = GetTicks();
            if (edges == 0) {
                t_first = now;
                // 智能卡模式下保护时间计入 TDC：等 TDC 过去再发第二个字符，
                // 这样两个起始沿的间隔才包含 SCGT。
                uint32_t t = GetTicks();
                while (usart_flag_get(SC_USART, USART_TDC_FLAG) == RESET) {
                    if (GetTicksDelta(t) > SC_TX_TIMEOUT_TICKS) {
                        return 0;
                    }
                }
                usart_flag_clear(SC_USART, USART_TDC_FLAG);
                usart_data_transmit(SC_USART, 0x00);
            } else if (edges == 1) {
                // 等第二个字符发完，避免下一步关 UEN 时把它截断。
                uint32_t t = GetTicks();
                while (usart_flag_get(SC_USART, USART_TDC_FLAG) == RESET) {
                    if (GetTicksDelta(t) > SC_TX_TIMEOUT_TICKS) {
                        break;
                    }
                }
                sc_rx_flush();
                return now - t_first;
            }
            edges++;
        }
        prev = lvl;
    }
    sc_rx_flush();
    return 0;
}

#define SC_ATR_IDLE_TICKS 45000u // 30ms 无数据判定 ATR 结束

static uint8_t sc_read_atr(uint8_t *atr, uint8_t max_len) {
    uint8_t n = 0;
    uint32_t idle = GetTicks();
    while (1) {
        if (usart_flag_get(SC_USART, USART_RDBF_FLAG) != RESET) {
            uint8_t b = (uint8_t) usart_data_receive(SC_USART);
            if (n < max_len) {
                atr[n++] = b;
            }
            idle = GetTicks();
            continue;
        }
        if (GetTicksDelta(idle) > SC_ATR_IDLE_TICKS) {
            break;
        }
    }
    return n;
}

typedef struct {
    uint8_t len;
    uint8_t ts;
    uint8_t t0;
    uint8_t ta1, tb1, tc1, td1;
    uint8_t proto;              // 第一个 TD 给出的协议；没有 TD1 时为 T=0
    uint8_t hist_off, hist_len;
    bool    has_ta1, has_tc1;
    bool    t0_only;            // 只提供 T=0，此时不需要 TCK
    bool    tck_expected;
    bool    tck_present;
    bool    tck_ok;
} sc_atr_info_t;

// 解析并打印 ATR 的 TS/T0/TA1/TB1/TC1/TDi/历史字节，并校验 TCK。
static void sc_atr_parse(const uint8_t *atr, uint8_t len, sc_atr_info_t *info) {
    uint8_t b[40];
    uint8_t i, level, present, k;
    bool non_t0 = false;

    memset(info, 0, sizeof(*info));
    info->len = len;
    info->t0_only = true;
    if (len < 2 || len > sizeof(b)) {
        dxl_print_dbg("ATR: bad length %u\n", len);
        return;
    }
    memcpy(b, atr, len);

    info->ts = b[0];
    if (b[0] == 0x3F) {
        for (i = 1; i < len; ++i) {
            b[i] = (uint8_t) ~b[i];
        }
        dxl_print_dbg("  TS=0x3F (inverse convention)\n");
    } else if (b[0] == 0x3B) {
        dxl_print_dbg("  TS=0x3B (direct convention)\n");
    } else {
        dxl_print_dbg("  TS=0x%02x (unknown convention)\n", b[0]);
        return;
    }

    info->t0 = b[1];
    k = b[1] & 0x0F;
    dxl_print_dbg("  T0=0x%02x:", b[1]);
    if (b[1] & 0x10) dxl_print_dbg(" TA1");
    if (b[1] & 0x20) dxl_print_dbg(" TB1");
    if (b[1] & 0x40) dxl_print_dbg(" TC1");
    if (b[1] & 0x80) dxl_print_dbg(" TD1");
    dxl_print_dbg(", %u historical byte(s)\n", k);

    i = 2;
    level = 1;
    present = (uint8_t)((b[1] >> 4) & 0x0F);
    while (1) {
        uint8_t ta = 0, tb = 0, tc = 0, td = 0;

        if (present & 0x1) {
            if (i >= len) { dxl_print_dbg("  ATR truncated\n"); return; }
            ta = b[i++];
        }
        if (present & 0x2) {
            if (i >= len) { dxl_print_dbg("  ATR truncated\n"); return; }
            tb = b[i++];
        }
        if (present & 0x4) {
            if (i >= len) { dxl_print_dbg("  ATR truncated\n"); return; }
            tc = b[i++];
        }
        if (present & 0x8) {
            if (i >= len) { dxl_print_dbg("  ATR truncated\n"); return; }
            td = b[i++];
        }

        if (present & 0x1) {
            if (level == 1) {
                info->ta1 = ta;
                info->has_ta1 = true;
            }
            dxl_print_dbg("  TA%u=0x%02x", level, ta);
            if (level == 1) {
                uint16_t fi = sc_fi_tab[(ta >> 4) & 0x0F];
                uint8_t di = sc_di_tab[ta & 0x0F];
                uint16_t fmax = sc_fmax_khz[(ta >> 4) & 0x0F];
                if (fi && di) {
                    dxl_print_dbg(": Fi=%u Di=%u, F/D=%u", fi, di, (unsigned)(fi / di));
                    if (fmax) {
                        dxl_print_dbg(", f_max=%u.%uMHz", fmax / 1000, (fmax % 1000) / 100);
                    }
                } else {
                    dxl_print_dbg(": RFU Fi/Di");
                }
            }
            dxl_print_dbg("\n");
        }
        if (present & 0x2) {
            if (level == 1) info->tb1 = tb;
            dxl_print_dbg("  TB%u=0x%02x%s\n", level, tb, tb == 0 ? " (no VPP)" : "");
        }
        if (present & 0x4) {
            if (level == 1) {
                info->tc1 = tc;
                info->has_tc1 = true;
            }
            if (tc == 0xFF) {
                dxl_print_dbg("  TC%u=0xFF (12 etu minimum)\n", level);
            } else {
                dxl_print_dbg("  TC%u=0x%02x (extra guard time %u etu)\n", level, tc, tc);
            }
        }
        if (present & 0x8) {
            if (level == 1) {
                info->td1 = td;
                info->proto = td & 0x0F;
            }
            if ((td & 0x0F) != 0) {
                non_t0 = true;
            }
            uint8_t next = (uint8_t)((td >> 4) & 0x0F);
            dxl_print_dbg("  TD%u=0x%02x: protocol T=%u", level, td, td & 0x0F);
            if (next) {
                // TA/TB/TC(i+1) 可以在没有 TD(i+1) 的情况下单独出现。
                dxl_print_dbg(" (more interface bytes follow)\n");
                present = next;
                level++;
                continue;
            }
            dxl_print_dbg("\n");
        }
        break;
    }

    info->t0_only = !non_t0;
    info->tck_expected = non_t0;
    if (!info->has_ta1) {
        dxl_print_dbg("  TA1 absent: default Fi=372 Di=1\n");
    }
    if (!info->t0_only) {
        dxl_print_dbg("  Non-T=0 protocol offered: TCK required\n");
    }

    if (i + k > len) {
        dxl_print_dbg("  ATR length mismatch (need %u, got %u)\n", i + k, len);
        k = (i < len) ? (uint8_t)(len - i) : 0;
    }
    info->hist_off = i;
    info->hist_len = k;
    if (k) {
        char hex[3 * 32 + 1];
        uint8_t hn = 0;
        for (uint8_t j = 0; j < k && j < 32; ++j) {
            hex[hn++] = "0123456789ABCDEF"[b[i + j] >> 4];
            hex[hn++] = "0123456789ABCDEF"[b[i + j] & 0x0F];
            hex[hn++] = ' ';
        }
        hex[hn] = 0;
        dxl_print_dbg("  Historical (%u): %s\n", k, hex);
        if (b[i] == 0x00 || b[i] == 0x80) {
            dxl_print_dbg("    category: status indicator\n");
        } else if ((b[i] >= 0x01 && b[i] <= 0x7F) || (b[i] >= 0x81 && b[i] <= 0x8F)) {
            dxl_print_dbg("    category: proprietary\n");
        } else {
            dxl_print_dbg("    category: RFU 0x%02x\n", b[i]);
        }
        i += k;
    }

    if (i < len) {
        uint8_t x = 0;
        info->tck_present = true;
        // 用原始收到的字节校验；反向约定时 b[] 是翻转后的副本。
        for (uint8_t j = 1; j <= i; ++j) {
            x ^= atr[j];
        }
        info->tck_ok = (x == 0);
        dxl_print_dbg("  TCK=0x%02x: %s%s\n", atr[i],
                      info->tck_ok ? "OK" : "BAD",
                      info->tck_expected ? "" : " (optional for T=0)");
        if (i + 1 < len) {
            dxl_print_dbg("  %u trailing byte(s) after TCK\n", len - i - 1);
        }
    } else {
        if (info->tck_expected) {
            info->tck_ok = false;
            dxl_print_dbg("  TCK missing (a non-T=0 protocol is offered)\n");
        } else {
            dxl_print_dbg("  TCK absent (T=0 only) - OK\n");
        }
    }
}

// 按 TA1 和实际卡片时钟算波特率：baud = clock * Di / Fi。
static uint32_t sc_baud_from_ta1(uint8_t ta1, uint32_t clock_hz) {
    uint16_t fi = sc_fi_tab[(ta1 >> 4) & 0x0F];
    uint8_t di = sc_di_tab[ta1 & 0x0F];
    if (fi == 0 || di == 0) {
        return 0;
    }
    return (clock_hz * di) / fi;
}

// ISO7816-3 PPS 交换。PPS0 bit4=1 表示带 PPS1（TA1=Fi/Di）。
// 成功返回 true，并把卡片实际生效的 TA1 写到 *accepted。
static bool sc_pps(uint8_t ta1, uint8_t *accepted) {
    uint8_t req[4];
    uint8_t rsp[8];
    uint8_t n = 0;
    uint8_t b;
    uint8_t x = 0;

    req[0] = 0xFF;
    req[1] = 0x10; // T=0，带 PPS1
    req[2] = ta1;
    req[3] = (uint8_t)(req[0] ^ req[1] ^ req[2]);

    sc_rx_flush();
    for (uint8_t i = 0; i < 4; ++i) {
        if (!sc_t0_send_byte(req[i])) {
            dxl_print_dbg("PPS: send timeout\n");
            return false;
        }
    }

    if (!sc_t0_recv_byte(&b)) {
        dxl_print_dbg("PPS: no response\n");
        return false;
    }
    rsp[n++] = b;
    while (n < sizeof(rsp) && sc_t0_recv_byte_timeout(&b, 15000)) { // 10ms 字符间隔
        rsp[n++] = b;
    }

    dxl_print_dbg("PPS response (%u):", n);
    for (uint8_t i = 0; i < n; ++i) {
        dxl_print_dbg(" %02x", rsp[i]);
    }
    dxl_print_dbg("\n");

    if (n < 3 || rsp[0] != 0xFF) {
        dxl_print_dbg("PPS: malformed response\n");
        return false;
    }
    for (uint8_t i = 0; i < n; ++i) {
        x ^= rsp[i];
    }
    if (x != 0) {
        dxl_print_dbg("PPS: PCK error (xor=0x%02x)\n", x);
        return false;
    }
    if ((rsp[1] & 0x0F) != 0) {
        dxl_print_dbg("PPS: card selected T=%u, not T=0\n", rsp[1] & 0x0F);
        return false;
    }
    if ((rsp[1] & 0x10) == 0) {
        dxl_print_dbg("PPS: card refused PPS1\n");
        return false;
    }
    *accepted = (n >= 4) ? rsp[2] : ta1;
    return true;
}

// ---------------------------------------------------------------- TLV 解析
static const char *sc_tag_name(uint16_t tag) {
    switch (tag) {
        case 0x4F:   return "ADF name";
        case 0x50:   return "Application label";
        case 0x57:   return "Track 2 equivalent";
        case 0x5A:   return "PAN (card number)";
        case 0x5F20: return "Cardholder name";
        case 0x5F24: return "Expiry date (YYMMDD)";
        case 0x5F25: return "Effective date (YYMMDD)";
        case 0x5F28: return "Issuer country";
        case 0x5F2D: return "Language";
        case 0x5F34: return "PAN sequence number";
        case 0x61:   return "Application template";
        case 0x6F:   return "FCI template";
        case 0x70:   return "Record template";
        case 0x80:   return "GPO response (AIP/AFL)";
        case 0x84:   return "DF name";
        case 0x87:   return "Application priority";
        case 0x88:   return "Directory SFI";
        case 0x8C:   return "CDOL1";
        case 0x8D:   return "CDOL2";
        case 0x94:   return "AFL";
        case 0xA5:   return "FCI proprietary";
        case 0x9F07: return "Usage control (AUC)";
        case 0x9F08: return "Application version";
        case 0x9F0E: return "Issuer action (deny)";
        case 0x9F0F: return "Issuer action (online)";
        case 0x9F10: return "Issuer application data";
        case 0x9F12: return "Preferred name";
        case 0x9F1F: return "Track 1 discretionary";
        case 0x9F38: return "PDOL";
        case 0x9F42: return "Currency code";
        case 0xBF0C: return "FCI issuer discretionary";
        default:     return "";
    }
}

static void sc_print_bcd(const uint8_t *v, uint16_t len, uint8_t max_chars) {
    char out[64];
    uint8_t n = 0;
    for (uint16_t i = 0; i < len && n < max_chars && n < sizeof(out) - 1; ++i) {
        uint8_t hi = v[i] >> 4;
        uint8_t lo = v[i] & 0x0F;
        if (hi <= 9) out[n++] = (char)('0' + hi);
        if (lo <= 9 && n < max_chars && n < sizeof(out) - 1) {
            out[n++] = (char)('0' + lo);
        }
    }
    out[n] = 0;
    dxl_print_dbg("%s", out);
}

// Track 2 equivalent data（0x57）：PAN 'D' YYMM 服务码，BCD 打包。
static void sc_show_track2(const uint8_t *v, uint16_t len) {
    char pan[24];
    char exp[5];
    uint8_t pn = 0, en = 0;
    bool after_d = false;

    for (uint16_t i = 0; i < len; ++i) {
        uint8_t nib[2] = { (uint8_t)(v[i] >> 4), (uint8_t)(v[i] & 0x0F) };
        for (uint8_t k = 0; k < 2; ++k) {
            uint8_t d = nib[k];
            if (!after_d) {
                if (d == 0x0D) {
                    after_d = true;
                } else if (d <= 9 && pn < sizeof(pan) - 1) {
                    pan[pn++] = (char)('0' + d);
                }
            } else if (d <= 9 && en < 4) {
                exp[en++] = (char)('0' + d);
            }
        }
    }
    pan[pn] = 0;
    exp[en] = 0;
    dxl_print_dbg(" track2_PAN=%s", pan);
    if (en == 4) {
        dxl_print_dbg(" expiry=20%c%c-%c%c", exp[0], exp[1], exp[2], exp[3]);
    }
}

static void sc_tlv_show_value(uint16_t tag, const uint8_t *v, uint16_t len) {
    if (tag == 0x5A) {
        dxl_print_dbg(" PAN=");
        sc_print_bcd(v, len, 19);
    } else if (tag == 0x5F24 && len == 3) {
        dxl_print_dbg(" expiry=20%02x-%02x-%02x", v[0], v[1], v[2]);
    } else if (tag == 0x5F25 && len == 3) {
        dxl_print_dbg(" effective=20%02x-%02x-%02x", v[0], v[1], v[2]);
    } else if (tag == 0x5F20) {
        dxl_print_dbg(" name=\"");
        for (uint16_t i = 0; i < len; ++i) {
            uint8_t c = v[i];
            dxl_print_dbg("%c", (c >= 0x20 && c < 0x7F) ? c : '.');
        }
        dxl_print_dbg("\"");
    } else if (tag == 0x57) {
        sc_show_track2(v, len);
    } else if (tag == 0x5F28 && len == 2) {
        dxl_print_dbg(" country=%02x%02x", v[0], v[1]);
    } else if (tag == 0x88 && len >= 1) {
        dxl_print_dbg(" SFI=%u", v[0]);
    } else if (tag == 0x9F42 && len == 2) {
        dxl_print_dbg(" currency=%02x%02x", v[0], v[1]);
    } else if (tag == 0x5F2D && len == 2) {
        dxl_print_dbg(" lang=%c%c", v[0], v[1]);
    } else if (tag == 0x80 && len >= 2) {
        // GPO 响应：2 字节 AIP + 若干 4 字节 AFL 项。
        dxl_print_dbg(" AIP=%02x%02x", v[0], v[1]);
        for (uint16_t i = 2; i + 3 < len; i += 4) {
            uint8_t sfi = (uint8_t)((v[i] >> 3) & 0x1F);
            dxl_print_dbg("; SFI%u rec%u-%u", sfi, v[i + 1], v[i + 2]);
        }
    } else if (tag == 0x9F38) {
        // PDOL 是 tag/length 列表，本身没有 value；这里只显示各项需求。
        uint16_t off = 0;
        dxl_print_dbg(" PDOL:");
        while (off + 1 < len) {
            uint16_t t = v[off++];
            if ((t & 0x1F) == 0x1F) {
                if (off >= len) break;
                t = (uint16_t)((t << 8) | v[off++]);
            }
            if (off >= len) break;
            uint8_t l = v[off++];
            dxl_print_dbg(" %04X/%u", t, l);
        }
    }
}

static void sc_tlv_dump(const uint8_t *p, uint16_t len, uint8_t depth) {
    uint16_t off = 0;
    char ind[24];

    if (depth > 8) {
        dxl_print_dbg("...TLV nesting too deep\n");
        return;
    }
    uint8_t ind_len = (uint8_t)(depth * 2);
    if (ind_len > sizeof(ind) - 1) {
        ind_len = sizeof(ind) - 1;
    }
    memset(ind, ' ', ind_len);
    ind[ind_len] = 0;

    while (off + 2 <= len) {
        uint16_t tag = p[off++];
        uint8_t tag_first = (uint8_t)tag;
        if ((tag & 0x1F) == 0x1F) {
            if (off >= len) break;
            tag = (uint16_t)((tag << 8) | p[off++]);
        }
        if (off >= len) break;
        uint16_t l = p[off++];
        if (l == 0x81) {
            if (off >= len) break;
            l = p[off++];
        } else if (l == 0x82) {
            if (off + 1 >= len) break;
            l = (uint16_t)((p[off] << 8) | p[off + 1]);
            off += 2;
        } else if (l >= 0x80) {
            break; // 0x80 是不定长；0x83+ 的多字节长度这里不支持
        }
        if (off + l > len) {
            dxl_print_dbg("%s...truncated TLV (need %u, have %u)\n", ind, l, len - off);
            l = (uint16_t)(len - off);
        }
        const char *name = sc_tag_name(tag);
        // constructed 位在多字节 tag 的第一个字节，不能看合并后的值。
        if (tag_first & 0x20) {
            dxl_print_dbg("%s%04X %s (%u bytes)\n", ind, tag, name, l);
            sc_tlv_dump(p + off, l, (uint8_t)(depth + 1));
        } else {
            dxl_print_dbg("%s%04X %s (%u):", ind, tag, name, l);
            for (uint16_t i = 0; i < l; ++i) {
                dxl_print_dbg(" %02x", p[off + i]);
            }
            sc_tlv_show_value(tag, p + off, l);
            dxl_print_dbg("\n");
        }
        off += l;
    }
}

// 在 FCI 里递归找 tag 0x88（PSE 目录文件所在的 SFI），找不到返回 0。
static uint8_t sc_tlv_find_sfi(const uint8_t *p, uint16_t len) {
    uint16_t off = 0;
    while (off + 2 <= len) {
        uint16_t tag = p[off++];
        uint8_t tag_first = (uint8_t)tag;
        if ((tag & 0x1F) == 0x1F) {
            if (off >= len) break;
            tag = (uint16_t)((tag << 8) | p[off++]);
        }
        if (off >= len) break;
        uint16_t l = p[off++];
        if (l == 0x81) {
            if (off >= len) break;
            l = p[off++];
        } else if (l == 0x82) {
            if (off + 1 >= len) break;
            l = (uint16_t)((p[off] << 8) | p[off + 1]);
            off += 2;
        } else if (l >= 0x80) {
            break;
        }
        if (off + l > len) {
            l = (uint16_t)(len - off);
        }
        if (tag == 0x88 && l >= 1) {
            return p[off];
        }
        if (tag_first & 0x20) {
            uint8_t sfi = sc_tlv_find_sfi(p + off, l);
            if (sfi) {
                return sfi;
            }
        }
        off += l;
    }
    return 0;
}

// 在 TLV 里递归找 tag；找到就把值拷到 out（AID 这类值很短）。
static bool sc_tlv_find_tag(const uint8_t *p, uint16_t len, uint16_t want,
                            uint8_t *out, uint8_t *out_len, uint8_t out_max) {
    uint16_t off = 0;
    while (off + 2 <= len) {
        uint16_t tag = p[off++];
        uint8_t tag_first = (uint8_t)tag;
        if ((tag & 0x1F) == 0x1F) {
            if (off >= len) break;
            tag = (uint16_t)((tag << 8) | p[off++]);
        }
        if (off >= len) break;
        uint16_t l = p[off++];
        if (l == 0x81) {
            if (off >= len) break;
            l = p[off++];
        } else if (l == 0x82) {
            if (off + 1 >= len) break;
            l = (uint16_t)((p[off] << 8) | p[off + 1]);
            off += 2;
        } else if (l >= 0x80) {
            break;
        }
        if (off + l > len) {
            l = (uint16_t)(len - off);
        }
        if (tag == want) {
            uint8_t n = (l > out_max) ? out_max : (uint8_t)l;
            memcpy(out, p + off, n);
            *out_len = n;
            return true;
        }
        if ((tag_first & 0x20) && sc_tlv_find_tag(p + off, l, want, out, out_len, out_max)) {
            return true;
        }
        off += l;
    }
    return false;
}

// PDOL 里每项是 tag + length，返回所有 length 之和（即 GPO 要填的数据长度）。
static uint16_t sc_pdol_data_len(const uint8_t *pdol, uint16_t len) {
    uint16_t off = 0;
    uint16_t total = 0;
    while (off + 1 < len) {
        uint16_t tag = pdol[off++];
        if ((tag & 0x1F) == 0x1F) {
            if (off >= len) break;
            tag = (uint16_t)((tag << 8) | pdol[off++]);
        }
        if (off >= len) break;
        uint8_t l = pdol[off++];
        total += l;
    }
    return total;
}

// GET PROCESSING OPTIONS。PDOL 数据全部填 0，只用于读取公开记录。
static uint16_t sc_gpo(const uint8_t *fci, uint16_t fci_len,
                       uint8_t *out, uint16_t out_max, uint16_t *out_len) {
    uint8_t pdol[128];
    uint8_t pdol_len = 0;
    uint8_t data[2 + 128];
    uint16_t need = 0;

    sc_tlv_find_tag(fci, fci_len, 0x9F38, pdol, &pdol_len, sizeof(pdol));
    if (pdol_len) {
        need = sc_pdol_data_len(pdol, pdol_len);
        // 0x83 的长度保持短格式，超过 127 的 PDOL 只填前 127 字节。
        if (need > 0x7F) {
            need = 0x7F;
        }
    }
    data[0] = 0x83;
    data[1] = (uint8_t)need;
    memset(data + 2, 0, need);
    return sc_t0_apdu(0x80, 0xA8, 0x00, 0x00, data, (uint8_t)(2 + need), 0,
                      out, out_max, out_len);
}

// 只读：按 SFI 逐条读记录，遇到 6A83（记录不存在）就停。
// aid/aid_len 非空时，顺便把记录里第一个 0x4F（ADF name）带出来。
static uint8_t sc_read_sfi_records_ex(uint8_t sfi, uint8_t max_rec,
                                      uint8_t *aid, uint8_t *aid_len) {
    uint8_t buf[260];
    uint8_t found = 0;

    for (uint8_t rec = 1; rec <= max_rec; ++rec) {
        uint16_t len = 0;
        uint16_t sw = sc_t0_apdu(0x00, 0xB2, rec, (uint8_t)((sfi << 3) | 0x04),
                                 NULL, 0, 0, buf, sizeof(buf), &len);
        if (sw == 0x6A83) {
            break; // 没有更多记录
        }
        if (sw == 0x6A82) {
            dxl_print_dbg("SFI %u: file not found\n", sfi);
            break;
        }
        if (sw == 0xFFFF) {
            dxl_print_dbg("SFI %u record %u: timeout\n", sfi, rec);
            break;
        }
        if (sw != 0x9000) {
            dxl_print_dbg("SFI %u record %u: SW=%04x\n", sfi, rec, sw);
            break;
        }
        if (len == 0) {
            dxl_print_dbg("SFI %u record %u: empty\n", sfi, rec);
            continue;
        }
        dxl_print_dbg("SFI %u record %u (%u bytes):\n", sfi, rec, len);
        sc_tlv_dump(buf, len, 2);
        if (aid && aid_len && *aid_len == 0) {
            sc_tlv_find_tag(buf, len, 0x4F, aid, aid_len, 16);
        }
        ++found;
    }
    return found;
}

static uint8_t sc_read_sfi_records(uint8_t sfi, uint8_t max_rec) {
    return sc_read_sfi_records_ex(sfi, max_rec, NULL, NULL);
}

#endif

void test_smartcard_module(void) {
    // 初始化I2C，等下需要拿来做状态切换
    StartTicks();
    I2C_init(true);
    uint8_t smartcard_addr = 0x52;

    SpinDelay(1000);
    usb_enable(); // 要初始化USB口，printf调试大法好
    uint8_t atr[50];
    uint8_t atr_len = 0;

    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
#if !SMARTCARD_SYNC_TEST
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
#endif

    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
#if SMARTCARD_SYNC_TEST
    uint8_t data[20];
    // FM4442 has no ISO7816 ATR. Read its 32-bit synchronous reset response.
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_3;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_bits_reset(GPIOA, GPIO_PINS_3);

    // I/O 读的时候用输入 + 上拉（和原来能正常读到 ATR 的配置一致）；
    // 发命令时在测试流程里临时切成开漏输出，发完再切回这里。
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_2;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOA, &gpio_init_struct);
#else
    // PA3: TMR2_CH4 -> TDA8035 CLKIN；TDA8035 再分频得到卡片时钟。
    // PA3 是时钟脚，不能挪作 USART2 RX/CK。
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = SC_CLK_PIN;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE3, GPIO_MUX_1);

    // PA2: USART2_TX，开漏。SmartCard 模式用这一根线完成收发。
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP; // ISO7816 I/O 需要上拉
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = SC_IO_PIN;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_7);

    // ISO7816 默认 F/D=372：3.6MHz / 372 ~= 9677 baud。
    sc_usart_init(SC_DEFAULT_BAUD);

    tmr_internal_clock_set(TMR2);
    tmr_reset(TMR2);
    tmr_base_init(TMR2, 39, 0); // 288 MHz / 40 = 7.2 MHz at TDA8035 CLKIN
    tmr_output_config_type tmr_output_struct;
    tmr_output_default_para_init(&tmr_output_struct);
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_4, &tmr_output_struct);
    tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, 20);
    tmr_counter_enable(TMR2, TRUE);
#endif

    while (1) {
        while (is_btn_pressed()) { } // 等待松开按钮
        dxl_print_dbg("按下按钮开始测试 智能卡模块 > \r\n");
        while (!is_btn_pressed()) { } // 等待按下按钮
        while (is_btn_pressed()) { } // 等待松开按钮

        // 设置供电电压（写周期不稳时可以把 1 改成 2 切到 5v）
        uint8_t voltage = 1; // 0 表示切换到 1.8v，1 表示切换到3v，2 表示切换到5v
        bool ret = I2C_BufferWrite(&voltage, 1, 0x01, (uint8_t)(smartcard_addr << 1));
        dxl_print_dbg("Set smartcard voltage to %s, Write %s\n",
            voltage == 0 ? "1.8v" : (voltage == 1 ? "3v" : "5v"),
            ret ? "success" : "failed");

#if SMARTCARD_SYNC_TEST

        // ---------------- FM4442 同步卡测试 ----------------
        // 复位/ATR 部分保持原来已经验证可用的写法，一个字都没动。
        // 命令部分按 SLE4432/4442 手册 2.2~2.5 节实现：
        //   1. 每字节 LSB 先发，I/O 在 CLK 低电平期间改变，卡片在 CLK 上升沿采样
        //   2. 命令 = START(CLK 高时 I/O 下降) + 3 字节 + 1 个附加时钟 + STOP(CLK 高时 I/O 上升)
        //   3. 读命令后必须补足 m 个时钟，否则卡片停在 outgoing data 模式，后面的 START 会被丢弃
        //   4. 写命令进入 processing 模式：卡片先把 I/O 拉低，处理完释放成高阻 Z，据此判断结束
        //   5. 改数据前必须先过 PSC 认证，否则 38H 写命令被静默忽略（这是之前读回 0xFF 的根因）
        //   6. CLK 7~50kHz，高/低电平各 >= 9us，这里用 12us（约 41.7kHz）
        uint8_t  fm_cmd[13][3];  // 命令 3 字节
        uint8_t  fm_kind[13];    // 0 = processing（写），1 = 读
        uint8_t  fm_nb[13];      // 读命令采样字节数
        uint16_t fm_pad[13];     // 读命令总共需要的时钟数 m
        uint8_t  fm_exp[13];     // 期望值，0xFF 表示不比较
        uint8_t  fm_res[13][4];  // 读回结果
        uint8_t  fm_n;           // 本遍事务条数
        uint8_t  fm_ec;          // 错误计数器
        uint8_t  fm_old = 0xFF;  // 主存储区 [0x20] 原值
        uint8_t  fm_pat[3];
        bool     fm_low_seen;
        bool     fm_done;
        uint16_t fm_k;

        // 命令帧时钟数：手册规定是 24bit + 1 个附加时钟后接 STOP（共 25 个时钟）。
        // 如果某批卡片实测写不进去，把这里改成 true，会在 STOP 前再补一个时钟（共 26 个）。
        const bool fm_extra_clk = false;

        // Select fXTAL (no divider): XTAL1 directly clocks the FM4442.
        data[0] = 3;
        ret = I2C_BufferWrite(data, 1, 0x05, smartcard_addr << 1);
        dxl_print_dbg("Set smartcard clock to %s, Write %s\n",
            data[0] == 0 ? "fXTAL/8" : (data[0] == 1 ? "fXTAL/4" : (data[0] == 2 ? "fXTAL/2" : "fXTAL")),
            ret ? "success" : "failed");

        // Hold reset low while activating the card, with the external clock low.
        data[0] = 0;
        ret = I2C_BufferWrite(data, 1, 0x04, smartcard_addr << 1);
        data[0] = 1;
        ret = I2C_BufferWrite(data, 1, 0x02, smartcard_addr << 1);
        dxl_print_dbg("Activate synchronous card: %s\n", ret ? "success" : "failed");
        SpinDelay(10);

        for (uint8_t bit = 0; bit < 15; ++bit) {
            gpio_bits_set(GPIOA, GPIO_PINS_3);
            SpinDelayUsPrecision(12);
            gpio_bits_reset(GPIOA, GPIO_PINS_3);
            SpinDelayUsPrecision(12);
        }

        // while (1) {} // TODO 测试

        // A rising reset edge initializes the FM4442 address and exposes DO0.
        data[0] = 1;
        ret = I2C_BufferWrite(data, 1, 0x04, smartcard_addr << 1);
        SpinDelayUsPrecision(12);

        // gpio_bits_set(GPIOA, GPIO_PINS_3);
        // SpinDelayUsPrecision(12);
        // gpio_bits_reset(GPIOA, GPIO_PINS_3);
        // SpinDelayUsPrecision(12);

        gpio_bits_set(GPIOA, GPIO_PINS_3);
        SpinDelayUsPrecision(12);
        gpio_bits_reset(GPIOA, GPIO_PINS_3);
        SpinDelayUsPrecision(12);

        data[0] = 0;
        ret = I2C_BufferWrite(data, 1, 0x04, smartcard_addr << 1);
        SpinDelayUsPrecision(12);

        // FM4442 synchronous reset response is 32 bits, LSB first, MSB last.

        memset(atr, 0, sizeof(atr));
        atr_len = 0;
        for (uint8_t bit = 0; bit < 32; ++bit) {
            if (GpioInputStatus(GPIOA, GPIO_PINS_2)) {
                atr[bit / 8] |= (uint8_t)(1u << (bit % 8));
            }
            if ((bit & 7u) == 7u) {
                ++atr_len;
            }
            gpio_bits_set(GPIOA, GPIO_PINS_3);
            SpinDelayUsPrecision(12);
            gpio_bits_reset(GPIOA, GPIO_PINS_3);
            SpinDelayUsPrecision(12);
        }

        dxl_print_dbg("FM4442 synchronous reset response:");
        for (uint8_t i = 0; i < atr_len; ++i) {
            dxl_print_dbg(" %02x", atr[i]);
        }
        dxl_print_dbg("\n");

        // 手册：第 33 个时钟把 I/O 释放成高阻 Z，ATR 结束、卡片进入命令模式
        gpio_bits_set(GPIOA, GPIO_PINS_3);
        SpinDelayUsPrecision(12);
        gpio_bits_reset(GPIOA, GPIO_PINS_3);
        SpinDelayUsPrecision(12);

        // ---- 第一条命令：READ SECURITY MEMORY (31H)，先拿到错误计数器 ----
        // 命令发送：I/O 切成开漏输出，写 1 即释放总线（板上有上拉）
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
        gpio_init_struct.gpio_pull = GPIO_PULL_UP;
        gpio_init_struct.gpio_pins = GPIO_PINS_2;
        gpio_init(GPIOA, &gpio_init_struct);
        gpio_bits_set(GPIOA, GPIO_PINS_2);
        gpio_bits_reset(GPIOA, GPIO_PINS_3);
        SpinDelayUsPrecision(12);                   // t1: START 前 I/O 高电平 >= 10us
        gpio_bits_set(GPIOA, GPIO_PINS_3);
        SpinDelayUsPrecision(12);
        gpio_bits_reset(GPIOA, GPIO_PINS_2);        // START: CLK 高时 I/O 下降
        SpinDelayUsPrecision(12);                   // t3
        gpio_bits_reset(GPIOA, GPIO_PINS_3);
        SpinDelayUsPrecision(12);

        fm_cmd[0][0] = 0x31;
        fm_cmd[0][1] = 0x00;
        fm_cmd[0][2] = 0x00;
        for (uint8_t bi = 0; bi < 3; ++bi) {
            for (uint8_t bit = 0; bit < 8; ++bit) {
                if ((fm_cmd[0][bi] >> bit) & 1u) {
                    gpio_bits_set(GPIOA, GPIO_PINS_2);
                } else {
                    gpio_bits_reset(GPIOA, GPIO_PINS_2);
                }
                SpinDelayUsPrecision(12);           // t4: 建立时间
                gpio_bits_set(GPIOA, GPIO_PINS_3);
                SpinDelayUsPrecision(12);
                gpio_bits_reset(GPIOA, GPIO_PINS_3);
                SpinDelayUsPrecision(12);
            }
        }

        if (fm_extra_clk) {                         // 可选：再多一个时钟
            gpio_bits_set(GPIOA, GPIO_PINS_3);
            SpinDelayUsPrecision(12);
            gpio_bits_reset(GPIOA, GPIO_PINS_3);
            SpinDelayUsPrecision(12);
        }
        gpio_bits_reset(GPIOA, GPIO_PINS_2);        // CLK 低时先拉低 I/O
        SpinDelayUsPrecision(12);                   // t5
        gpio_bits_set(GPIOA, GPIO_PINS_3);          // 附加时钟（第 25 个）
        SpinDelayUsPrecision(12);
        gpio_bits_set(GPIOA, GPIO_PINS_2);          // STOP: CLK 高时 I/O 上升
        SpinDelayUsPrecision(12);                   // t6
        gpio_bits_reset(GPIOA, GPIO_PINS_3);
        SpinDelayUsPrecision(12);

        // 接收：I/O 切回输入（原来 ATR 就是这么读的）
        gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_pull = GPIO_PULL_UP;
        gpio_init_struct.gpio_pins = GPIO_PINS_2;
        gpio_init(GPIOA, &gpio_init_struct);
        SpinDelayUsPrecision(12);

        memset(fm_res[0], 0, sizeof(fm_res[0]));
        for (fm_k = 0; fm_k < 33; ++fm_k) {         // 32 个数据时钟 + 1 个归位时钟
            gpio_bits_reset(GPIOA, GPIO_PINS_3);
            SpinDelayUsPrecision(12);               // t7: 下降沿后数据有效
            if (fm_k < 32 && GpioInputStatus(GPIOA, GPIO_PINS_2)) {
                fm_res[0][fm_k >> 3] |= (uint8_t)(1u << (fm_k & 7));
            }
            gpio_bits_set(GPIOA, GPIO_PINS_3);
            SpinDelayUsPrecision(12);
        }
        fm_ec = fm_res[0][0] & 0x07;
        dxl_print_dbg("Security memory: EC=0x%02x ref=%02x %02x %02x (未认证时参考字节读回 00)\n",
                      fm_ec, fm_res[0][1], fm_res[0][2], fm_res[0][3]);

        // ---- 事务表：第一遍做 PSC 认证 + 复查 + 读原值，第二遍写数据并回读 ----
        for (uint8_t pass = 0; pass < 2; ++pass) {
            fm_n = 0;

            if (pass == 0) {
                // 读保护存储区（34H）：32 个保护位，1 表示对应字节已锁定、不可再改写
                fm_cmd[fm_n][0] = 0x34; fm_cmd[fm_n][1] = 0x00; fm_cmd[fm_n][2] = 0x00;
                fm_kind[fm_n] = 1; fm_nb[fm_n] = 4; fm_pad[fm_n] = 33; fm_exp[fm_n] = 0xFF; ++fm_n;

                if (fm_ec >= 0x04) {
                    // PSC 认证 5 步：消耗一个错误计数位 -> 比较 3 个参考字节 -> 清错误计数器
                    fm_cmd[fm_n][0] = 0x39; fm_cmd[fm_n][1] = 0x00; fm_cmd[fm_n][2] = (uint8_t)(fm_ec & 0x06);
                    fm_kind[fm_n] = 0; fm_exp[fm_n] = 0xFF; ++fm_n;
                    fm_cmd[fm_n][0] = 0x33; fm_cmd[fm_n][1] = 0x01; fm_cmd[fm_n][2] = 0xFF;
                    fm_kind[fm_n] = 0; fm_exp[fm_n] = 0xFF; ++fm_n;
                    fm_cmd[fm_n][0] = 0x33; fm_cmd[fm_n][1] = 0x02; fm_cmd[fm_n][2] = 0xFF;
                    fm_kind[fm_n] = 0; fm_exp[fm_n] = 0xFF; ++fm_n;
                    fm_cmd[fm_n][0] = 0x33; fm_cmd[fm_n][1] = 0x03; fm_cmd[fm_n][2] = 0xFF;
                    fm_kind[fm_n] = 0; fm_exp[fm_n] = 0xFF; ++fm_n;
                    fm_cmd[fm_n][0] = 0x39; fm_cmd[fm_n][1] = 0x00; fm_cmd[fm_n][2] = 0xFF;
                    fm_kind[fm_n] = 0; fm_exp[fm_n] = 0xFF; ++fm_n;
                } else {
                    dxl_print_dbg("PSC verify skipped: error counter nearly exhausted (EC=0x%02x)\n", fm_ec);
                }

                // 复查安全存储区：EC 回到 0x07 说明认证成功
                fm_cmd[fm_n][0] = 0x31; fm_cmd[fm_n][1] = 0x00; fm_cmd[fm_n][2] = 0x00;
                fm_kind[fm_n] = 1; fm_nb[fm_n] = 4; fm_pad[fm_n] = 33; fm_exp[fm_n] = 0xFF; ++fm_n;

                // 读主存储区 [0x20] 原值（手册要求改数据前必须先读），m=(256-32)*8+1
                fm_cmd[fm_n][0] = 0x30; fm_cmd[fm_n][1] = 0x20; fm_cmd[fm_n][2] = 0x00;
                fm_kind[fm_n] = 1; fm_nb[fm_n] = 1; fm_pad[fm_n] = 1793; fm_exp[fm_n] = 0xFF; ++fm_n;
            } else {
                // 写 0x5A / 0xA5 / 原值，每次写完立即回读校验
                fm_pat[0] = 0x5A;
                fm_pat[1] = 0xA5;
                fm_pat[2] = fm_old;
                for (uint8_t p = 0; p < 3; ++p) {
                    fm_cmd[fm_n][0] = 0x38; fm_cmd[fm_n][1] = 0x20; fm_cmd[fm_n][2] = fm_pat[p];
                    fm_kind[fm_n] = 0; fm_exp[fm_n] = 0xFF; ++fm_n;

                    fm_cmd[fm_n][0] = 0x30; fm_cmd[fm_n][1] = 0x20; fm_cmd[fm_n][2] = 0x00;
                    fm_kind[fm_n] = 1; fm_nb[fm_n] = 1; fm_pad[fm_n] = 1793; fm_exp[fm_n] = fm_pat[p]; ++fm_n;
                }
            }

            for (uint8_t i = 0; i < fm_n; ++i) {
                // ---- 发送 3 字节命令 ----
                gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
                gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
                gpio_init_struct.gpio_pull = GPIO_PULL_UP;
                gpio_init_struct.gpio_pins = GPIO_PINS_2;
                gpio_init(GPIOA, &gpio_init_struct);
                gpio_bits_set(GPIOA, GPIO_PINS_2);
                gpio_bits_reset(GPIOA, GPIO_PINS_3);
                SpinDelayUsPrecision(12);               // t1
                gpio_bits_set(GPIOA, GPIO_PINS_3);
                SpinDelayUsPrecision(12);
                gpio_bits_reset(GPIOA, GPIO_PINS_2);    // START
                SpinDelayUsPrecision(12);               // t3
                gpio_bits_reset(GPIOA, GPIO_PINS_3);
                SpinDelayUsPrecision(12);

                for (uint8_t bi = 0; bi < 3; ++bi) {
                    for (uint8_t bit = 0; bit < 8; ++bit) {
                        if ((fm_cmd[i][bi] >> bit) & 1u) {
                            gpio_bits_set(GPIOA, GPIO_PINS_2);
                        } else {
                            gpio_bits_reset(GPIOA, GPIO_PINS_2);
                        }
                        SpinDelayUsPrecision(12);       // t4
                        gpio_bits_set(GPIOA, GPIO_PINS_3);
                        SpinDelayUsPrecision(12);
                        gpio_bits_reset(GPIOA, GPIO_PINS_3);
                        SpinDelayUsPrecision(12);
                    }
                }

                if (fm_extra_clk) {                     // 可选：再多一个时钟
                    gpio_bits_set(GPIOA, GPIO_PINS_3);
                    SpinDelayUsPrecision(12);
                    gpio_bits_reset(GPIOA, GPIO_PINS_3);
                    SpinDelayUsPrecision(12);
                }
                gpio_bits_reset(GPIOA, GPIO_PINS_2);    // CLK 低时先拉低 I/O
                SpinDelayUsPrecision(12);               // t5
                gpio_bits_set(GPIOA, GPIO_PINS_3);      // 附加时钟（第 25 个）
                SpinDelayUsPrecision(12);
                gpio_bits_set(GPIOA, GPIO_PINS_2);      // STOP：CLK 高时 I/O 上升
                SpinDelayUsPrecision(12);               // t6
                gpio_bits_reset(GPIOA, GPIO_PINS_3);
                SpinDelayUsPrecision(12);

                // ---- 切回输入接收应答 ----
                gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
                gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
                gpio_init_struct.gpio_pull = GPIO_PULL_UP;
                gpio_init_struct.gpio_pins = GPIO_PINS_2;
                gpio_init(GPIOA, &gpio_init_struct);
                SpinDelayUsPrecision(12);

                memset(fm_res[i], 0, sizeof(fm_res[i]));
                fm_low_seen = false;
                fm_done = false;

                if (fm_kind[i] == 0) {
                    // processing：卡片先把 I/O 拉低，处理完释放成高阻 Z
                    for (fm_k = 0; fm_k < 1200; ++fm_k) {
                        gpio_bits_set(GPIOA, GPIO_PINS_3);
                        SpinDelayUsPrecision(12);
                        if (!GpioInputStatus(GPIOA, GPIO_PINS_2)) {
                            fm_low_seen = true;
                        }
                        gpio_bits_reset(GPIOA, GPIO_PINS_3);
                        SpinDelayUsPrecision(12);
                        if (fm_low_seen && GpioInputStatus(GPIOA, GPIO_PINS_2)) {
                            fm_done = true;
                            break;
                        }
                    }
                    dxl_print_dbg("[%u] cmd %02x %02x %02x -> processing %s "
                                  "(I/O %s, %u clocks)\n",
                                  i, fm_cmd[i][0], fm_cmd[i][1], fm_cmd[i][2],
                                  fm_done ? "done" : "TIMEOUT",
                                  fm_low_seen ? "pulled low" : "stayed high - command REJECTED",
                                  (unsigned)fm_k);
                } else {
                    // outgoing data：第 k 个下降沿后第 k 个 bit 有效，采够后继续补时钟把卡片送回命令模式
                    for (fm_k = 0; fm_k < fm_pad[i]; ++fm_k) {
                        gpio_bits_reset(GPIOA, GPIO_PINS_3);
                        SpinDelayUsPrecision(12);       // t7
                        if (fm_k < (uint16_t)fm_nb[i] * 8 && GpioInputStatus(GPIOA, GPIO_PINS_2)) {
                            fm_res[i][fm_k >> 3] |= (uint8_t)(1u << (fm_k & 7));
                        }
                        gpio_bits_set(GPIOA, GPIO_PINS_3);
                        SpinDelayUsPrecision(12);
                    }

                    if (fm_cmd[i][0] == 0x31) {
                        dxl_print_dbg("[%u] cmd %02x %02x %02x -> EC=0x%02x ref=%02x %02x %02x -> %s\n",
                                      i, fm_cmd[i][0], fm_cmd[i][1], fm_cmd[i][2],
                                      fm_res[i][0] & 0x07, fm_res[i][1], fm_res[i][2], fm_res[i][3],
                                      ((fm_res[i][0] & 0x07) == 0x07) ? "PSC verified" : "PSC NOT verified");
                    } else if (fm_cmd[i][0] == 0x34) {
                        dxl_print_dbg("[%u] protection memory: %02x %02x %02x %02x "
                                      "(bit=1 表示该字节已锁定)\n",
                                      i, fm_res[i][0], fm_res[i][1], fm_res[i][2], fm_res[i][3]);
                    } else if (pass == 0) {
                        // 第一遍：只记录原值，不做比较
                        fm_old = fm_res[i][0];
                        dxl_print_dbg("[%u] cmd %02x %02x %02x -> original [0x20]=0x%02x\n",
                                      i, fm_cmd[i][0], fm_cmd[i][1], fm_cmd[i][2], fm_res[i][0]);
                    } else {
                        dxl_print_dbg("[%u] cmd %02x %02x %02x -> write 0x%02x, readback 0x%02x -> %s\n",
                                      i, fm_cmd[i][0], fm_cmd[i][1], fm_cmd[i][2],
                                      fm_exp[i], fm_res[i][0],
                                      (fm_res[i][0] == fm_exp[i]) ? "PASS" : "FAIL");
                    }
                }
            }
        }

        data[0] = 0;
        ret = I2C_BufferWrite(data, 1, 0x02, smartcard_addr << 1);
        dxl_print_dbg("Deactivate synchronous card: %s\n", ret ? "success" : "failed");
#else
        // ---------------- ISO7816-3 异步卡测试（USART SmartCard 模式） ----------------
        const uint8_t sc_addr = smartcard_addr;

        // 上一轮可能协商到了高速，先回到默认的 3.6MHz / 9677bps。
        sc_set_baud(SC_DEFAULT_BAUD);

        // 设置时钟频率，0=fXTAL/8，1=fXTAL/4，2=fXTAL/2，3=fXTAL
        ret = sc_reg_write(sc_addr, 0x05, 2); // 3.6MHz -> 9677bps @ F/D=372
        dxl_print_dbg("Set smartcard clock to fXTAL/2 (3.6MHz): %s\n", ret ? "success" : "failed");

        // RST 保持低电平，先给卡片上电和时钟。
        ret = sc_reg_write(sc_addr, 0x04, 0);
        ret = sc_reg_write(sc_addr, 0x02, 1);
        dxl_print_dbg("Activate smartcard: %s\n", ret ? "success" : "failed");
        SpinDelay(10);

        // ---- 硬件 GT 自检：卡片在复位状态，I/O 不会被卡片驱动 ----
        sc_set_guard_time(0);
        uint32_t gt0_ticks = sc_measure_char_ticks();
        sc_set_guard_time(2);
        uint32_t gt2_ticks = sc_measure_char_ticks();
        if (gt0_ticks && gt2_ticks && gt2_ticks > gt0_ticks) {
            uint32_t gt_delta = gt2_ticks - gt0_ticks;
            dxl_print_dbg("Guard time: GT=0 %u ticks (%u us), GT=2 %u ticks (%u us), delta=%u ticks (%u us)\n",
                          gt0_ticks, SC_TICK_US(gt0_ticks),
                          gt2_ticks, SC_TICK_US(gt2_ticks),
                          gt_delta, SC_TICK_US(gt_delta));
            dxl_print_dbg("  2 etu @ 9677bps = 207us -> hardware GT %s\n",
                          (gt_delta > 150) ? "active" : "NOT visible");
        } else {
            dxl_print_dbg("Guard time: measurement failed (GT=0 %u, GT=2 %u)\n", gt0_ticks, gt2_ticks);
        }

        // 释放复位，接收 ATR。先清掉 GT 测量留下的回灌字节。
        sc_rx_flush();
        ret = sc_reg_write(sc_addr, 0x04, 1);
        memset(atr, 0, sizeof(atr));
        atr_len = sc_read_atr(atr, sizeof(atr));
        dxl_print_dbg("ATR received (%u bytes):", atr_len);
        for (uint8_t i = 0; i < atr_len; ++i) {
            dxl_print_dbg(" %02x", atr[i]);
        }
        dxl_print_dbg("\n");

        if (atr_len < 2) {
            dxl_print_dbg("No valid ATR; check card, clock and I/O.\n");
            sc_reg_write(sc_addr, 0x02, 0);
            continue;
        }

        sc_atr_info_t atr_info;
        sc_atr_parse(atr, atr_len, &atr_info);
        sc_rx_flush();
        // ATR 结束后留 400 etu（约 41ms）给卡片切换回接收状态。
        SpinDelay(50);

        // TC1 是卡片要求的额外保护时间；硬件 GT 至少给 2 etu。
        uint16_t gt = 2;
        if (atr_info.has_tc1 && atr_info.tc1 != 0xFF) {
            gt = (uint16_t)atr_info.tc1 + 2;
            if (gt > 254) {
                gt = 254;
            }
        }
        sc_set_guard_time((uint8_t)gt);
        if (atr_info.has_tc1) {
            dxl_print_dbg("Hardware guard time: %u etu (TC1=0x%02x)\n", gt, atr_info.tc1);
        } else {
            dxl_print_dbg("Hardware guard time: %u etu (TC1 absent)\n", gt);
        }

        if (atr_info.proto != 0) {
            dxl_print_dbg("Card does not run T=0 first; skip T=0 tests.\n");
            sc_reg_write(sc_addr, 0x02, 0);
            continue;
        }

        // PPS 必须紧跟在 ATR 之后、第一条 APDU 之前。
        // 只有 ATR 带 TA1（卡片声明支持 Fi/Di）时才发 PPS：没有 TA1 的卡会把
        // PPSS=0xFF 当成不完整的命令头，后续字节全部错位。
        uint8_t pps_ta1 = 0;
        uint32_t sc_rate = SC_DEFAULT_BAUD;
        bool pps_attempted = false;
        bool pps_ok = false;
        if (atr_info.has_ta1) {
            dxl_print_dbg("PPS: request Fi=512/Di=8 (TA1=0x94)...\n");
            pps_attempted = true;
            pps_ok = sc_pps(0x94, &pps_ta1);
        } else {
            dxl_print_dbg("PPS skipped: ATR has no TA1 (no Fi/Di negotiation)\n");
        }

        if (pps_ok) {
            uint32_t baud_fast = sc_baud_from_ta1(pps_ta1, SC_CLOCK_FAST);
            uint32_t baud_36 = sc_baud_from_ta1(pps_ta1, SC_CLOCK_DEFAULT);
            if (baud_fast == 0) {
                dxl_print_dbg("PPS: TA1=0x%02x has RFU Fi/Di; stay at 9677bps\n", pps_ta1);
            } else {
                // PPS 协商的是 F/D；要到 ~115200 还需要把卡片时钟从 3.6MHz 切到 7.2MHz。
                sc_reg_write(sc_addr, 0x05, 3);
                sc_set_baud(baud_fast);
                sc_rate = baud_fast;
                dxl_print_dbg("PPS accepted: TA1=0x%02x Fi=%u Di=%u, clock 7.2MHz, %u -> %u bps (3.6MHz would be %u)\n",
                              pps_ta1,
                              sc_fi_tab[(pps_ta1 >> 4) & 0x0F], sc_di_tab[pps_ta1 & 0x0F],
                              SC_DEFAULT_BAUD, baud_fast, baud_36);
                uint16_t fmax = sc_fmax_khz[(pps_ta1 >> 4) & 0x0F];
                if (fmax && fmax < 7200) {
                    dxl_print_dbg("  note: 7.2MHz exceeds the card's advertised f_max %u.%uMHz\n",
                                  fmax / 1000, (fmax % 1000) / 100);
                }

                // 用 READ RECORD 验证高速链路：任何 SW（包括 6A82）都说明
                // 卡片收到了命令并回了应答；只有超时才说明速率不对。
                uint8_t  tmp[8];
                uint16_t tmp_len = 0;
                uint16_t sw = sc_t0_apdu(0x00, 0xB2, 0x01, 0x0C, NULL, 0, 0,
                                         tmp, sizeof(tmp), &tmp_len);
                if (sw == 0xFFFF) {
                    dxl_print_dbg("High-speed link check failed (no response); fall back to 9677bps\n");
                    sc_reg_write(sc_addr, 0x05, 2);
                    sc_set_baud(SC_DEFAULT_BAUD);
                    sc_rate = SC_DEFAULT_BAUD;
                    // 复位卡片，清掉 PPS 协商结果。
                    sc_rx_flush();
                    sc_reg_write(sc_addr, 0x04, 0);
                    SpinDelay(5);
                    sc_reg_write(sc_addr, 0x04, 1);
                    atr_len = sc_read_atr(atr, sizeof(atr));
                    dxl_print_dbg("Fallback ATR (%u bytes):", atr_len);
                    for (uint8_t i = 0; i < atr_len; ++i) {
                        dxl_print_dbg(" %02x", atr[i]);
                    }
                    dxl_print_dbg("\n");
                    sc_rx_flush();
                } else {
                    dxl_print_dbg("High-speed link check: SW=%04x (link OK)\n", sw);
                }
            }
        }

        // PPS 失败（尤其是超时）后卡片可能停在半帧状态，复位并重读 ATR 再继续。
        if (pps_attempted && !pps_ok) {
            dxl_print_dbg("PPS failed; reset card and re-read ATR\n");
            sc_rx_flush();
            sc_reg_write(sc_addr, 0x04, 0);
            SpinDelay(5);
            sc_reg_write(sc_addr, 0x04, 1);
            atr_len = sc_read_atr(atr, sizeof(atr));
            dxl_print_dbg("Re-ATR (%u bytes):", atr_len);
            for (uint8_t i = 0; i < atr_len; ++i) {
                dxl_print_dbg(" %02x", atr[i]);
            }
            dxl_print_dbg("\n");
            sc_rx_flush();
            SpinDelay(10);
        }

        // ---- 只读公开数据：PSE -> AID -> SFI 1~3 ----
        {
            static const uint8_t mf_id[] = {0x3F, 0x00};
            static const uint8_t aid_pse[] = {'1', 'P', 'A', 'Y', '.', 'S', 'Y', 'S', '.', 'D', 'D', 'F', '0', '1'};
            uint8_t  apdu_rx[260];
            uint16_t apdu_len;
            uint16_t sw;
            bool pse_ok = false;

            SpinDelay(10); // ATR/PPS 之后给卡片一点处理时间

            dxl_print_dbg("--- SELECT PSE 1PAY.SYS.DDF01 ---\n");
            sw = sc_t0_apdu(0x00, 0xA4, 0x04, 0x00, aid_pse, 14, 0,
                            apdu_rx, sizeof(apdu_rx), &apdu_len);
            if (sw == 0xFFFF) {
                dxl_print_dbg("SELECT PSE: no response\n");
            } else if (sw == 0x9000) {
                pse_ok = true;
                dxl_print_dbg("SELECT PSE: SW=9000, FCI:\n");
                sc_tlv_dump(apdu_rx, apdu_len, 0);

                uint8_t dir_sfi = sc_tlv_find_sfi(apdu_rx, apdu_len);
                uint8_t aid[16] = {0};
                uint8_t aid_len = 0;
                if (dir_sfi) {
                    dxl_print_dbg("--- PSE directory SFI %u records ---\n", dir_sfi);
                    if (sc_read_sfi_records_ex(dir_sfi, 16, aid, &aid_len) == 0) {
                        dxl_print_dbg("PSE SFI %u: no readable records\n", dir_sfi);
                    }
                }

                if (aid_len) {
                    dxl_print_dbg("--- SELECT AID");
                    for (uint8_t i = 0; i < aid_len; ++i) {
                        dxl_print_dbg(" %02x", aid[i]);
                    }
                    dxl_print_dbg(" ---\n");
                    sw = sc_t0_apdu(0x00, 0xA4, 0x04, 0x00, aid, aid_len, 0,
                                    apdu_rx, sizeof(apdu_rx), &apdu_len);
                    if (sw == 0x9000) {
                        dxl_print_dbg("SELECT AID: SW=9000, FCI:\n");
                        sc_tlv_dump(apdu_rx, apdu_len, 0);

                        // EMV 流程：SELECT AID 之后先 GPO，再读记录。
                        uint8_t  gpo_rx[260];
                        uint16_t gpo_len = 0;
                        uint16_t gpo_sw = sc_gpo(apdu_rx, apdu_len, gpo_rx, sizeof(gpo_rx), &gpo_len);
                        if (gpo_sw == 0x9000) {
                            dxl_print_dbg("GPO: SW=9000:\n");
                            sc_tlv_dump(gpo_rx, gpo_len, 0);
                        } else {
                            if (gpo_sw == 0xFFFF) {
                                dxl_print_dbg("GPO: no response; re-select AID and continue with READ RECORD\n");
                            } else {
                                dxl_print_dbg("GPO: SW=%04x; re-select AID and continue with READ RECORD\n", gpo_sw);
                            }
                            // 失败的 GPO 可能让卡片停在事务状态，重选一次再读记录。
                            sc_t0_apdu(0x00, 0xA4, 0x04, 0x00, aid, aid_len, 0,
                                       apdu_rx, sizeof(apdu_rx), &apdu_len);
                        }

                        for (uint8_t sfi = 1; sfi <= 3; ++sfi) {
                            dxl_print_dbg("--- App SFI %u records ---\n", sfi);
                            if (sc_read_sfi_records(sfi, 16) == 0) {
                                dxl_print_dbg("App SFI %u: no readable records\n", sfi);
                            }
                        }
                    } else if (sw == 0xFFFF) {
                        dxl_print_dbg("SELECT AID: no response\n");
                    } else {
                        dxl_print_dbg("SELECT AID: SW=%04x\n", sw);
                    }
                } else {
                    dxl_print_dbg("PSE: no AID found in directory\n");
                }
            } else {
                dxl_print_dbg("SELECT PSE: SW=%04x\n", sw);
            }

            if (!pse_ok) {
                // 没有 PSE 的卡（社保卡等）直接在 MF 下读 SFI 1~3。
                dxl_print_dbg("--- SELECT MF 3F00 ---\n");
                sw = sc_t0_apdu(0x00, 0xA4, 0x00, 0x00, mf_id, 2, 0,
                                apdu_rx, sizeof(apdu_rx), &apdu_len);
                if (sw == 0xFFFF) {
                    dxl_print_dbg("SELECT MF: no response\n");
                } else {
                    dxl_print_dbg("SELECT MF: SW=%04x\n", sw);
                }
                for (uint8_t sfi = 1; sfi <= 3; ++sfi) {
                    dxl_print_dbg("--- SFI %u records ---\n", sfi);
                    if (sc_read_sfi_records(sfi, 16) == 0) {
                        dxl_print_dbg("SFI %u: no readable records\n", sfi);
                    }
                }
            }
        }

        // ---- 高速下再确认一次硬件 GT：先复位卡片，0x00 不会被当成 APDU ----
        if (sc_rate != SC_DEFAULT_BAUD) {
            sc_reg_write(sc_addr, 0x04, 0);
            SpinDelay(10);
            sc_set_guard_time(0);
            uint32_t fast_gt0 = sc_measure_char_ticks();
            sc_set_guard_time(2);
            uint32_t fast_gt2 = sc_measure_char_ticks();
            if (fast_gt0 && fast_gt2 && fast_gt2 > fast_gt0) {
                uint32_t fast_delta = fast_gt2 - fast_gt0;
                dxl_print_dbg("Fast GT @ %u bps: GT=0 %u, GT=2 %u, delta=%u ticks (%u us)\n",
                              sc_rate, fast_gt0, fast_gt2, fast_delta, SC_TICK_US(fast_delta));
                dxl_print_dbg("  expected 2 etu = %u us -> hardware GT %s\n",
                              2000000u / sc_rate, (fast_delta > 12) ? "active" : "NOT visible");
            } else {
                dxl_print_dbg("Fast GT @ %u bps: measurement failed\n", sc_rate);
            }
        }

        // 去激活（关闭卡片）
        ret = sc_reg_write(sc_addr, 0x02, 0);
        dxl_print_dbg("Deactivate smartcard: %s\n", ret ? "success" : "failed");
#endif
    }
}

// 覆盖 UnitTestMain 实现单元测试
void UnitTestMain(void);

void UnitTestMain(void) {
    // ------------------------------- 测试 等待第一次烧录之后，按钮松开  -------------------------------
    while (1) {
        if (!is_btn_pressed()) break;
        LED_A_ON();
        SpinDelay(200);
        LED_B_ON();
        SpinDelay(200);
        LED_C_ON();
        SpinDelay(200);
        LED_D_ON();
        SpinDelay(200);
    }

    // ------------------------------- 关闭所有的LED -------------------------------
    LEDsoff();

    // ------------------------------- 功耗测试 -------------------------------
    // test_max_power();

    // ------------------------------- 关闭FPGA输出 -------------------------------
    // 不然的话调试的时候也会很发热
    SpinDelay(500);
    FpgaWriteConfWord(FPGA_MAJOR_MODE_OFF);

    // ------------------------------- 测试 智能卡模块 -------------------------------
    test_smartcard_module();

    // ------------------------------- 测试 NVIC 系统复位 -------------------------------
    // test_nvic_reset();

    // ------------------------------- 测试 配置UART TX2为DBGIO输出 -------------------------------
    // test_config_uart_tx2_to_dbgio();

    // ------------------------------- 测试 蓝牙电池套件UART通信 -------------------------------
    // test_bwm_uart();

    // ------------------------------- 测试 VUSB供电检测 -------------------------------
    // test_vusb_check();

    // ------------------------------- 测试 24c02 -------------------------------
    // test_24c02();

    // ------------------------------- 测试 F0 通信 -------------------------------
    // test_f0_com_by_usb_cep();

    // ------------------------------- 测试 四个灯 -------------------------------
    // test_4leds();

    // ------------------------------- 测试 库仑计和充电器的中断事件 -------------------------------
    // test_coulometer_charger_int();

    // ------------------------------- 测试 充电器 -------------------------------
    // test_bat_charger();

    // ------------------------------- 测试 充电器（测完就进主循环） -------------------------------
    test_bat_charger_only_settings();

    // ------------------------------- 测试 库仑计 -------------------------------
    // test_bat_coulometer();

    // ------------------------------- 测试 复合多频天线切换IO -------------------------------
    // test_i2c_ant_software();

    // ------------------------------- 测试按钮关机 -------------------------------
    // test_power_off_by_btn();

    // ------------------------------- 测试蜂鸣器 -------------------------------
    // usb_enable();
    // SpinDelay(1000);
    // dxl_print_dbg("SystemStart\n");
    // test_beep();

    // ------------------------------- 测试模拟按钮长按退出isp  -------------------------------
    // test_isp_exit();

    // ------------------------------- 测试 USB互联口上的单线串口  -------------------------------
    // test_usb_xlink_1line_uart();

    // ------------------------------- 测试 USB互联口上的SPI  -------------------------------
    // test_usb_xlink_spi();

    // ------------------------------- 测试 软件I2C  -------------------------------
    // test_init_debug_pin();
    test_i2c_rgb_simple();

    // ------------------------------- 测试 硬件I2C配置CC控制器  -------------------------------
    // test_i2c_cc();

    // ------------------------------- 测试 硬件I2C配置RGB灯的状态  -------------------------------
    // test_i2c_rgb();

    // ------------------------------- 测试 USB主从机切换状态  -------------------------------
    // test_usb_id_pin();

    // ------------------------------- 测试 ADC采样RSSI电压值  -------------------------------
    // FpgaSetup24MHzClk();
    // FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER); // 开场
    // AdcSetupRssiChannel(ADC_RSSI_CH_HF);
    // volatile uint16_t adc_val = 0;
    // volatile uint16_t adc_vref = g_adc_vref_value;
    // while (1) {
    //     if (AdcRssiDataReady(ADC_RSSI_CH_HF)) {
    //         adc_val = AdcRssiDataRead(ADC_RSSI_CH_HF);
    //     }
    //     (void)adc_val;
    //     (void)adc_vref;
    //     // printf("vref_value = %f V\r\n", ((double)1.2 * 4095) / adc1_ordinary_value);
    //     AdcRssiConversionStart();
    //
    //     SpinDelay(1);
    //
    //     adc_val = AdcRssiAvg(ADC_RSSI_CH_HF);
    //
    //     SpinDelay(1);
    //
    //     adc_val = AdcRssiAvgToMilliVolt(ADC_RSSI_CH_HF);
    //
    //     SpinDelay(1);
    // }

    // Gpio_FPGA_SWITCH_High();
    // bool rfON = true;
    // while (1) {
    //     LED_A_ON();
    //     LED_B_ON();
    //     LED_C_ON();
    //     LED_D_ON();
    //     if (!is_btn_pressed()) continue;
    //     if (rfON) {
    //         FpgaWriteConfWord(FPGA_MAJOR_MODE_OFF);
    //     } else {
    //         FpgaWriteConfWord(FPGA_MAJOR_MODE_HF_READER);
    //     }
    //     rfON = !rfON;
    //     LED_C_OFF();
    //     LED_D_OFF();
    //     SpinDelay(500);
    // }

    // ------------------------------- 测试 按下按钮清除外部flash  -------------------------------
    // while (1) {
    //     LED_C_ON();
    //     LED_D_ON();
    //     if (!is_btn_pressed()) continue;
    //     LED_C_OFF();
    //     LED_D_OFF();
    //     LED_A_ON();
    //     if (Flash_WipeMemory()) {
    //         LED_A_OFF();
    //     } else {
    //         LED_A_ON();
    //         LED_B_ON();
    //     }
    // }

    // while (1) {
    //     if (is_btn_pressed()) {
    //         LED_A_ON();
    //     } else {
    //         LED_A_OFF();
    //     }
    // }

    // while (1) {
    //     Gpio_LED_A_High();
    //     SpinDelay(500);
    //     Gpio_LED_A_Low();
    //     SpinDelay(500);
    // }

    // ------------------------------- 测试 utils.c 中封装的按钮检测的逻辑（长按，双击） -------------------------------
    // StartTickCount();
    // while (1) {
    //     volatile uint32_t ticks = GetTickCount();
    //     volatile int click = BUTTON_CLICKED(800); // BUTTON_HELD 或者 BUTTON_CLICKED
    //     ticks = GetTickCount() - ticks;
    //     (void)ticks;
    //     if (click == BUTTON_NO_CLICK) {
    //         SpinDelay(1);
    //     } else if (click == BUTTON_HOLD) {
    //         SpinDelay(2);
    //     } else if (click == BUTTON_DOUBLE_CLICK) {
    //         SpinDelay(3);
    //     } else if (click == BUTTON_SINGLE_CLICK) {
    //         SpinDelay(4);
    //     } else {
    //         SpinDelay(5);
    //     }
    // }

    // ------------------------------- 测试 Flash相关的操作 -------------------------------
    // StartTickCount();
    // FlashInit();
    // while (1) {
    //     // volatile uint8_t sts = Flash_ReadStat1();
    //     // if (1) {
    //     //     (void)sts;
    //     // }
    //
    //     volatile uint8_t flash_uid[8] = {0x01};
    //     Flash_UniqueID((uint8_t *) flash_uid);
    //     if (1) {
    //         (void) flash_uid;
    //         SpinDelay(1);
    //     }
    //
    //     // volatile uint8_t flash_data[256] = { 0x00 };
    //     // __NOP();
    //     // Flash_ReadDataCont(0x00, (uint8_t *)flash_data, sizeof(flash_data));
    //     // if (1) {
    //     //     (void)flash_data;
    //     //     __NOP();
    //     // }
    //
    //     uint8_t flash_rw[255] = {0x00};
    //
    //     Flash_ReadDataCont(0x00, flash_rw, sizeof(flash_rw));
    //     SpinDelay(1);
    //
    //     Flash_WriteEnable();
    //     Flash_Erase4k(0x00, 0x00);
    //     if (Flash_CheckBusy(BUSY_TIMEOUT)) return;
    //
    //     SpinDelay(1000); // wait erase finish
    //     if (Flash_ReadDataCont(0x00, flash_rw, sizeof(flash_rw)) == 0) return;
    //     SpinDelay(1);
    //
    //     for (int i = 0; i < sizeof(flash_rw); i++) flash_rw[i] = i;
    //     Flash_WriteEnable();
    //     if (Flash_WriteDataCont(0x00, flash_rw, sizeof(flash_rw)) == 0) return;
    //     if (Flash_CheckBusy(BUSY_TIMEOUT)) return;
    //     SpinDelay(1000);
    //
    //     memset(flash_rw, 'a', sizeof(flash_rw));
    //     Flash_ReadDataCont(0x00, flash_rw, sizeof(flash_rw));
    //     SpinDelay(1);
    //
    //     // uint8_t flash_rw[256] = { 0x00 };
    //     // Flash_ReadData(0x00, flash_rw, sizeof(flash_rw));
    //     // SpinDelay(1);
    // }


    // ------------------------------- 测试 StartTickCount 和 GetTickCountDelta的精度 -------------------------------
    // StartTickCount();
    // while (1) {
    //     volatile uint32_t tickCount = GetTickCount();
    //     (void)tickCount;
    //     while (GetTickCountDelta(tickCount) != 500) {
    //         /* set pa.01 */
    //         GPIOA->scr = GPIO_PINS_1;
    //     }
    //     while (GetTickCountDelta(tickCount) != 1000) {
    //         /* reset pa.01 */
    //         GPIOA->clr = GPIO_PINS_1;
    //     }
    // }

    // ------------------------------- 测试 SpinDelayUs 的精度 -------------------------------
    // while (1) {
    //     GPIOA->scr = GPIO_PINS_1; // set pa.01
    //     SpinDelayUsPrecision(500);
    //     GPIOA->clr = GPIO_PINS_1; // reset pa.01
    //     SpinDelayUsPrecision(500);
    // }

    // ------------------------------- 测试 StartTicks 的精度 -------------------------------
    //  注意，StartTicks和StartCountUS和StartCountSspClk不能同时使用
    // StartTicks();
    // volatile uint32_t tickCount;
    // while (1) {
    //     // 拉高测试
    //     GPIOA->scr = GPIO_PINS_1; // set pa.01
    //     tickCount = GetTicks();
    //     while (GetTicks() - tickCount < 3) {}  // 1.5tick = 1us, 3tick = 2us, 4.5tick = 3us, 6tick = 4us
    //
    //     // 拉低测试
    //     GPIOA->clr = GPIO_PINS_1; // reset pa.01
    //     tickCount = GetTicks();
    //     while (GetTicks() - tickCount < 3) {}
    // }

    // ------------------------------- 测试 StartTicks 重置计数值 -------------------------------
    //  注意，StartTicks和StartCountUS和StartCountSspClk不能同时使用
    // StartTicks();
    // volatile uint32_t tickCount;
    // while (1) {
    //     tickCount = GetTicks();
    //     (void)tickCount;
    //     SpinDelay(1);
    //     tickCount = GetTicks();
    //     (void)tickCount;
    //     SpinDelay(1);
    //
    //     ResetTicks();
    //     tickCount = GetTicks();
    //     (void)tickCount;
    //     SpinDelay(1);
    // }

    // ------------------------------- 测试 StartCountSspClk 外部输入时钟计数 -------------------------------
    //  注意，StartTicks和StartCountUS和StartCountSspClk不能同时使用
    // StartCountSspClk();
    // volatile uint32_t clkCount;
    // while (1) {
    //     GPIOA->scr = GPIO_PINS_1; // set pa.01
    //     clkCount = GetCountSspClk();
    //     while (GetCountSspClk() == clkCount) {}
    //     GPIOA->clr = GPIO_PINS_1; // reset pa.01
    //     clkCount = GetCountSspClk();
    //     while (GetCountSspClk() == clkCount) {}
    // }

    // ------------------------------- 测试 StartCountUS 的精度 -------------------------------
    //  注意，StartCountUS和StartTicks和和StartCountSspClk不能同时使用
    // StartCountUS();
    // volatile uint32_t usCount;
    // while (1) {
    //     // 拉高测试
    //     GPIOA->scr = GPIO_PINS_1; // set pa.01
    //     usCount = GetCountUS();
    //     while (GetTicks() - usCount < 3) {}
    //
    //     // 拉低测试
    //     GPIOA->clr = GPIO_PINS_1; // reset pa.01
    //     usCount = GetCountUS();
    //     while (GetTicks() - usCount < 3) {}
    // }

    // StartTickCount();
    //
    // uint32_t tStart = GetTickCount();
    // while (GetTickCountDelta(tStart) != 2000); // 等一会儿再启动发送，USB可能还在枚举
    //
    // while (1) {
    //     GPIOC->clr = GPIO_PINS_0; // 熄灯
    //     GPIOC->scr = GPIO_PINS_1; // 熄灯
    //     if (!is_btn_pressed()) {
    //         continue;
    //     }
    //     GPIOC->clr = GPIO_PINS_1; // 熄灯
    //
    //     // ------------------------------- 开始堵塞收发相关的处理逻辑 -------------------------------
    //     uint8_t buffer[150];
    //     while (1) {
    //         uint32_t recv_len = usb_read(buffer, sizeof(buffer)); // 收一串数据
    //         if (recv_len != 0) {
    //             if (usb_write(buffer, recv_len) != PM3_SUCCESS) {
    //                 // 回环测试，发回去
    //                 while (1)
    //                     __NOP(); // send error!!!
    //             }
    //         }
    //     }

    // ------------------------------- 测试USB失能 -------------------------------
    // usb_disable();
    // while (!is_btn_pressed()) {}
    // usb_enable();

    // ------------------------------- 开始异步发送相关的处理逻辑 -------------------------------
    // if (async_usb_write_start() != PM3_SUCCESS) {
    //     GPIOC->scr = GPIO_PINS_0;
    //     while (1) {} // 出错了
    // }
    // while (1) {
    //     tickCount = GetTickCount();
    //     while (GetTickCountDelta(tickCount) <= 1); // 等待一小会儿，模拟数据采集的时间差
    //
    //     // 提交数据到缓冲区，但是先不发送，先堆积着。
    //     for (uint8_t i = 0; i < 64; i++) {
    //         async_usb_write_pushByte(i); // 提交一个字节。
    //     }
    //
    //     // 放完了数据，可以提交写入确认，让HOST在下一次的IN事务中取走数据，并且此函数还切换了双buffer的idx
    //     if (!async_usb_write_requestWrite()) {
    //         GPIOC->scr = GPIO_PINS_0;
    //         while (1) {} // 不能到这里，到这里说明提交速度大于USB的发送速度了
    //     }
    // }
    // // 然后等待结束发送。
    // if (async_usb_write_stop() != PM3_SUCCESS) {
    //     GPIOC->scr = GPIO_PINS_0;
    //     while (1) {} // 不能到这里，到这里说明数据写入有问题。
    // }
}

#endif
