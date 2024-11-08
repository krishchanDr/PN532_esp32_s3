#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C configuration parameters
#define I2C_MASTER_SCL_IO       9               // SCL pin
#define I2C_MASTER_SDA_IO       8               // SDA pin
#define I2C_MASTER_NUM          I2C_NUM_0        // I2C port
#define I2C_MASTER_FREQ_HZ      100000           // I2C frequency
#define I2C_MASTER_TX_BUF_DISABLE 0               // Disable TX buffer
#define I2C_MASTER_RX_BUF_DISABLE 0               // Disable RX buffer
#define PN532_I2C_ADDRESS       0x24             // Default I2C address for PN532
#define PN532_I2C_READY (0x01)        ///< Ready
#define PN532_I2C_READY (0x01)        ///< Ready

// Define the SAMConfig command
uint8_t SAMConfigCmd[] = {0x00, 0xFF, 0x05, 0xFB, 0xD4, 0x14, 0x01, 0x14, 0x01, 0x02, 0x00,};

// Define the command to read a passive target (RFID tag)
uint8_t ReadPassiveTargetCmd[] = {0x0, 0xFF, 0x4, 0xFC, 0xD4, 0x4A, 0x1, 0x0, 0xE1, 0x0};

static const char *TAG = "PN532";

// Initialize the I2C bus
esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000  ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed!");
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed!");
    }
    return err;
}

// Send a command to the PN532 and wait for a response
esp_err_t pn532_send_command(uint8_t *command, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PN532_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, command, length, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// Receive response from the PN532
esp_err_t pn532_receive_response(uint8_t *response, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PN532_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, response, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

bool waitready() {
  uint8_t rdy[1];

  while (1) {
    esp_err_t ret = pn532_receive_response(rdy, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive response from PN532 %s response = %d",__func__,ret);
        //return false;
    }    
    if((rdy[0] == PN532_I2C_READY))
    {
        return true;
    }    
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
  return true;
}

// Send SAMConfig command to the PN532
esp_err_t sam_configure() {
    esp_err_t ret = pn532_send_command(SAMConfigCmd, sizeof(SAMConfigCmd));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send SAMConfig command");
        return ret;
    }
    waitready();
    vTaskDelay(pdMS_TO_TICKS(50)); 

    uint8_t response[16];
    ret = pn532_receive_response(response, 7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive response from PN532");
        return ret;
    }
   #if PN32_DEBUG 
    ESP_LOGI(TAG, "READ ACK");
          for (int i = 0; i < 8; i++) {
            printf( "0x%02X ", response[ i]);  // The UID starts at response[6]
        }
        printf("\n");
    #endif
        ret = pn532_receive_response(response, 7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive response from PN532");
        return ret;
    }
#if PN32_DEBUG 
    ESP_LOGI(TAG, "READ ACK");
          for (int i = 0; i < 8; i++) {
            printf( "0x%02X ", response[ i]);  // The UID starts at response[6]
        }
        printf("\n");
#endif
    ESP_LOGI(TAG, "SAMConfig response: 0x%02X", response[0]);
    return ESP_OK;
}

// Function to read an RFID tag
esp_err_t read_rfid_tag() {
     vTaskDelay(pdMS_TO_TICKS(50)); 

    esp_err_t ret = pn532_send_command(ReadPassiveTargetCmd, sizeof(ReadPassiveTargetCmd));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send read passive target command");
        return ret;
    }
    
    waitready();
    uint8_t response[20];  // Buffer for the response
    ret = pn532_receive_response(response, 7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive response from PN532");
        return ret;
    }
#if PN32_DEBUG 
    ESP_LOGI(TAG, "READ ACK");
          for (int i = 0; i < 7; i++) {
            printf( "0x%02X ", response[ i]);  // The UID starts at response[6]
        }
        printf("\n");
#endif
    waitready();

    ret = pn532_receive_response(response, sizeof(response));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive response from PN532");
        return ret;
    }
        ESP_LOGI(TAG, "tag detected");
          for (int i = 0; i < 20; i++) {
            printf( "0x%02X ", response[ i]);  // The UID starts at response[6]
        }
        printf("\n");
    return ESP_OK;
}

void app_main(void) {
    // Initialize I2C
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return;
    }

    // Configure PN532 SAM
    ret = sam_configure();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SAM configuration failed");
        return;
    }
    while(1)
    {
        // Read RFID tag
        ret = read_rfid_tag();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read RFID tag");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}
