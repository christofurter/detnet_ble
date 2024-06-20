//
// Created by christo on 2024/06/20.
//

#include <nvs.h>
#include <esp_log.h>
#include <esp_console.h>
#include "nvm.h"

#define tag "nvm.c"

#define NVM_NVS_NAMESPACE   "nvm_cfg"
#define NVM_NVS_KEY_BOX_NO  "box"


int nvm_set_box(char *boxNo) {
    int rc = -1;
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open(NVM_NVS_NAMESPACE, NVS_READWRITE, &nvsHandle);
    if(err == ESP_OK) {
        err = nvs_set_str(nvsHandle, NVM_NVS_KEY_BOX_NO, boxNo);
        nvs_close(nvsHandle);
        if(err == ESP_OK) {
            printf("Set box: %s\n", boxNo);
            rc = 0;
        } else {
            printf("box number failed to save found\n");
            rc = -3;
        }
    } else {
        printf("unable to open handle\n");
        rc = -2;
    }
    return rc;
}

int nvm_get_box(char *boxNo, uint32_t len) {
    nvs_handle_t nvsHandle;
    int rc = -1;
    esp_err_t err = nvs_open(NVM_NVS_NAMESPACE, NVS_READONLY, &nvsHandle);
    if(err == ESP_OK) {
        size_t rxLen = len;
        err = nvs_get_str(nvsHandle, NVM_NVS_KEY_BOX_NO, boxNo, &rxLen);
        nvs_close(nvsHandle);
        if(err == ESP_OK) {
            printf("Found value[%d]: %s\n", rxLen, boxNo);
            rc = (int)rxLen;
        } else {
            printf("no saved box number found\n");
        }
    } else {
        printf("unable to open handle\n");
    }
    return rc;
}

static int debug_boxNo(int argc, char **argv)
{
    ESP_LOGI(tag, "BoxNo maybe working?, lol!\n");
//    printf("%" PRIu32 "\n", esp_get_free_heap_size());
    if(argc == 1) {
        char box[32] = {0};
        int len = nvm_get_box(box, 32);
        if(len > 0) {
            printf("BOX [%d]= %s\n", len, box);
        }
    }
    if(argc == 2) {
        int rc = nvm_set_box(argv[1]);
        printf("update box (%s) : %d\n", argv[1], rc);
    }
    return 0;
}

static void register_BoxNo(void)
{
    const esp_console_cmd_t cmd = {
            .command = "box",
            .help = "Get or set the box number",
            .hint = NULL,
            .func = &debug_boxNo,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

void nvm_register_console() {
    register_BoxNo();
}