//
// Created by christo on 2024/06/20.
//
#ifndef H_NVM23_
#define H_NVM23_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int nvm_set_box(char *boxNo);
int nvm_get_box(char *boxNo, uint32_t len);

void nvm_register_console();

#ifdef __cplusplus
}
#endif


#endif
