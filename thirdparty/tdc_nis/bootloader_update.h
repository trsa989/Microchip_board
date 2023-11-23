
#ifndef BOOTLOADER_UPDATE_H
#define BOOTLOADER_UPDATE_H

/* System includes */


#ifdef __cplusplus
extern "C" {
#endif

void CheckVersion();
void ReplaceBootloader();
void JumpToBootloader();


#ifdef __cplusplus
}
#endif
#endif /* USI_H_INCLUDE */