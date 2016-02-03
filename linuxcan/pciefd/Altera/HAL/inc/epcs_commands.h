#include <asm/io.h>

#define epcs_read    0x03
#define epcs_pp      0x02
#define epcs_wren    0x06
#define epcs_wrdi    0x04
#define epcs_rdsr    0x05
#define epcs_wrsr    0x01
#define epcs_se      0xD8
#define epcs_be      0xC7
#define epcs_dp      0xB9
#define epcs_res     0xAB
#define epcs_rdid    0x9F

uint8_t epcs_read_device_id(void *base);
uint8_t epcs_read_electronic_signature(void *base);
uint8_t epcs_read_status_register(void *base);
void epcs_sector_erase(void *base, uint32_t offset);
int32_t epcs_read_buffer(void *base, int offset, uint8_t *dest_addr, int length);
void epcs_write_enable(void *base);
void epcs_write_status_register(void *base, uint8_t value);
int32_t epcs_write_buffer(void *base, int offset, const uint8_t *src_addr, int length);
