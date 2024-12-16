
#pragma once
#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    uint8_t start[2];      
    uint32_t id;           
    int64_t time_delta;    
    float cir[2];          
    uint8_t end[2];        
} __attribute__((packed)) csi_data_t;

void init_uart(void);
int uart_send_data(const char *data, uint8_t len);

#ifdef __cplusplus
}
#endif