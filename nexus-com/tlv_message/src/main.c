#include <stdio.h>
#include "tlv_message.h"
#include "crc32.h"
#include "inet.h"

int main(void) 
{
    printf("Hello world\n");

    message_t msg;
    message_init(&msg);

    uint8_t my_cmd = COMMAND_MOVE_MOTOR;
    message_tlv_add(&msg, TLV_COMMAND, sizeof(uint8_t), &my_cmd);

    tlv_motor_position_t m1_pos = {10, 20, -15};
    m1_pos.x = htonl(m1_pos.x);
    m1_pos.y = htonl(m1_pos.y);
    m1_pos.z = htonl(m1_pos.z);
    message_tlv_add(&msg, TLV_MOTOR_POSITION, sizeof(tlv_motor_position_t), (uint8_t *) &m1_pos);

    message_tlv_add_checksum(&msg);
    message_print(&msg);
    // printf("\nCheck sum value: %d\n", checksum);
    return 0;
}