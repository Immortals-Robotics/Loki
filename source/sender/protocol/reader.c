#include "reader.h"

#include <string.h>
#include "defines.h"
#include "half.h"

void read_bytes(const uint8_t *const buffer, size_t *const pos, void *const data, const size_t count)
{
    memcpy(data, buffer + (*pos), count);
    *pos += count;
}

void read_bits8(const uint8_t *const buffer, size_t *const pos, struct bits8_t *const data)
{
    uint8_t bits_data;
    read_uint8(buffer, pos, &bits_data);
    data->bit0 = bits_data & 0x01;
    data->bit1 = bits_data >> 1 & 0x01;
    data->bit2 = bits_data >> 2 & 0x01;
    data->bit3 = bits_data >> 3 & 0x01;
    data->bit4 = bits_data >> 4 & 0x01;
    data->bit5 = bits_data >> 5 & 0x01;
    data->bit6 = bits_data >> 6 & 0x01;
    data->bit7 = bits_data >> 7 & 0x01;
}

void read_bits16(const uint8_t *const buffer, size_t *const pos, struct bits16_t *const data)
{
    uint16_t bits_data;
    read_uint16(buffer, pos, &bits_data);
    data->bit0  = bits_data & 0x01;
    data->bit1  = bits_data >> 1 & 0x01;
    data->bit2  = bits_data >> 2 & 0x01;
    data->bit3  = bits_data >> 3 & 0x01;
    data->bit4  = bits_data >> 4 & 0x01;
    data->bit5  = bits_data >> 5 & 0x01;
    data->bit6  = bits_data >> 6 & 0x01;
    data->bit7  = bits_data >> 7 & 0x01;
    data->bit8  = bits_data >> 8 & 0x01;
    data->bit9  = bits_data >> 9 & 0x01;
    data->bit10 = bits_data >> 10 & 0x01;
    data->bit11 = bits_data >> 11 & 0x01;
    data->bit12 = bits_data >> 12 & 0x01;
    data->bit13 = bits_data >> 13 & 0x01;
    data->bit14 = bits_data >> 14 & 0x01;
    data->bit15 = bits_data >> 15 & 0x01;
}

void read_bits32(const uint8_t *const buffer, size_t *const pos, struct bits32_t *const data)
{
    uint32_t bits_data;
    read_uint32(buffer, pos, &bits_data);
    data->bit0  = bits_data & 0x01;
    data->bit1  = bits_data >> 1 & 0x01;
    data->bit2  = bits_data >> 2 & 0x01;
    data->bit3  = bits_data >> 3 & 0x01;
    data->bit4  = bits_data >> 4 & 0x01;
    data->bit5  = bits_data >> 5 & 0x01;
    data->bit6  = bits_data >> 6 & 0x01;
    data->bit7  = bits_data >> 7 & 0x01;
    data->bit8  = bits_data >> 8 & 0x01;
    data->bit9  = bits_data >> 9 & 0x01;
    data->bit10 = bits_data >> 10 & 0x01;
    data->bit11 = bits_data >> 11 & 0x01;
    data->bit12 = bits_data >> 12 & 0x01;
    data->bit13 = bits_data >> 13 & 0x01;
    data->bit14 = bits_data >> 14 & 0x01;
    data->bit15 = bits_data >> 15 & 0x01;
    data->bit16 = bits_data >> 16 & 0x01;
    data->bit17 = bits_data >> 17 & 0x01;
    data->bit18 = bits_data >> 18 & 0x01;
    data->bit19 = bits_data >> 19 & 0x01;
    data->bit20 = bits_data >> 20 & 0x01;
    data->bit21 = bits_data >> 21 & 0x01;
    data->bit22 = bits_data >> 22 & 0x01;
    data->bit23 = bits_data >> 23 & 0x01;
    data->bit24 = bits_data >> 24 & 0x01;
    data->bit25 = bits_data >> 25 & 0x01;
    data->bit26 = bits_data >> 26 & 0x01;
    data->bit27 = bits_data >> 27 & 0x01;
    data->bit28 = bits_data >> 28 & 0x01;
    data->bit29 = bits_data >> 29 & 0x01;
    data->bit30 = bits_data >> 30 & 0x01;
    data->bit31 = bits_data >> 31 & 0x01;
}

void read_uint8(const uint8_t *const buffer, size_t *const pos, uint8_t *const data)
{
    *data = buffer[*pos];
    *pos += 1;
}

void read_uint16(const uint8_t *const buffer, size_t *const pos, uint16_t *const data)
{
    *data = (buffer[*pos]) | (buffer[*pos + 1] << 8);

    *pos += 2;
}

void read_uint16_from_buff(const uint8_t *const buffer, uint16_t *const data)
{
    *data = (buffer[0]) | (buffer[1] << 8);
}

void read_uint32(const uint8_t *const buffer, size_t *const pos, uint32_t *const data)
{
    *data = (buffer[*pos]) | (buffer[*pos + 1] << 8) | (buffer[*pos + 2] << 16) | (buffer[*pos + 3] << 24);

    *pos += 4;
}

void read_float_h(const uint8_t *const buffer, size_t *const pos, union FLOAT_32 *const data)
{
    uint16_t h_data;
    read_uint16(buffer, pos, &h_data);
    data->u32 = half_to_float(h_data);
}

void read_float_h_from_2x_buf(const uint8_t *const buffer, float *const float_out)
{
    uint16_t       h_data;
    union FLOAT_32 data;
    read_uint16_from_buff(buffer, &h_data);
    data.u32   = half_to_float(h_data);
    *float_out = data.f32;
}

void read_float(const uint8_t *const buffer, size_t *const pos, union FLOAT_32 *const data)
{
    read_uint32(buffer, pos, &data->u32);
}

void read_v2f(const uint8_t *const buffer, size_t *const pos, struct Vector2f_V2 *const data)
{
    read_float(buffer, pos, &data->x);
    read_float(buffer, pos, &data->y);
}

void read_v2f_h(const uint8_t *const buffer, size_t *const pos, struct Vector2f_V2 *const data)
{
    read_float_h(buffer, pos, &data->x);
    read_float_h(buffer, pos, &data->y);
}

void read_v3f(const uint8_t *const buffer, size_t *const pos, struct Vector3f *const data)
{
    read_float(buffer, pos, &data->x);
    read_float(buffer, pos, &data->y);
    read_float(buffer, pos, &data->z);
}

void read_v3f_h(const uint8_t *const buffer, size_t *const pos, struct Vector3f *const data)
{
    read_float_h(buffer, pos, &data->x);
    read_float_h(buffer, pos, &data->y);
    read_float_h(buffer, pos, &data->z);
}

void read_v4f(const uint8_t *const buffer, size_t *const pos, struct Vector4f *const data)
{
    read_float(buffer, pos, &data->x);
    read_float(buffer, pos, &data->y);
    read_float(buffer, pos, &data->z);
    read_float(buffer, pos, &data->w);
}

void read_v4f_h(const uint8_t *const buffer, size_t *const pos, struct Vector4f *const data)
{
    read_float_h(buffer, pos, &data->x);
    read_float_h(buffer, pos, &data->y);
    read_float_h(buffer, pos, &data->z);
    read_float_h(buffer, pos, &data->w);
}

uint8_t read_robot_command_fixed(const uint8_t *const buffer, const size_t size, struct RobotCommand *const data)
{
    size_t pos = 0;

    uint8_t header;
    read_uint8(buffer, &pos, &header);
    if (header != MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_COMMAND))
        return PARSE_RESULT_HEADER_CORRUPTED;

    read_v2f_h(buffer, &pos, &data->velocity);

    read_float_h(buffer, &pos, &data->omega);
    read_float_h(buffer, &pos, &data->target_orientation);

    read_float_h(buffer, &pos, &data->orientation);

    read_float_h(buffer, &pos, &data->shoot_power);
    read_float_h(buffer, &pos, &data->dribbler);
    read_float_h(buffer, &pos, &data->servo);

    read_uint8(buffer, &pos, &data->beep);

    uint8_t packed_data;
    read_uint8(buffer, &pos, &packed_data);
    data->shoot_type      = (enum ShootType)(packed_data & 0x01);
    data->feedback        = (enum FeedbackRequestType)((packed_data >> 1) & 0x03);
    data->halt            = (packed_data >> 3) & 0x01;
    data->has_orientation = (packed_data >> 4) & 0x01;

    return (pos == size ? PARSE_RESULT_SUCCESS : PARSE_RESULT_SIZE_MISMATCH);
}

uint8_t read_robot_command_fixed_V2(const uint8_t *const buffer, const size_t size, struct RobotCommand_V2 *const data)
{
    size_t pos = 0;

    uint8_t header;
    read_uint8(buffer, &pos, &header);
    if (header != MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_COMMAND))
        return PARSE_RESULT_HEADER_CORRUPTED;

    read_v2f_h(buffer, &pos, &data->velocity);

    read_float_h(buffer, &pos, &data->omega);
    read_float_h(buffer, &pos, &data->target_orientation);

    read_float_h(buffer, &pos, &data->orientation);

    union FLOAT_32 power;
    read_float_h(buffer, &pos, &power);
    data->direct_power = (uint8_t) power.f32;
    read_float_h(buffer, &pos, &power);
    data->chip_power = (uint8_t) power.f32;
    read_float_h(buffer, &pos, &data->dribbler);

    uint8_t packed_data;
    read_uint8(buffer, &pos, &packed_data);
    data->feedback_request = (enum FeedbackRequestType)((packed_data >> 1) & 0x03);
    data->halt             = (packed_data >> 3) & 0x01;
    data->has_orientation  = (packed_data >> 4) & 0x01;

    return (pos == size ? PARSE_RESULT_SUCCESS : PARSE_RESULT_SIZE_MISMATCH);
}

uint8_t read_robot_config_fixed(const uint8_t *const buffer, const size_t size, struct RobotConfig *const data)
{
    size_t pos = 0;

    uint8_t header;
    read_uint8(buffer, &pos, &header);
    if (header != MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_CONFIG))
        return PARSE_RESULT_HEADER_CORRUPTED;

    read_float_h(buffer, &pos, &data->kp);
    read_float_h(buffer, &pos, &data->ki);
    read_float_h(buffer, &pos, &data->kd);
    read_float_h(buffer, &pos, &data->i_limit);

    read_float_h(buffer, &pos, &data->head_offset);

    read_v3f_h(buffer, &pos, &data->direct_coeffs);
    read_v3f_h(buffer, &pos, &data->chip_coeffs);

    return (pos == size ? PARSE_RESULT_SUCCESS : PARSE_RESULT_SIZE_MISMATCH);
}

uint8_t read_robot_matrix_fixed(const uint8_t *const buffer, const size_t size, struct RobotMatrix *const data)
{
    size_t pos = 0;

    uint8_t header;
    read_uint8(buffer, &pos, &header);
    if (header != MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_MATRIX))
        return PARSE_RESULT_HEADER_CORRUPTED;

    read_v3f_h(buffer, &pos, &data->matrix[0]);
    read_v3f_h(buffer, &pos, &data->matrix[1]);
    read_v3f_h(buffer, &pos, &data->matrix[2]);
    read_v3f_h(buffer, &pos, &data->matrix[3]);

    return (pos == size ? PARSE_RESULT_SUCCESS : PARSE_RESULT_SIZE_MISMATCH);
}

uint8_t read_robot_feedback_fixed(const uint8_t *const buffer, const size_t size, struct RobotFeedback *const data)
{
    size_t pos = 0;

    uint8_t header;
    read_uint8(buffer, &pos, &header);
    if (header != MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_FEEDBACK))
        return PARSE_RESULT_HEADER_CORRUPTED;

    read_float_h(buffer, &pos, &data->battery_voltage);
    read_float_h(buffer, &pos, &data->capacitor_voltage);

    read_float_h(buffer, &pos, &data->omega);
    read_float_h(buffer, &pos, &data->orientation);

    read_v4f_h(buffer, &pos, &data->motor_velocity);
    read_v4f_h(buffer, &pos, &data->motor_target);

    read_bits8(buffer, &pos, &data->motor_fault);
    read_bits8(buffer, &pos, &data->button_status);

    uint8_t packed_data;
    read_uint8(buffer, &pos, &packed_data);

    data->fault              = packed_data & 0x01;
    data->ball_detected      = (packed_data >> 1) & 0x01;
    data->booster_enabled    = (packed_data >> 2) & 0x01;
    data->dribbler_connected = (packed_data >> 3) & 0x01;

    return (pos == size ? PARSE_RESULT_SUCCESS : PARSE_RESULT_SIZE_MISMATCH);
}

uint8_t read_robot_feedback_custom_fixed(const uint8_t *const buffer, const size_t size,
                                         struct RobotFeedbackCustom *const data)
{
    size_t pos = 0;

    uint8_t header;
    read_uint8(buffer, &pos, &header);
    if (header != MESSAGE_HEADER(PROTO_VERSION_FIXED, TYPE_FEEDBACK_CUSTOM))
        return PARSE_RESULT_HEADER_CORRUPTED;

    uint8_t data_length;
    read_uint8(buffer, &pos, &data_length);
    data->length = data_length;
    read_bytes(buffer, &pos, &data->debug_dump, data->length);

    return (pos == size ? PARSE_RESULT_SUCCESS : PARSE_RESULT_SIZE_MISMATCH);
}
