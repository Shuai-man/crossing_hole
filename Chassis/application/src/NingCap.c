#include "NingCap.h"

#include <string.h>

SuperCapSendData cap_send_data;
SuperCapRecvData cap_recv_data;
NingCapController cap_controller;
static volatile uint32_t cap_recv_sequence;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

void CapControllerInit(void)
{
    memset(&cap_send_data, 0, sizeof(cap_send_data));
    memset(&cap_recv_data, 0, sizeof(cap_recv_data));
    memset(&cap_controller, 0, sizeof(cap_controller));
    cap_recv_sequence = 0U;
    cap_controller.cap_energy_min =
        0.5f * CAP_MIN_VOL * CAP_MIN_VOL * CAP_CAPACITY;
    cap_controller.cap_energy_max =
        0.5f * CAP_MAX_VOL * CAP_MAX_VOL * CAP_CAPACITY;
    cap_controller.cap_vol_state = CapVol_Low;
}


uint8_t not_use_cap = 0;
void SendCapPack(SuperCapSendData *send_data, float referee_power_limit)
{
    float encoded_power;

    if (send_data == 0)
    {
        return;//无效地址
    }

    encoded_power = clamp_float(referee_power_limit * 100.0f, 0.0f, 125.0f * 100.0f);
    send_data->buffer_energy = 0U;
    send_data->P_ref = (uint16_t)encoded_power;
    send_data->reverse = 0U;
    send_data->wireless_start = not_use_cap;
}

void ReceiveCapDecode(const uint8_t *recv_data, SuperCapRecvData *decoded_data)
{
    if (recv_data == 0 || decoded_data == 0)
    {
        return;//无效地址
    }
    memcpy(decoded_data, recv_data, sizeof(*decoded_data));
    cap_recv_sequence++;
}

void CapVoltageUpdate(void)
{
    if(cap_controller.cap_vol < CAP_VOL_LOW)
    {
        cap_controller.cap_vol_state = CapVol_Low;
    }
    else if(cap_controller.cap_vol > CAP_VOL_HIGH)
    {
        cap_controller.cap_vol_state = CapVol_High;
    }
    else
    {
        // 中间状态切换，防止状态频繁跳变
        if(cap_controller.cap_vol_state == CapVol_Low && cap_controller.cap_vol > CAP_VOL_MID)
        {
            cap_controller.cap_vol_state = CapVol_Middle;
        }
        else if(cap_controller.cap_vol_state == CapVol_High && cap_controller.cap_vol < CAP_VOL_MID)
        {
            cap_controller.cap_vol_state = CapVol_Middle;
        }
    }
}

void NingCapUpdateState(uint8_t online)
{
    float usable_energy_range;

    if (online == 0U)
    {
        cap_controller.cap_vol = 0.0f;
        cap_controller.cap_energy = 0.0f;
        cap_controller.cap_energy_pecent = 0.0f;
        cap_controller.chassis_power = 0.0f;
        cap_controller.referee_power = 0.0f;
        cap_controller.cap_vol_state = CapVol_Low;
        cap_controller.energy_available = 0U;
        cap_controller.power_data_valid = 0U;
        return;
    }

    cap_controller.cap_vol = cap_recv_data.cap_vol / 100.0f;
    cap_controller.chassis_power = cap_recv_data.chassis_power / 100.0f;
    cap_controller.referee_power = cap_recv_data.referee_power / 100.0f;
    cap_controller.power_measurement_sequence = cap_recv_sequence;
    cap_controller.power_data_valid = 1U;
    if (cap_controller.cap_vol > CAP_MAX_VOL + 5.0f)
    {
        cap_controller.cap_energy = 0.0f;
        cap_controller.cap_energy_pecent = 0.0f;
        cap_controller.chassis_power = 0.0f;
        cap_controller.referee_power = 0.0f;
        cap_controller.cap_vol_state = CapVol_Low;
        cap_controller.energy_available = 0U;
        cap_controller.power_data_valid = 0U;
        return;
    }

    cap_controller.cap_energy =
        0.5f * cap_controller.cap_vol * cap_controller.cap_vol * CAP_CAPACITY;
    usable_energy_range =
        cap_controller.cap_energy_max - cap_controller.cap_energy_min;
    if (usable_energy_range > 0.0f)
    {
        cap_controller.cap_energy_pecent =
            clamp_float((cap_controller.cap_energy - cap_controller.cap_energy_min) /
                            usable_energy_range,
                        0.0f,
                        1.0f);
    }
    else
    {
        cap_controller.cap_energy_pecent = 0.0f;
    }

    if (cap_controller.energy_available != 0U)
    {
        if (cap_controller.cap_vol < CAP_USE_DISABLE_VOL)
        {
            cap_controller.energy_available = 0U;
        }
    }
    else if (cap_controller.cap_vol > CAP_USE_ENABLE_VOL)
    {
        cap_controller.energy_available = 1U;
    }

    CapVoltageUpdate();

}

uint8_t NingCapHasEnergy(void)
{
    return cap_controller.energy_available;
}
