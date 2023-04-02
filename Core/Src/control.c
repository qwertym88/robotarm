#include "control.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// 当前机械臂坐标
point_t endpos;
// 当前机械臂角度坐标
angle_t endagl;

/**
 * @brief char*转double
 * @param str 待处理字符串
 * @param len 字符串长度
 * @param res 存储结果
 */
int m_atof(const char *str, int len, double *res)
{
    if (len <= 0)
        return 0;

    double integer = 0, decimal = 0, symbol = 1, fac = 1;
    char flg = 'i';
    int i = 0;

    if (str[0] == '-')
        symbol = -1, ++i;

    for (; i < len; ++i)
    {
        if (str[i] >= '0' && str[i] <= '9')
            if (flg == 'i')
                integer = integer * 10 + str[i] - '0';
            else
                fac *= 0.1, decimal += (str[i] - '0') * fac;
        else if (str[i] == '.' && flg == 'i')
            flg = 'd';
        else
            return 0;
    }

    *res = (integer + decimal) * symbol;
    return 1;
}

/**
 * @brief 解析命令并执行
 * @param str 输入字符串
 * @param len 字符串长度
 */
int parexc(char *str, uint16_t len)
{
    // 调试信息统一在这里输出，所以可能看着挺乱
    // 预留了一个log等级，万一有用呢？

    int i = 0, j = 0;
    while (str[i] != '\0' && str[i] != ' ')
        ++i;
    if (str[i] == '\0')
        return 0;
    str[i++] = '\0';

    uint8_t endline = '\n';

    if (strcmp(str, "MOVP") == 0 || strcmp(str, "MOVA") == 0)
    {

        double r[3] = {0, 0, 0};
        j = i;
        for (int index = 0; index < 3; index++)
        {
            while (str[j] != '\0' && str[j] != ' ')
                ++j;
            if (!m_atof(str + i, j - i, r + index))
                return 0;
            i = ++j;
        }

        if (strcmp(str, "MOVP") == 0)
        {
            point_t dst = {r[0], r[1], r[2]};
            print_info("execute: MOVP ", r[0], r[1], r[2], ";\n");

            uint8_t result = move(endpos, dst, 60);
            if (result == 1)
            {
                // 若移动，更新末端点位置
                endpos = dst;
                endagl = pointToAngle(dst);

                // print result
                m_printf("%c\n", '0' + result);
                // log info
                print_info("Complete. joint angle: ", endagl.j1, endagl.j2, endagl.j3, "\n");
            }
            else
            {
                // print result
                m_printf("%c\n", '0' + result);
                // log info
                print_info("Cannot reach ", r[0], r[1], r[2], ", move failed\n");
            }
        }
        else if (strcmp(str, "MOVA") == 0)
        {
            angle_t dst = {r[0], r[1], r[2]};
            print_info("execute: MOVEA ", r[0], r[1], r[2], "\n");

            uint8_t result = movea_s(endagl, dst);
            if (result == 1)
            {
                // 若移动，更新末端点位置
                endpos = angleToPoint(dst);
                endagl = dst;

                // print result
                m_printf("%c\n", '0' + result);
                // log info
                print_info("Complete. point: ", endpos.x, endpos.y, endpos.z, "\n");
            }
            else
            {
                // print result
                m_printf("%c\n", '0' + result);
                // log info
                print_info("Cannot turn to ", r[0], r[1], r[2], ", move failed\n");
            }
        }
        else
            return 0;
    }
    else if (strcmp(str, "MOVE") == 0)
    {
        double r[4] = {0, 0, 0, 0};
        j = i;
        for (int index = 0; index < 4; index++)
        {
            while (str[j] != '\0' && str[j] != ' ')
                ++j;
            if (!m_atof(str + i, j - i, r + index))
                return 0;
            i = ++j;
        }

        point_t dst = {r[0], r[1], r[2]};
        if (CURRENT_LEVEL <= INFO_LEVEL)
        {
            USART_OUT;
            printf("[info] execute: MOVE ");
            print_float(r[0]);
            print_float(r[1]);
            print_float(r[2]);
            print_float(r[3]);
            printf(";\n");
            USART_IN;
        }
        uint8_t result = move(endpos, dst, r[3]);
        if (result == 1)
        {
            // 若移动，更新末端点位置
            endpos = dst;
            endagl = pointToAngle(dst);

            // print result
            m_printf("%c\n", '0' + result);
            // log info
            print_info("Complete. joint angle: ", endagl.j1, endagl.j2, endagl.j3, "\n");
        }
        else
        {
            // print result
            m_printf("%c\n", '0' + result);
            // log info
            print_info("Cannot reach ", r[0], r[1], r[2], ", move failed\n");
        }
    }
    else if (strcmp(str, "CURVE") == 0)
    {
        double r[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        j = i;
        for (int index = 0; index < 13; index++)
        {
            while (str[j] != '\0' && str[j] != ' ')
                ++j;
            if (!m_atof(str + i, j - i, r + index))
                return 0;
            i = ++j;
        }

        point_t ori = {r[0], r[1], r[2]};
        point_t dst = {r[3], r[4], r[5]};
        point_t op = {r[6], r[7], r[8]};
        point_t dp = {r[9], r[10], r[11]};

        uint8_t result = curve(ori, dst, op, dp, r[12]);
        if (result == 1)
        {
            // 若移动，更新末端点位置
            endpos = dst;
            endagl = pointToAngle(dst);

            // print result
            m_printf("%c\n", '0' + result);
            // log info
            // print_info("Complete. joint angle: ", endagl.j1, endagl.j2, endagl.j3, "\n");
        }
        else
        {
            // print result
            m_printf("%c\n", '0' + result);
            // log info
            // print_info("Cannot reach ", r[0], r[1], r[2], ", move failed\n");
        }
    }
    else if (strcmp(str, "PUMP") == 0)
    {
        if (strcmp(str + i, "ON") == 0)
        {
            uint8_t result = pump_on();

            // print result
            m_printf("%c\n", '0' + result);
            // log info
            if (CURRENT_LEVEL <= INFO_LEVEL)
                m_printf("[info] pump on%c", '\n');
        }
        else if (strcmp(str + i, "OFF") == 0)
        {
            uint8_t result = pump_off();

            // print result
            m_printf("%c\n", '0' + result);
            // log info
            if (CURRENT_LEVEL <= INFO_LEVEL)
                m_printf("[info] pump off%c", '\n');
        }
        else if (strcmp(str + i, "?") == 0)
        {
            uint8_t result = pump_status();

            // print result
            m_printf("%c\n", '0' + result);
            // log info
            if (CURRENT_LEVEL <= INFO_LEVEL)
            {
                if (result == 1)
                    m_printf("[info] pump is working%c", '\n');
                else
                    m_printf("[info] pump is turned off%c", '\n');
            }
        }
        else
            return 0;
    }
    else if (strcmp(str, "GET") == 0)
    {
        if (strcmp(str + i, "POS") == 0)
        {
            USART_OUT;
            // print result
            HAL_UART_Transmit(&huart2, (uint8_t *)&endpos, sizeof(point_t), 0xffff);
            HAL_UART_Transmit(&huart2, &endline, sizeof(uint8_t), 0xffff);
            HAL_Delay(50);
            USART_IN;
            // log info
            if (CURRENT_LEVEL <= INFO_LEVEL)
                m_printf("[info] get position%c", '\n');
        }
        else if (strcmp(str + i, "AGL") == 0)
        {
            USART_OUT;
            // print result
            HAL_UART_Transmit(&huart2, (uint8_t *)&endagl, sizeof(angle_t), 0xffff);
            HAL_UART_Transmit(&huart2, &endline, sizeof(uint8_t), 0xffff);
            HAL_Delay(50);
            USART_IN;
            // log info
            if (CURRENT_LEVEL <= INFO_LEVEL)
                m_printf("[info] get angle%c", '\n');
        }
        else
            return 0;
    }
    else
        return 0;
    return 1;
}
